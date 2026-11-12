#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rb10_rmpflow_rviz/ur10_model.hpp"
#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"

namespace ur10_intent_admittance_rmp_v2
{

class AdmittanceExternalRmp final : public rclcpp::Node
{
public:
  using Model = rb10_rmpflow_rviz::UR10Model;
  using JointVector = Model::JointVector;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;

  AdmittanceExternalRmp()
  : Node("admittance_external_rmp")
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 125.0);
    virtual_mass_ = vector3_parameter("virtual_mass", {3.0, 3.0, 3.0});
    damping_ = vector3_parameter("contact_damping", {45.0, 45.0, 45.0});
    metric_weight_ = vector3_parameter("contact_metric", {200.0, 200.0, 200.0});
    const double contact_metric_scale =
      declare_parameter<double>("contact_metric_scale", 1.0);
    if (!std::isfinite(contact_metric_scale) || contact_metric_scale <= 0.0) {
      throw std::invalid_argument("contact_metric_scale must be finite and positive");
    }
    metric_weight_ *= contact_metric_scale;
    enter_force_n_ = declare_parameter<double>("enter_force_n", 8.0);
    release_force_n_ = declare_parameter<double>("release_force_n", 3.0);
    enter_probability_ = declare_parameter<double>("enter_probability", 0.60);
    release_probability_ = declare_parameter<double>("release_probability", 0.35);
    transition_sec_ = declare_parameter<double>("transition_sec", 0.60);
    joint_state_timeout_sec_ = declare_parameter<double>("joint_state_timeout_sec", 0.10);
    wrench_timeout_sec_ = declare_parameter<double>("wrench_timeout_sec", 0.25);
    intent_timeout_sec_ = declare_parameter<double>("intent_timeout_sec", 0.50);
    max_force_n_ = declare_parameter<double>("max_force_n", 30.0);
    max_task_accel_m_s2_ = declare_parameter<double>("max_task_accel_m_s2", 2.0);
    max_joint_accel_per_joint_rad_s2_ = vector6_parameter(
      "max_joint_accel_per_joint_rad_s2", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    singular_value_damping_ratio_ =
      declare_parameter<double>("singular_value_damping_ratio", 0.05);
    minimum_task_alignment_cosine_ =
      declare_parameter<double>("minimum_task_alignment_cosine", 0.50);
    eigenvalue_floor_ = declare_parameter<double>("eigenvalue_floor", 1e-8);
    const auto joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "/UR10_right/joint_states");
    const auto contact_point_topic = declare_parameter<std::string>(
      "contact_point_topic", "/admittance_rmp/contact_point");
    const auto wrench_topic = declare_parameter<std::string>(
      "wrench_topic", "/admittance_rmp/wrench");
    const auto intent_topic = declare_parameter<std::string>(
      "intent_probability_topic", "/worker_intent/compliance_probability");
    const auto output_prefix = declare_parameter<std::string>(
      "output_prefix", "/escape_on/external_rmp/admittance");

    if (
      publish_rate_hz <= 0.0 || transition_sec_ <= 0.0 ||
      enter_force_n_ <= release_force_n_ || release_force_n_ < 0.0 ||
      enter_probability_ <= release_probability_ || release_probability_ < 0.0 ||
      enter_probability_ > 1.0 || joint_state_timeout_sec_ <= 0.0 ||
      wrench_timeout_sec_ <= 0.0 || intent_timeout_sec_ <= 0.0 ||
      max_force_n_ <= 0.0 || !std::isfinite(max_task_accel_m_s2_) ||
      max_task_accel_m_s2_ <= 0.0 ||
      (max_joint_accel_per_joint_rad_s2_.array() <= 0.0).any() ||
      !std::isfinite(singular_value_damping_ratio_) || singular_value_damping_ratio_ < 0.0 ||
      !std::isfinite(minimum_task_alignment_cosine_) ||
      minimum_task_alignment_cosine_ < 0.0 || minimum_task_alignment_cosine_ > 1.0 ||
      eigenvalue_floor_ <= 0.0 || (virtual_mass_.array() <= 0.0).any() ||
      (damping_.array() < 0.0).any() || (metric_weight_.array() < 0.0).any())
    {
      throw std::invalid_argument("invalid admittance external-RMP parameters");
    }

    metric_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      output_prefix + "/metric_sqrt", rclcpp::QoS(10));
    acceleration_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      output_prefix + "/acceleration", rclcpp::QoS(10));
    raw_acceleration_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      output_prefix + "/acceleration_raw", rclcpp::QoS(10));
    task_acceleration_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      output_prefix + "/task_acceleration_limited", rclcpp::QoS(10));
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/admittance_rmp/integration_mode", rclcpp::QoS(10));
    activation_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/admittance_rmp/activation", rclcpp::QoS(10));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {on_joint_state(*msg);});
    contact_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      contact_point_topic, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {on_contact_point(*msg);});
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {on_wrench(*msg);});
    intent_sub_ = create_subscription<std_msgs::msg::Float64>(
      intent_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {on_intent(*msg);});
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/admittance_rmp/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::scoped_lock lock(mutex_);
        active_ = false;
        activation_phase_ = 0.0;
        have_contact_point_ = false;
        response->success = true;
        response->message = "admittance leaf reset to zero";
      });

    nominal_dt_ = 1.0 / publish_rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(nominal_dt_), [this]() {publish_external_rmp();});

    RCLCPP_WARN(
      get_logger(),
      "EXPERIMENTAL seventh RMP leaf ready. It publishes no robot command; "
      "the existing guarded controller remains the sole command publisher.");
  }

private:
  Eigen::Vector3d vector3_parameter(
    const std::string & name, const std::vector<double> & defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != 3U) {
      throw std::invalid_argument(name + " must contain three values");
    }
    const Eigen::Vector3d result(values[0], values[1], values[2]);
    if (!result.allFinite()) {
      throw std::invalid_argument(name + " must be finite");
    }
    return result;
  }

  JointVector vector6_parameter(
    const std::string & name, const std::vector<double> & defaults)
  {
    const auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != 6U) {
      throw std::invalid_argument(name + " must contain six values");
    }
    JointVector result;
    for (int joint = 0; joint < result.size(); ++joint) {
      result[joint] = values[static_cast<std::size_t>(joint)];
    }
    if (!result.allFinite()) {
      throw std::invalid_argument(name + " must be finite");
    }
    return result;
  }

  static std::size_t link_index(const std::string & frame)
  {
    for (std::size_t index = 0; index < Model::LINK_COUNT; ++index) {
      if (frame == Model::link_names[index]) {
        return index;
      }
    }
    return Model::LINK_COUNT;
  }

  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    JointVector q = JointVector::Zero();
    JointVector qd = JointVector::Zero();
    for (std::size_t joint = 0; joint < Model::joint_names.size(); ++joint) {
      const auto found = std::find(msg.name.begin(), msg.name.end(), Model::joint_names[joint]);
      if (found == msg.name.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "joint_states is missing required UR10 joint %s", Model::joint_names[joint]);
        return;
      }
      const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), found));
      if (index >= msg.position.size()) {
        return;
      }
      q[static_cast<int>(joint)] = msg.position[index];
      if (index < msg.velocity.size()) {
        qd[static_cast<int>(joint)] = msg.velocity[index];
      }
    }
    if (!q.allFinite() || !qd.allFinite()) {
      return;
    }
    std::scoped_lock lock(mutex_);
    q_ = q;
    qd_ = qd;
    joint_state_received_at_ = now();
    have_joint_state_ = true;
  }

  void on_contact_point(const geometry_msgs::msg::PointStamped & msg)
  {
    const auto link = link_index(msg.header.frame_id);
    const Eigen::Vector3d point(msg.point.x, msg.point.y, msg.point.z);
    if (link >= Model::LINK_COUNT || !point.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "contact point must use a UR10 link frame and finite link-local coordinates");
      return;
    }
    std::scoped_lock lock(mutex_);
    contact_link_ = link;
    contact_point_local_ = point;
    have_contact_point_ = true;
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped & msg)
  {
    if (msg.header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "wrench must be expressed in base_link");
      return;
    }
    Eigen::Vector3d force(
      msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    if (!force.allFinite()) {
      return;
    }
    const double norm = force.norm();
    if (norm > max_force_n_) {
      force *= max_force_n_ / norm;
    }
    std::scoped_lock lock(mutex_);
    force_world_ = force;
    wrench_received_at_ = now();
    have_wrench_ = true;
  }

  void on_intent(const std_msgs::msg::Float64 & msg)
  {
    if (!std::isfinite(msg.data)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    compliance_probability_ = std::clamp(msg.data, 0.0, 1.0);
    intent_received_at_ = now();
    have_intent_ = true;
  }

  static double smoothstep(double value)
  {
    const double x = std::clamp(value, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  void publish_zero(const std::string & mode)
  {
    std_msgs::msg::Float64MultiArray metric;
    metric.data.assign(36U, 0.0);
    std_msgs::msg::Float64MultiArray acceleration;
    acceleration.data.assign(6U, 0.0);
    std_msgs::msg::Float64MultiArray task_acceleration;
    task_acceleration.data.assign(3U, 0.0);
    metric_pub_->publish(metric);
    acceleration_pub_->publish(acceleration);
    raw_acceleration_pub_->publish(acceleration);
    task_acceleration_pub_->publish(task_acceleration);
    // Target arbitration must follow the RMP that was actually admitted to
    // the root solve.  A requested contact transition is not sufficient: the
    // leaf may still be rejected for stale inputs, singular direction, or a
    // computation failure.  In every zero-output path keep the target fully
    // authoritative by publishing zero effective activation.
    std_msgs::msg::Float64 activation;
    activation.data = 0.0;
    activation_pub_->publish(activation);
    std_msgs::msg::String status;
    status.data = mode;
    mode_pub_->publish(status);
  }

  void publish_external_rmp()
  {
    JointVector q;
    JointVector qd;
    Eigen::Vector3d force;
    Eigen::Vector3d point;
    std::size_t link = Model::LINK_COUNT;
    double probability = 0.0;
    bool valid = false;
    double transition = 0.0;
    std::string invalid_reason = "inactive";
    const auto current_time = now();

    {
      std::scoped_lock lock(mutex_);
      const bool fresh_joint = have_joint_state_ &&
        (current_time - joint_state_received_at_).seconds() <= joint_state_timeout_sec_;
      const bool fresh_wrench = have_wrench_ &&
        (current_time - wrench_received_at_).seconds() <= wrench_timeout_sec_;
      const bool fresh_intent = have_intent_ &&
        (current_time - intent_received_at_).seconds() <= intent_timeout_sec_;
      const double force_norm = fresh_wrench ? force_world_.norm() : 0.0;
      const double p = fresh_intent ? compliance_probability_ : 0.0;

      if (!active_ && have_contact_point_ && fresh_joint && fresh_wrench && fresh_intent &&
        force_norm >= enter_force_n_ && p >= enter_probability_)
      {
        active_ = true;
      } else if (active_ &&
        (!have_contact_point_ || !fresh_joint || !fresh_wrench || !fresh_intent ||
        force_norm <= release_force_n_ || p <= release_probability_))
      {
        active_ = false;
      }

      const double target = active_ ? 1.0 : 0.0;
      const double max_delta = nominal_dt_ / transition_sec_;
      activation_phase_ += std::clamp(target - activation_phase_, -max_delta, max_delta);
      activation_phase_ = std::clamp(activation_phase_, 0.0, 1.0);

      if (!fresh_joint) {
        invalid_reason = "stale_joint_state";
      } else if (!fresh_wrench) {
        invalid_reason = "stale_wrench";
      } else if (!fresh_intent) {
        invalid_reason = "stale_intent";
      } else if (!have_contact_point_) {
        invalid_reason = "no_contact_point";
      } else if (activation_phase_ <= 1e-6) {
        invalid_reason = "inactive";
      } else {
        q = q_;
        qd = qd_;
        force = force_world_;
        point = contact_point_local_;
        link = contact_link_;
        probability = p;
        valid = true;
      }
      transition = valid ? smoothstep(activation_phase_) * probability : 0.0;
    }

    if (!valid) {
      publish_zero(invalid_reason);
      return;
    }

    try {
      const auto geometry = contact_geometry(q, qd, link, point);

      // Limit the requested task acceleration before pullback.  This preserves
      // the measured force direction, unlike independent joint clipping.
      const Eigen::Vector3d raw_task_acceleration = force.cwiseQuotient(virtual_mass_);
      const Eigen::Vector3d limited_task_acceleration = direction_preserving_task_limit(
        raw_task_acceleration, max_task_accel_m_s2_);
      const Eigen::Vector3d limited_force =
        limited_task_acceleration.cwiseProduct(virtual_mass_);
      const auto pair = contact_natural_pair(
        geometry, limited_force, virtual_mass_, damping_, metric_weight_, transition);

      Eigen::SelfAdjointEigenSolver<Matrix6> eigensolver(pair.metric);
      if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("contact metric eigendecomposition failed");
      }
      const auto eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0);
      const Matrix6 basis = eigensolver.eigenvectors();
      Matrix6 metric_sqrt = basis * eigenvalues.cwiseSqrt().asDiagonal() * basis.transpose();
      const JointVector raw_acceleration = resolve_pair_damped(
        pair, singular_value_damping_ratio_, eigenvalue_floor_);
      const JointVector acceleration = direction_preserving_joint_limit(
        raw_acceleration, max_joint_accel_per_joint_rad_s2_);
      const Eigen::Vector3d achieved_task_acceleration = geometry.jacobian * acceleration;
      const double alignment = task_alignment_cosine(
        limited_task_acceleration, achieved_task_acceleration);
      if (alignment < minimum_task_alignment_cosine_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "admittance direction suppressed near singular geometry: alignment=%.3f < %.3f",
          alignment, minimum_task_alignment_cosine_);
        publish_zero("singular_direction_blocked");
        return;
      }
      if (!metric_sqrt.allFinite() || !acceleration.allFinite()) {
        throw std::runtime_error("non-finite external RMP output");
      }

      std_msgs::msg::Float64MultiArray metric_msg;
      metric_msg.data.reserve(36U);
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          metric_msg.data.push_back(metric_sqrt(row, col));
        }
      }
      std_msgs::msg::Float64MultiArray acceleration_msg;
      acceleration_msg.data.reserve(6U);
      std_msgs::msg::Float64MultiArray raw_acceleration_msg;
      raw_acceleration_msg.data.reserve(6U);
      for (int joint = 0; joint < 6; ++joint) {
        acceleration_msg.data.push_back(acceleration[joint]);
        raw_acceleration_msg.data.push_back(raw_acceleration[joint]);
      }
      std_msgs::msg::Float64MultiArray task_acceleration_msg;
      task_acceleration_msg.data.assign(
        limited_task_acceleration.data(), limited_task_acceleration.data() + 3);
      metric_pub_->publish(metric_msg);
      acceleration_pub_->publish(acceleration_msg);
      raw_acceleration_pub_->publish(raw_acceleration_msg);
      task_acceleration_pub_->publish(task_acceleration_msg);
      // Publish activation only after the metric/acceleration pair has passed
      // every validity and direction check and is being emitted this cycle.
      std_msgs::msg::Float64 activation_msg;
      activation_msg.data = transition;
      activation_pub_->publish(activation_msg);
      std_msgs::msg::String status;
      status.data = "active";
      mode_pub_->publish(status);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "admittance leaf failed: %s", error.what());
      publish_zero("computation_error");
    }
  }

  std::mutex mutex_;
  JointVector q_{JointVector::Zero()};
  JointVector qd_{JointVector::Zero()};
  Eigen::Vector3d force_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_point_local_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d virtual_mass_{Eigen::Vector3d::Ones()};
  Eigen::Vector3d damping_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d metric_weight_{Eigen::Vector3d::Ones()};
  std::size_t contact_link_{Model::LINK_COUNT};
  bool have_joint_state_{false};
  bool have_wrench_{false};
  bool have_intent_{false};
  bool have_contact_point_{false};
  bool active_{false};
  double compliance_probability_{0.0};
  double activation_phase_{0.0};
  double nominal_dt_{0.008};
  double enter_force_n_{8.0};
  double release_force_n_{3.0};
  double enter_probability_{0.60};
  double release_probability_{0.35};
  double transition_sec_{0.60};
  double joint_state_timeout_sec_{0.10};
  double wrench_timeout_sec_{0.25};
  double intent_timeout_sec_{0.50};
  double max_force_n_{30.0};
  double max_task_accel_m_s2_{2.0};
  JointVector max_joint_accel_per_joint_rad_s2_{JointVector::Ones()};
  double singular_value_damping_ratio_{0.05};
  double minimum_task_alignment_cosine_{0.50};
  double eigenvalue_floor_{1e-8};
  rclcpp::Time joint_state_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wrench_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time intent_received_at_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metric_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr raw_acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr task_acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr activation_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr contact_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr intent_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ur10_intent_admittance_rmp_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<ur10_intent_admittance_rmp_v2::AdmittanceExternalRmp>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("admittance_external_rmp"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
