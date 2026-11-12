#include <algorithm>
#include <chrono>
#include <cmath>
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
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rb10_rmpflow_rviz/ur10_model.hpp"
#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"

namespace ur10_intent_admittance_rmp_v2
{

class AdmittancePreview final : public rclcpp::Node
{
public:
  using Model = rb10_rmpflow_rviz::UR10Model;
  using JointVector = Model::JointVector;

  AdmittancePreview()
  : Node("admittance_preview")
  {
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 125.0);
    virtual_mass_ = vector3_parameter("virtual_mass", {3.0, 3.0, 3.0});
    contact_damping_ = vector3_parameter("contact_damping", {45.0, 45.0, 45.0});
    contact_metric_ = vector3_parameter("contact_metric", {200.0, 200.0, 200.0});
    enter_force_n_ = declare_parameter<double>("enter_force_n", 2.0);
    release_force_n_ = declare_parameter<double>("release_force_n", 1.0);
    transition_sec_ = declare_parameter<double>("transition_sec", 0.20);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.25);
    stop_damping_ = declare_parameter<double>("release_stop_damping", 8.0);
    max_acceleration_ = declare_parameter<double>("max_joint_accel_rad_s2", 1.0);
    max_velocity_ = declare_parameter<double>("max_joint_velocity_rad_s", 0.30);
    root_offset_ = declare_parameter<double>("root_regularization", 1e-4);

    if (
      rate_hz <= 0.0 || enter_force_n_ <= release_force_n_ ||
      release_force_n_ < 0.0 || transition_sec_ <= 0.0 || input_timeout_sec_ <= 0.0 ||
      stop_damping_ <= 0.0 || max_acceleration_ <= 0.0 || max_velocity_ <= 0.0 ||
      root_offset_ <= 0.0)
    {
      throw std::invalid_argument("invalid admittance preview parameters");
    }

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/UR10_right/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {on_measured_joint_state(*msg);});
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/admittance_rmp/contact_point", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {on_point(*msg);});
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/admittance_rmp/wrench", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {on_wrench(*msg);});
    intent_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/worker_intent/compliance_probability", rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {on_intent(*msg);});

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/admittance_preview/joint_states", rclcpp::SensorDataQoS());
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/admittance_preview/mode", rclcpp::QoS(10));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/admittance_preview/reset_to_measured",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::scoped_lock lock(mutex_);
        if (!have_measured_) {
          response->success = false;
          response->message = "no measured UR10 joint state";
          return;
        }
        q_sim_ = q_measured_;
        qd_sim_.setZero();
        initialized_ = true;
        active_ = false;
        phase_ = 0.0;
        response->success = true;
        response->message = "preview reset to measured pose";
      });

    dt_ = 1.0 / rate_hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), [this]() {step();});
    RCLCPP_WARN(
      get_logger(),
      "ADMITTANCE-ONLY PREVIEW: publishes simulated joint states only; "
      "it never publishes /UR10_right/targetJ.");
  }

private:
  Eigen::Vector3d vector3_parameter(
    const std::string & name, const std::vector<double> & defaults)
  {
    const auto value = declare_parameter<std::vector<double>>(name, defaults);
    if (value.size() != 3U) {
      throw std::invalid_argument(name + " must contain three values");
    }
    return {value[0], value[1], value[2]};
  }

  bool parse_joint_state(const sensor_msgs::msg::JointState & msg, JointVector & q) const
  {
    for (std::size_t joint = 0; joint < Model::joint_names.size(); ++joint) {
      const auto found = std::find(msg.name.begin(), msg.name.end(), Model::joint_names[joint]);
      if (found == msg.name.end()) {
        return false;
      }
      const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), found));
      if (index >= msg.position.size() || !std::isfinite(msg.position[index])) {
        return false;
      }
      q[static_cast<int>(joint)] = msg.position[index];
    }
    return true;
  }

  void on_measured_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    JointVector q;
    if (!parse_joint_state(msg, q)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    q_measured_ = q;
    have_measured_ = true;
    if (!initialized_) {
      q_sim_ = q;
      qd_sim_.setZero();
      initialized_ = true;
    }
  }

  void on_point(const geometry_msgs::msg::PointStamped & msg)
  {
    if (msg.header.frame_id != "forearm_link") {
      return;
    }
    const Eigen::Vector3d point(msg.point.x, msg.point.y, msg.point.z);
    if (!point.allFinite()) {
      return;
    }
    std::scoped_lock lock(mutex_);
    point_ = point;
    point_time_ = now();
    have_point_ = true;
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped & msg)
  {
    if (msg.header.frame_id != "base_link") {
      return;
    }
    const Eigen::Vector3d force(
      msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    if (!force.allFinite()) {
      return;
    }
    std::scoped_lock lock(mutex_);
    force_ = force;
    wrench_time_ = now();
    have_wrench_ = true;
  }

  void on_intent(const std_msgs::msg::Float64 & msg)
  {
    if (!std::isfinite(msg.data)) {
      return;
    }
    std::scoped_lock lock(mutex_);
    probability_ = std::clamp(msg.data, 0.0, 1.0);
    intent_time_ = now();
    have_intent_ = true;
  }

  static double smoothstep(double value)
  {
    const double x = std::clamp(value, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  void publish(const std::string & mode)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.assign(Model::joint_names.begin(), Model::joint_names.end());
    msg.position.assign(q_sim_.data(), q_sim_.data() + q_sim_.size());
    msg.velocity.assign(qd_sim_.data(), qd_sim_.data() + qd_sim_.size());
    joint_pub_->publish(msg);
    std_msgs::msg::String status;
    status.data = mode;
    mode_pub_->publish(status);
  }

  void step()
  {
    std::scoped_lock lock(mutex_);
    if (!initialized_) {
      return;
    }
    const auto current = now();
    const bool fresh_point = have_point_ &&
      (current - point_time_).seconds() <= input_timeout_sec_;
    const bool fresh_wrench = have_wrench_ &&
      (current - wrench_time_).seconds() <= input_timeout_sec_;
    const bool fresh_intent = have_intent_ &&
      (current - intent_time_).seconds() <= input_timeout_sec_;
    const double force_norm = fresh_wrench ? force_.norm() : 0.0;
    const double p = fresh_intent ? probability_ : 0.0;

    if (!active_ && fresh_point && fresh_wrench && fresh_intent &&
      force_norm >= enter_force_n_ && p > 0.5)
    {
      active_ = true;
    } else if (active_ &&
      (!fresh_point || !fresh_wrench || !fresh_intent ||
      force_norm <= release_force_n_ || p <= 0.5))
    {
      active_ = false;
    }

    const double target = active_ ? 1.0 : 0.0;
    phase_ += std::clamp(target - phase_, -dt_ / transition_sec_, dt_ / transition_sec_);
    phase_ = std::clamp(phase_, 0.0, 1.0);

    JointVector qdd = -stop_damping_ * qd_sim_;
    std::string mode = active_ ? "active" : "hold";
    if (phase_ > 1e-6 && fresh_point && fresh_wrench && fresh_intent) {
      try {
        const auto geometry = contact_geometry(
          q_sim_, qd_sim_, Model::FOREARM_LINK, point_);
        const auto pair = contact_natural_pair(
          geometry, force_, virtual_mass_, contact_damping_, contact_metric_,
          smoothstep(phase_) * p);
        qdd = resolve_pair(pair, root_offset_);
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "preview step failed: %s", error.what());
        qdd = -stop_damping_ * qd_sim_;
        mode = "computation_error";
      }
    }

    for (int joint = 0; joint < 6; ++joint) {
      qdd[joint] = std::clamp(qdd[joint], -max_acceleration_, max_acceleration_);
      qd_sim_[joint] = std::clamp(
        qd_sim_[joint] + qdd[joint] * dt_, -max_velocity_, max_velocity_);
    }
    JointVector next = Model::clamp_positions(q_sim_ + qd_sim_ * dt_);
    for (int joint = 0; joint < 6; ++joint) {
      if (next[joint] != q_sim_[joint] + qd_sim_[joint] * dt_) {
        qd_sim_[joint] = 0.0;
      }
    }
    q_sim_ = next;
    if (!active_ && qd_sim_.cwiseAbs().maxCoeff() < 1e-4) {
      qd_sim_.setZero();
    }
    publish(mode);
  }

  std::mutex mutex_;
  JointVector q_measured_{JointVector::Zero()};
  JointVector q_sim_{JointVector::Zero()};
  JointVector qd_sim_{JointVector::Zero()};
  Eigen::Vector3d point_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d force_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d virtual_mass_{Eigen::Vector3d::Ones()};
  Eigen::Vector3d contact_damping_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_metric_{Eigen::Vector3d::Ones()};
  bool have_measured_{false};
  bool initialized_{false};
  bool have_point_{false};
  bool have_wrench_{false};
  bool have_intent_{false};
  bool active_{false};
  double probability_{0.0};
  double phase_{0.0};
  double dt_{0.008};
  double enter_force_n_{2.0};
  double release_force_n_{1.0};
  double transition_sec_{0.20};
  double input_timeout_sec_{0.25};
  double stop_damping_{8.0};
  double max_acceleration_{1.0};
  double max_velocity_{0.30};
  double root_offset_{1e-4};
  rclcpp::Time point_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wrench_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time intent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr intent_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ur10_intent_admittance_rmp_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ur10_intent_admittance_rmp_v2::AdmittancePreview>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("admittance_preview"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
