#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "rb10_rmpflow_rviz/ur10_model.hpp"
#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"

namespace ur10_intent_admittance_rmp_v2
{

class GatedTargetExternalRmp final : public rclcpp::Node
{
public:
  using Model = rb10_rmpflow_rviz::UR10Model;
  using JointVector = Model::JointVector;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;

  GatedTargetExternalRmp()
  : Node("gated_target_external_rmp")
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 125.0);
    p_gain_ = declare_parameter<double>("p_gain", 40.0);
    d_gain_ = declare_parameter<double>("d_gain", 120.0);
    accel_norm_eps_ = declare_parameter<double>("accel_norm_eps", 0.08);
    metric_alpha_length_scale_ =
      declare_parameter<double>("metric_alpha_length_scale", 0.01);
    min_metric_alpha_ = declare_parameter<double>("min_metric_alpha", 0.1);
    max_metric_scalar_ = declare_parameter<double>("max_metric_scalar", 500.0);
    min_metric_scalar_ = declare_parameter<double>("min_metric_scalar", 100.0);
    contact_metric_floor_ = declare_parameter<double>("contact_metric_floor", 0.001);
    joint_state_timeout_sec_ = declare_parameter<double>("joint_state_timeout_sec", 0.10);
    goal_timeout_sec_ = declare_parameter<double>("goal_timeout_sec", 0.50);
    activation_timeout_sec_ = declare_parameter<double>("activation_timeout_sec", 0.20);
    eigenvalue_floor_ = declare_parameter<double>("eigenvalue_floor", 1e-8);
    const auto joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "/UR10_right/joint_states");
    const auto goal_pose_topic = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    const auto goal_position_topic = declare_parameter<std::string>(
      "goal_position_topic", "/goal_position");
    const auto activation_topic = declare_parameter<std::string>(
      "activation_topic", "/admittance_rmp/activation");
    const auto output_prefix = declare_parameter<std::string>(
      "output_prefix", "/escape_on/external_rmp/target_gated");

    if (
      publish_rate_hz <= 0.0 || p_gain_ < 0.0 || d_gain_ < 0.0 ||
      accel_norm_eps_ <= 0.0 || metric_alpha_length_scale_ <= 0.0 ||
      min_metric_alpha_ < 0.0 || min_metric_alpha_ > 1.0 ||
      max_metric_scalar_ <= 0.0 || min_metric_scalar_ < 0.0 ||
      contact_metric_floor_ < 0.0 || contact_metric_floor_ > 1.0 ||
      joint_state_timeout_sec_ <= 0.0 || goal_timeout_sec_ <= 0.0 ||
      activation_timeout_sec_ <= 0.0 || eigenvalue_floor_ <= 0.0)
    {
      throw std::invalid_argument("invalid gated target external-RMP parameters");
    }

    metric_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      output_prefix + "/metric_sqrt", rclcpp::QoS(10));
    acceleration_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      output_prefix + "/acceleration", rclcpp::QoS(10));
    scale_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/admittance_rmp/target_metric_scale", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/admittance_rmp/target_gate_status", rclcpp::QoS(10));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {on_joint_state(*msg);});
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg->header.frame_id != "base_link" && !msg->header.frame_id.empty()) {
          return;
        }
        set_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      });
    goal_position_sub_ = create_subscription<geometry_msgs::msg::Point>(
      goal_position_topic, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Point::SharedPtr msg) {set_goal(msg->x, msg->y, msg->z);});
    activation_sub_ = create_subscription<std_msgs::msg::Float64>(
      activation_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        std::scoped_lock lock(mutex_);
        activation_ = std::clamp(msg->data, 0.0, 1.0);
        activation_received_at_ = now();
        have_activation_ = true;
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() {publish_target_rmp();});
    RCLCPP_WARN(
      get_logger(),
      "EXPERIMENTAL contact-gated target RMP ready (full-contact metric floor=%.4f)",
      contact_metric_floor_);
  }

private:
  void set_goal(double x, double y, double z)
  {
    const Eigen::Vector3d goal(x, y, z);
    if (!goal.allFinite()) {
      return;
    }
    std::scoped_lock lock(mutex_);
    goal_ = goal;
    goal_received_at_ = now();
    have_goal_ = true;
  }

  void on_joint_state(const sensor_msgs::msg::JointState & msg)
  {
    JointVector q = JointVector::Zero();
    JointVector qd = JointVector::Zero();
    for (std::size_t joint = 0; joint < Model::joint_names.size(); ++joint) {
      const auto found = std::find(msg.name.begin(), msg.name.end(), Model::joint_names[joint]);
      if (found == msg.name.end()) {
        return;
      }
      const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), found));
      if (index >= msg.position.size()) {
        return;
      }
      q[static_cast<int>(joint)] = msg.position[index];
      if (index < msg.velocity.size() && std::isfinite(msg.velocity[index])) {
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

  void publish_zero(const std::string & reason)
  {
    std_msgs::msg::Float64MultiArray metric;
    metric.data.assign(36U, 0.0);
    std_msgs::msg::Float64MultiArray acceleration;
    acceleration.data.assign(6U, 0.0);
    metric_pub_->publish(metric);
    acceleration_pub_->publish(acceleration);
    std_msgs::msg::String status;
    status.data = reason;
    status_pub_->publish(status);
  }

  void publish_target_rmp()
  {
    JointVector q;
    JointVector qd;
    Eigen::Vector3d goal;
    double activation = 0.0;
    const auto current = now();
    {
      std::scoped_lock lock(mutex_);
      const bool fresh_joint = have_joint_state_ &&
        (current - joint_state_received_at_).seconds() <= joint_state_timeout_sec_;
      const bool fresh_goal = have_goal_ &&
        (current - goal_received_at_).seconds() <= goal_timeout_sec_;
      if (!fresh_joint || !fresh_goal) {
        publish_zero(!fresh_joint ? "stale_joint_state" : "stale_goal");
        return;
      }
      q = q_;
      qd = qd_;
      goal = goal_;
      if (have_activation_ &&
        (current - activation_received_at_).seconds() <= activation_timeout_sec_)
      {
        activation = activation_;
      }
    }

    try {
      const auto geometry = contact_geometry(
        q, qd, Model::TOOL0, Eigen::Vector3d::Zero());
      const Eigen::Vector3d delta = goal - geometry.position;
      const double distance = delta.norm();
      const double soft_distance = std::max(distance, accel_norm_eps_ / 10.0);
      const Eigen::Vector3d direction = delta / soft_distance;
      const Eigen::Vector3d task_acceleration =
        p_gain_ * delta / (distance + accel_norm_eps_) - d_gain_ * geometry.velocity;
      const double scaled_distance = distance / metric_alpha_length_scale_;
      const double alpha =
        (1.0 - min_metric_alpha_) * std::exp(-0.5 * scaled_distance * scaled_distance) +
        min_metric_alpha_;
      Eigen::Matrix3d task_metric =
        alpha * max_metric_scalar_ * Eigen::Matrix3d::Identity() +
        (1.0 - alpha) * min_metric_scalar_ * direction * direction.transpose();
      const double target_scale =
        contact_metric_floor_ + (1.0 - activation) * (1.0 - contact_metric_floor_);
      task_metric *= target_scale;

      NaturalPair pair;
      pair.metric = geometry.jacobian.transpose() * task_metric * geometry.jacobian;
      pair.force = geometry.jacobian.transpose() * task_metric *
        (task_acceleration - geometry.curvature);
      const JointVector acceleration = resolve_pair_damped(pair, 0.0, eigenvalue_floor_);

      Eigen::SelfAdjointEigenSolver<Matrix6> eigensolver(pair.metric);
      if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("gated target metric eigendecomposition failed");
      }
      const auto values = eigensolver.eigenvalues().cwiseMax(0.0);
      const Matrix6 basis = eigensolver.eigenvectors();
      const Matrix6 metric_sqrt =
        basis * values.cwiseSqrt().asDiagonal() * basis.transpose();
      if (!metric_sqrt.allFinite() || !acceleration.allFinite()) {
        throw std::runtime_error("non-finite gated target output");
      }

      std_msgs::msg::Float64MultiArray metric_msg;
      metric_msg.data.reserve(36U);
      for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
          metric_msg.data.push_back(metric_sqrt(row, col));
        }
      }
      std_msgs::msg::Float64MultiArray acceleration_msg;
      acceleration_msg.data.assign(acceleration.data(), acceleration.data() + 6);
      metric_pub_->publish(metric_msg);
      acceleration_pub_->publish(acceleration_msg);
      std_msgs::msg::Float64 scale_msg;
      scale_msg.data = target_scale;
      scale_pub_->publish(scale_msg);
      std_msgs::msg::String status;
      status.data = "active";
      status_pub_->publish(status);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "gated target RMP failed: %s", error.what());
      publish_zero("computation_error");
    }
  }

  std::mutex mutex_;
  JointVector q_{JointVector::Zero()};
  JointVector qd_{JointVector::Zero()};
  Eigen::Vector3d goal_{Eigen::Vector3d::Zero()};
  bool have_joint_state_{false};
  bool have_goal_{false};
  bool have_activation_{false};
  double activation_{0.0};
  double p_gain_{40.0};
  double d_gain_{120.0};
  double accel_norm_eps_{0.08};
  double metric_alpha_length_scale_{0.01};
  double min_metric_alpha_{0.1};
  double max_metric_scalar_{500.0};
  double min_metric_scalar_{100.0};
  double contact_metric_floor_{0.001};
  double joint_state_timeout_sec_{0.10};
  double goal_timeout_sec_{0.50};
  double activation_timeout_sec_{0.20};
  double eigenvalue_floor_{1e-8};
  rclcpp::Time joint_state_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time activation_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr metric_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr scale_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr activation_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ur10_intent_admittance_rmp_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ur10_intent_admittance_rmp_v2::GatedTargetExternalRmp>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("gated_target_external_rmp"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
