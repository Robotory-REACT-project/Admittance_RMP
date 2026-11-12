#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ur10_intent_admittance_rmp_v2
{

class ContactForceAdapter final : public rclcpp::Node
{
public:
  ContactForceAdapter()
  : Node("contact_force_adapter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.20);
    // 2026-11-07: sensor 1's reliable cp / contact_state topics arrive only
    // ~10 / ~27 Hz with gaps up to ~1.1 s while its corrected wrench arrives
    // ~96 Hz, so a 0.2 s freshness rule on every input dropped the force for
    // most of a push.  The wrench must still be fresh (input_timeout_sec); the
    // contact point and the contact state may be held for up to these times.
    // A held (stale) contact=true additionally needs the fresh corrected force
    // to stay above held_state_min_force_n so a release is not prolonged.
    // Defaults reproduce the previous behaviour exactly.
    point_hold_sec_ = declare_parameter<double>("point_hold_sec", input_timeout_sec_);
    state_hold_sec_ = declare_parameter<double>("state_hold_sec", input_timeout_sec_);
    held_state_min_force_n_ = declare_parameter<double>("held_state_min_force_n", 0.0);
    manual_probability_ = declare_parameter<double>("manual_compliance_probability", 1.0);
    publish_manual_intent_ = declare_parameter<bool>("publish_manual_intent", true);
    force_sign_ = declare_parameter<double>("force_sign", 1.0);
    target_point_frame_ = declare_parameter<std::string>(
      "target_point_frame", "forearm_link");
    target_wrench_frame_ = declare_parameter<std::string>(
      "target_wrench_frame", "base_link");
    const auto output_point_topic = declare_parameter<std::string>(
      "output_contact_point_topic", "/admittance_rmp/contact_point");
    const auto output_wrench_topic = declare_parameter<std::string>(
      "output_wrench_topic", "/admittance_rmp/wrench");
    const auto output_intent_topic = declare_parameter<std::string>(
      "output_intent_topic", "/worker_intent/compliance_probability");

    if (
      publish_rate_hz <= 0.0 || input_timeout_sec_ <= 0.0 ||
      !(point_hold_sec_ >= input_timeout_sec_) || !(state_hold_sec_ >= input_timeout_sec_) ||
      !(held_state_min_force_n_ >= 0.0) ||
      !std::isfinite(force_sign_) || force_sign_ == 0.0 ||
      manual_probability_ < 0.0 || manual_probability_ > 1.0)
    {
      throw std::invalid_argument("invalid contact-force adapter parameters");
    }

    sensors_[0].expected_frame = "ur10_jsshape_supporter1";
    sensors_[1].expected_frame = "ur10_jsshape_supporter2";
    configure_sensor(
      0, "/sensor1/contactsensing/cp",
      "/sensor1/contactsensing/wrench_corrected",
      "/sensor1/contactsensing/contact_state");
    configure_sensor(
      1, "/sensor2/contactsensing/cp",
      "/sensor2/contactsensing/wrench_corrected",
      "/sensor2/contactsensing/contact_state");

    point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      output_point_topic, rclcpp::QoS(10));
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      output_wrench_topic, rclcpp::SensorDataQoS());
    intent_pub_ = create_publisher<std_msgs::msg::Float64>(
      output_intent_topic, rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/admittance_rmp/contact_adapter_status", rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() {publish_selected_contact();});

    RCLCPP_WARN(
      get_logger(),
      "Contact adapter accepts hardware contact from sensor 1 and sensor 2 only. "
      "Sensors 3 and 4 are intentionally excluded.");
  }

private:
  struct SensorState
  {
    std::string expected_frame;
    geometry_msgs::msg::PointStamped point;
    geometry_msgs::msg::WrenchStamped wrench;
    rclcpp::Time point_received{0, 0, RCL_ROS_TIME};
    rclcpp::Time wrench_received{0, 0, RCL_ROS_TIME};
    rclcpp::Time state_received{0, 0, RCL_ROS_TIME};
    bool have_point{false};
    bool have_wrench{false};
    bool have_state{false};
    bool contact{false};
  };

  void configure_sensor(
    std::size_t index, const std::string & point_topic,
    const std::string & wrench_topic, const std::string & state_topic)
  {
    point_subs_[index] = create_subscription<geometry_msgs::msg::PointStamped>(
      point_topic, rclcpp::QoS(10),
      [this, index](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        const bool finite = std::isfinite(msg->point.x) && std::isfinite(msg->point.y) &&
          std::isfinite(msg->point.z);
        if (msg->header.frame_id != sensors_[index].expected_frame || !finite) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "sensor%zu contact point rejected: expected frame %s, received %s",
            index + 1U, sensors_[index].expected_frame.c_str(),
            msg->header.frame_id.c_str());
          return;
        }
        std::scoped_lock lock(mutex_);
        sensors_[index].point = *msg;
        sensors_[index].point_received = now();
        sensors_[index].have_point = true;
      });
    wrench_subs_[index] = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic, rclcpp::SensorDataQoS(),
      [this, index](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        const auto & f = msg->wrench.force;
        const bool finite =
          std::isfinite(f.x) && std::isfinite(f.y) && std::isfinite(f.z);
        if (msg->header.frame_id != sensors_[index].expected_frame || !finite) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "sensor%zu corrected wrench rejected: expected frame %s, received %s",
            index + 1U, sensors_[index].expected_frame.c_str(),
            msg->header.frame_id.c_str());
          return;
        }
        std::scoped_lock lock(mutex_);
        sensors_[index].wrench = *msg;
        sensors_[index].wrench_received = now();
        sensors_[index].have_wrench = true;
      });
    state_subs_[index] = create_subscription<std_msgs::msg::Bool>(
      state_topic, rclcpp::QoS(10),
      [this, index](std_msgs::msg::Bool::SharedPtr msg) {
        std::scoped_lock lock(mutex_);
        sensors_[index].contact = msg->data;
        sensors_[index].state_received = now();
        sensors_[index].have_state = true;
      });
  }

  static double force_norm(const geometry_msgs::msg::WrenchStamped & msg)
  {
    const auto & f = msg.wrench.force;
    return std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
  }

  bool fresh(const rclcpp::Time & received, const rclcpp::Time & current) const
  {
    return within(received, current, input_timeout_sec_);
  }

  static bool within(const rclcpp::Time & received, const rclcpp::Time & current, double limit)
  {
    return (current - received).seconds() <= limit;
  }

  // Contact validity of one sensor.  `held` reports that a stale (but still
  // within its hold time) contact point or contact state was used.
  bool sensor_valid(const SensorState & sensor, const rclcpp::Time & current, bool & held) const
  {
    held = false;
    if (!(sensor.have_point && sensor.have_wrench && sensor.have_state && sensor.contact)) {
      return false;
    }
    if (!fresh(sensor.wrench_received, current) ||
      !within(sensor.point_received, current, point_hold_sec_) ||
      !within(sensor.state_received, current, state_hold_sec_))
    {
      return false;
    }
    if (!fresh(sensor.point_received, current)) {
      held = true;
    }
    if (!fresh(sensor.state_received, current)) {
      held = true;
      if (force_norm(sensor.wrench) < held_state_min_force_n_) {
        return false;
      }
    }
    return true;
  }

  void publish_zero_wrench(const std::string & status)
  {
    geometry_msgs::msg::WrenchStamped output;
    output.header.stamp = now();
    output.header.frame_id = target_wrench_frame_;
    wrench_pub_->publish(output);
    publish_status(status);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String status;
    status.data = text;
    status_pub_->publish(status);
  }

  void publish_selected_contact()
  {
    if (publish_manual_intent_) {
      std_msgs::msg::Float64 intent;
      intent.data = manual_probability_;
      intent_pub_->publish(intent);
    }

    const auto current = now();
    std::array<SensorState, 2> snapshot;
    {
      std::scoped_lock lock(mutex_);
      snapshot = sensors_;
    }

    int selected = -1;
    double selected_norm = -1.0;
    bool selected_held = false;
    for (std::size_t index = 0; index < snapshot.size(); ++index) {
      const auto & sensor = snapshot[index];
      bool held = false;
      if (!sensor_valid(sensor, current, held)) {
        continue;
      }
      const double norm = force_norm(sensor.wrench);
      if (norm > selected_norm) {
        selected = static_cast<int>(index);
        selected_norm = norm;
        selected_held = held;
      }
    }

    if (selected < 0) {
      publish_zero_wrench("inactive:no_fresh_sensor1_or_sensor2_contact");
      return;
    }

    const auto & sensor = snapshot[static_cast<std::size_t>(selected)];
    try {
      const auto point_transform = tf_buffer_.lookupTransform(
        target_point_frame_, sensor.point.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PointStamped point_output;
      tf2::doTransform(sensor.point, point_output, point_transform);
      point_output.header.stamp = current;
      point_output.header.frame_id = target_point_frame_;

      const auto force_transform = tf_buffer_.lookupTransform(
        target_wrench_frame_, sensor.wrench.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::Vector3Stamped force_input;
      force_input.header = sensor.wrench.header;
      force_input.vector = sensor.wrench.wrench.force;
      geometry_msgs::msg::Vector3Stamped force_output;
      tf2::doTransform(force_input, force_output, force_transform);

      geometry_msgs::msg::WrenchStamped wrench_output;
      wrench_output.header.stamp = current;
      wrench_output.header.frame_id = target_wrench_frame_;
      wrench_output.wrench.force.x = force_sign_ * force_output.vector.x;
      wrench_output.wrench.force.y = force_sign_ * force_output.vector.y;
      wrench_output.wrench.force.z = force_sign_ * force_output.vector.z;

      point_pub_->publish(point_output);
      wrench_pub_->publish(wrench_output);
      publish_status(
        "active:sensor" + std::to_string(selected + 1) + (selected_held ? ":held" : ""));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "contact adapter TF unavailable: %s", error.what());
      publish_zero_wrench("inactive:tf_unavailable");
    }
  }

  std::mutex mutex_;
  std::array<SensorState, 2> sensors_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr, 2> point_subs_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr, 2> wrench_subs_;
  std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, 2> state_subs_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr intent_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_point_frame_;
  std::string target_wrench_frame_;
  double input_timeout_sec_{0.20};
  double point_hold_sec_{0.20};
  double state_hold_sec_{0.20};
  double held_state_min_force_n_{0.0};
  double manual_probability_{1.0};
  double force_sign_{1.0};
  bool publish_manual_intent_{true};
};

}  // namespace ur10_intent_admittance_rmp_v2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<ur10_intent_admittance_rmp_v2::ContactForceAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("contact_force_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
