#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ProximityVisualizer : public rclcpp::Node
{
public:
  ProximityVisualizer()
  : Node("proximity_visualizer")
  {
    const auto topic1 = declare_parameter<std::string>(
      "distance_topic1", "/fused_distance1");
    const auto topic2 = declare_parameter<std::string>(
      "distance_topic2", "/fused_distance2");
    const auto frame1 = declare_parameter<std::string>(
      "sensor_frame1", "proximity1");
    const auto frame2 = declare_parameter<std::string>(
      "sensor_frame2", "proximity2");
    marker_topic1_ = declare_parameter<std::string>(
      "marker_topic1", "/proximity_sensor/distance_marker1");
    marker_topic2_ = declare_parameter<std::string>(
      "marker_topic2", "/proximity_sensor/distance_marker2");
    max_visualization_range_ = declare_parameter<double>("max_visualization_range", 0.5);
    marker_diameter_ = declare_parameter<double>("marker_diameter", 0.04);
    ema_alpha_ = declare_parameter<double>("ema_alpha", 0.2);
    const auto marker_publish_rate = declare_parameter<double>("marker_publish_rate", 30.0);

    if (frame1.empty() || frame2.empty()) {
      throw std::invalid_argument("sensor frames must not be empty");
    }
    if (max_visualization_range_ <= 0.0) {
      throw std::invalid_argument("max_visualization_range must be greater than zero");
    }
    if (marker_diameter_ <= 0.0) {
      throw std::invalid_argument("marker_diameter must be greater than zero");
    }
    if (ema_alpha_ <= 0.0 || ema_alpha_ > 1.0) {
      throw std::invalid_argument("ema_alpha must be in the interval (0, 1]");
    }
    if (marker_publish_rate <= 0.0) {
      throw std::invalid_argument("marker_publish_rate must be greater than zero");
    }

    frame1_ = frame1;
    frame2_ = frame2;
    distance1_ = std::numeric_limits<double>::quiet_NaN();
    distance2_ = std::numeric_limits<double>::quiet_NaN();

    marker_pub1_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic1_, 10);
    marker_pub2_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic2_, 10);

    range_sub1_ = create_subscription<sensor_msgs::msg::Range>(
      topic1, rclcpp::QoS(10),
      [this](sensor_msgs::msg::Range::ConstSharedPtr msg) {
        update_filtered_distance(msg->range, distance1_);
      });
    range_sub2_ = create_subscription<sensor_msgs::msg::Range>(
      topic2, rclcpp::QoS(10),
      [this](sensor_msgs::msg::Range::ConstSharedPtr msg) {
        update_filtered_distance(msg->range, distance2_);
      });

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / marker_publish_rate));
    marker_timer_ = create_wall_timer(timer_period, [this]() {publish_markers();});

    RCLCPP_INFO(
      get_logger(),
      "Visualizing %s from %s on %s and %s from %s on %s "
      "(distance unit: m, EMA: %.2f * new + %.2f * previous)",
      topic1.c_str(), frame1.c_str(), marker_topic1_.c_str(),
      topic2.c_str(), frame2.c_str(), marker_topic2_.c_str(),
      ema_alpha_, 1.0 - ema_alpha_);
  }

private:
  void update_filtered_distance(const double sample, double & filtered_distance) const
  {
    if (!std::isfinite(sample) || sample <= 0.0) {
      filtered_distance = sample;
      return;
    }

    if (!std::isfinite(filtered_distance) || filtered_distance <= 0.0) {
      filtered_distance = sample;
      return;
    }

    filtered_distance = ema_alpha_ * sample + (1.0 - ema_alpha_) * filtered_distance;
  }

  visualization_msgs::msg::Marker make_marker(
    const std::string & sensor_frame,
    const std::int32_t marker_id,
    const double distance) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = sensor_frame;
    marker.ns = "proximity_distance";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;

    if (!std::isfinite(distance) || distance <= 0.0 || distance > max_visualization_range_) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      return marker;
    }

    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = distance;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_diameter_;
    marker.scale.y = marker_diameter_;
    marker.scale.z = marker_diameter_;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.frame_locked = true;
    return marker;
  }

  void publish_markers()
  {
    marker_pub1_->publish(make_marker(frame1_, 1, distance1_));
    marker_pub2_->publish(make_marker(frame2_, 2, distance2_));
  }

  std::string marker_topic1_;
  std::string marker_topic2_;
  std::string frame1_;
  std::string frame2_;
  double max_visualization_range_;
  double marker_diameter_;
  double ema_alpha_;
  double distance1_;
  double distance2_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub1_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub2_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub1_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub2_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProximityVisualizer>());
  rclcpp::shutdown();
  return 0;
}
