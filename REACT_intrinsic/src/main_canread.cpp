#include "canread_aidinsensor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("aidinsensor_read");

  std::string can_interface = node->declare_parameter<std::string>("can_interface", "can1");
  double sampling_frequency_hz =
    node->declare_parameter<double>("sampling_frequency_hz", 500.0);

  // Preserve the ROS 1 positional form: main_aidinread <frequency_hz> [can_interface].
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  try {
    if (args.size() > 1) {
      sampling_frequency_hz = std::stod(args[1]);
    }
    if (args.size() > 2) {
      can_interface = args[2];
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Invalid command-line argument: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  if (sampling_frequency_hz <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "sampling_frequency_hz must be greater than zero");
    rclcpp::shutdown();
    return 1;
  }

  CanReadAidinSensor sensor(node, can_interface, 1.0 / sampling_frequency_hz);
  RCLCPP_INFO(
    node->get_logger(), "Aidin sensor reader started: interface=%s, frequency=%.1f Hz",
    can_interface.c_str(), sampling_frequency_hz);
  sensor.run();

  rclcpp::shutdown();
  return 0;
}
