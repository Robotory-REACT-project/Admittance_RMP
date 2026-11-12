#include "ifs_algorithm.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <filesystem>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nonlinear_surface_contactsensing");

  const std::filesystem::path package_share =
    ament_index_cpp::get_package_share_directory("REACT_intrinsic");
  const std::string default_mesh =
    (package_share / "meshes" / "NRS_spindle_surf.STL").string();

  node->declare_parameter<std::string>("filename", "contact_path");
  const std::string mesh_directory =
    node->declare_parameter<std::string>("mesh_directory", default_mesh);
  const std::string contact_sensor_topic =
    node->declare_parameter<std::string>("contact_sensor_topic", "/ur10skku/ftdata");
  node->declare_parameter<double>("contact_force_threshold", 1.0);
  node->declare_parameter<bool>("use_wrench_tf", false);
  node->declare_parameter<std::string>("contact_frame", "base_link");
  node->declare_parameter<std::string>("wrench_source_frame", "");
  const double wrench_tf_timeout_sec =
    node->declare_parameter<double>("wrench_tf_timeout_sec", 0.05);
  const std::vector<double> transformation = node->declare_parameter<std::vector<double>>(
    "mesh_transformation",
    {1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0, 0.0});

  if (transformation.size() != 12) {
    RCLCPP_ERROR(
      node->get_logger(), "mesh_transformation must contain exactly 12 values; received %zu",
      transformation.size());
    rclcpp::shutdown();
    return 1;
  }
  if (wrench_tf_timeout_sec < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "wrench_tf_timeout_sec must be non-negative");
    rclcpp::shutdown();
    return 1;
  }

  Transformation transform;
  transform <<
    transformation[0], transformation[1], transformation[2], transformation[3],
    transformation[4], transformation[5], transformation[6], transformation[7],
    transformation[8], transformation[9], transformation[10], transformation[11],
    0.0, 0.0, 0.0, 1.0;

  IfsAlgorithm contact_sensing(node, contact_sensor_topic);
  if (!contact_sensing.set_ContactShape(mesh_directory, transform)) {
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(), "REACT intrinsic sensing started: sensor_topic=%s, mesh=%s",
    contact_sensor_topic.c_str(), mesh_directory.c_str());
  contact_sensing.run();

  rclcpp::shutdown();
  return 0;
}
