cmake_minimum_required(VERSION 3.8)
project(REACT_intrinsic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ifs_lib
  src/ifs_algorithm.cpp
  src/stl_loader.cpp
  src/canread_aidinsensor.cpp
)
target_include_directories(ifs_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ifs_lib
  ament_index_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(ifs_lib Eigen3::Eigen)

add_executable(main_aidinread src/main_canread.cpp)
target_link_libraries(main_aidinread ifs_lib)
ament_target_dependencies(main_aidinread rclcpp)

add_executable(main_ifs src/main_ifs.cpp)
target_link_libraries(main_ifs ifs_lib)
ament_target_dependencies(main_ifs ament_index_cpp rclcpp)

add_executable(proximity_visualizer src/proximity_visualizer.cpp)
ament_target_dependencies(proximity_visualizer rclcpp sensor_msgs visualization_msgs)

add_executable(sensor_data_logger src/sensor_data_logger.cpp)
ament_target_dependencies(sensor_data_logger rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  ifs_lib
  main_aidinread
  main_ifs
  proximity_visualizer
  sensor_data_logger
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/plot_sensor_logs.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_sensor_data
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

ament_package()
