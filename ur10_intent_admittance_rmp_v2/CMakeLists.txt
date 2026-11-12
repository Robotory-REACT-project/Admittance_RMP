cmake_minimum_required(VERSION 3.8)
project(ur10_intent_admittance_rmp_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2026-11-07: without a build type colcon compiled this package at -O0, so
# the Eigen-heavy controller fork needed 4-7 ms of an 8 ms (125 Hz) cycle and
# fell behind during contact (then caught up with back-to-back cycles, i.e.
# joint speeds above the 0.25 rad/s guard after release).  The production
# package is built Release; default to the same here.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rb10_rmpflow_rviz REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# rb10_rmpflow_rviz installs the UR10 model as public headers but does not
# currently export an include target. Resolve its installed include directory
# without changing that production package.
get_filename_component(RMPFLOW_PREFIX "${rb10_rmpflow_rviz_DIR}/../../.." ABSOLUTE)
get_filename_component(RMPFLOW_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../RMP_prox" ABSOLUTE)
find_path(
  RMPFLOW_INCLUDE_DIR
  rb10_rmpflow_rviz/ur10_model.hpp
  HINTS "${RMPFLOW_PREFIX}/include"
  REQUIRED
)

# Build the contact-gated controller from an isolated generated copy.  The
# production six-RMP source and executable remain byte-for-byte untouched.
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated_internal_gate")
set(GENERATED_CONTROLLER "${GENERATED_DIR}/rmpflow_internal_gated_controller.cpp")
set(GENERATED_LAUNCH_DIR "${GENERATED_DIR}/launch")
execute_process(
  COMMAND
    "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_internal_gated_fork.py"
    --source-root "${RMPFLOW_SOURCE_ROOT}"
    --controller-output "${GENERATED_CONTROLLER}"
    --launch-output-dir "${GENERATED_LAUNCH_DIR}"
  RESULT_VARIABLE GATE_GENERATION_RESULT
  OUTPUT_VARIABLE GATE_GENERATION_OUTPUT
  ERROR_VARIABLE GATE_GENERATION_ERROR
)
if(NOT GATE_GENERATION_RESULT EQUAL 0)
  message(FATAL_ERROR
    "Failed to generate internal-gated controller fork:\n"
    "${GATE_GENERATION_OUTPUT}\n${GATE_GENERATION_ERROR}")
endif()

set(CASADI_ROOT "/home/ur/ros2_ws/third_party/casadi/install")
set(PINOCCHIO_ROOT "/home/ur/ros2_ws/third_party/pinocchio/install")
find_library(
  PINOCCHIO_LIBRARY pinocchio
  HINTS "${PINOCCHIO_ROOT}/lib"
  REQUIRED
)
find_library(TINYXML2_LIBRARY tinyxml2 REQUIRED)

add_executable(
  rmpflow_internal_gated_controller
  "${GENERATED_CONTROLLER}"
  "${RMPFLOW_SOURCE_ROOT}/src/rb10_socket_client.cpp"
)
target_include_directories(
  rmpflow_internal_gated_controller
  PRIVATE
    include
    "${RMPFLOW_INCLUDE_DIR}"
    "${PINOCCHIO_ROOT}/include"
    "${CASADI_ROOT}/include"
)
set_target_properties(
  rmpflow_internal_gated_controller PROPERTIES
  BUILD_RPATH "${CASADI_ROOT}/lib;${PINOCCHIO_ROOT}/lib"
  INSTALL_RPATH "${CASADI_ROOT}/lib;${PINOCCHIO_ROOT}/lib"
)
target_link_libraries(
  rmpflow_internal_gated_controller
  Eigen3::Eigen
  "${RMPFLOW_PREFIX}/lib/librb10_pinocchio_rmp.a"
  "${RMPFLOW_PREFIX}/lib/librb10_casadi_graph.a"
  "${RMPFLOW_PREFIX}/lib/librb10_rmp_core.a"
  Boost::filesystem
  Boost::serialization
  Threads::Threads
  yaml-cpp
  "${CASADI_ROOT}/lib/libcasadi.so"
  "${PINOCCHIO_LIBRARY}"
  "${TINYXML2_LIBRARY}"
)
ament_target_dependencies(
  rmpflow_internal_gated_controller
  ament_index_cpp
  geometry_msgs
  rcl_interfaces
  rclcpp
  realtime_tools
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

add_executable(
  admittance_external_rmp
  src/admittance_external_rmp_node.cpp
)
target_include_directories(
  admittance_external_rmp
  PRIVATE include "${RMPFLOW_INCLUDE_DIR}"
)
target_link_libraries(admittance_external_rmp Eigen3::Eigen)
ament_target_dependencies(
  admittance_external_rmp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

add_executable(
  contact_force_adapter
  src/contact_force_adapter_node.cpp
)

add_executable(
  gated_target_external_rmp
  src/gated_target_external_rmp_node.cpp
)
target_include_directories(
  gated_target_external_rmp
  PRIVATE include "${RMPFLOW_INCLUDE_DIR}"
)
target_link_libraries(gated_target_external_rmp Eigen3::Eigen)
ament_target_dependencies(
  gated_target_external_rmp
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(
  admittance_preview
  src/admittance_preview_node.cpp
)
target_include_directories(
  admittance_preview
  PRIVATE include "${RMPFLOW_INCLUDE_DIR}"
)
target_link_libraries(admittance_preview Eigen3::Eigen)
ament_target_dependencies(
  admittance_preview
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
ament_target_dependencies(
  contact_force_adapter
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    admittance_external_rmp
    gated_target_external_rmp
    contact_force_adapter
    admittance_preview
    rmpflow_internal_gated_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES
  "${GENERATED_LAUNCH_DIR}/internal_gate_escape_overlay.launch.py"
  "${GENERATED_LAUNCH_DIR}/internal_gate_ur10_proximity_urcb2.launch.py"
  "${GENERATED_LAUNCH_DIR}/internal_gate_ur10_fused2_urcb2.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contact_policy test/test_contact_policy.cpp)
  target_include_directories(
    test_contact_policy
    PRIVATE include "${RMPFLOW_INCLUDE_DIR}"
  )
  target_link_libraries(test_contact_policy Eigen3::Eigen)

  # v2 goal 1-2: base-lever compensation of the admittance flow.
  ament_add_gtest(test_base_lever_boost test/test_base_lever_boost.cpp)
  target_include_directories(
    test_base_lever_boost
    PRIVATE include "${RMPFLOW_INCLUDE_DIR}"
  )
  target_link_libraries(test_base_lever_boost Eigen3::Eigen)

  # v2 "GM" (2026-11-07): force-based authority, push-gradient target
  # projection and the unreachable-lever tail.
  ament_add_gtest(test_contact_arbitration test/test_contact_arbitration.cpp)
  target_include_directories(
    test_contact_arbitration
    PRIVATE include "${RMPFLOW_INCLUDE_DIR}"
  )
  target_link_libraries(test_contact_arbitration Eigen3::Eigen)

  # v2 goal 2: continuous intent (evidence memory, compliance gate,
  # geometric target scale).
  ament_add_gtest(test_contact_intent test/test_contact_intent.cpp)
  target_include_directories(test_contact_intent PRIVATE include)

  # v2: offline equivalence of the intent-gated avoidance against the
  # production solver (linked read-only, same libraries as the fork).
  ament_add_gtest(test_intent_avoidance_gate test/test_intent_avoidance_gate.cpp)
  target_include_directories(
    test_intent_avoidance_gate
    PRIVATE
      include
      "${RMPFLOW_INCLUDE_DIR}"
      "${PINOCCHIO_ROOT}/include"
      "${CASADI_ROOT}/include"
  )
  set_target_properties(
    test_intent_avoidance_gate PROPERTIES
    BUILD_RPATH "${CASADI_ROOT}/lib;${PINOCCHIO_ROOT}/lib"
  )
  target_link_libraries(
    test_intent_avoidance_gate
    Eigen3::Eigen
    "${RMPFLOW_PREFIX}/lib/librb10_pinocchio_rmp.a"
    "${RMPFLOW_PREFIX}/lib/librb10_casadi_graph.a"
    "${RMPFLOW_PREFIX}/lib/librb10_rmp_core.a"
    Boost::filesystem
    Boost::serialization
    Threads::Threads
    yaml-cpp
    "${CASADI_ROOT}/lib/libcasadi.so"
    "${PINOCCHIO_LIBRARY}"
    "${TINYXML2_LIBRARY}"
  )
  ament_target_dependencies(test_intent_avoidance_gate ament_index_cpp)
endif()

ament_package()
