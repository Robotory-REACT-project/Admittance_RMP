#pragma once

#include "stl_loader.hpp"

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

struct WrenchVector
{
  float at[3];
};

struct WrenchAxis
{
  WrenchVector r0;
  WrenchVector f;
};

inline void insert_wrench(WrenchVector & wrench, float x, float y, float z)
{
  wrench.at[0] = x;
  wrench.at[1] = y;
  wrench.at[2] = z;
}

// 윗면·옆면·모서리 확률 (합 1). 모서리 넘어가기 대책(kEdgeTransitionFix)에서 쓴다.
struct FaceWeights
{
  double top;
  double side;
  double edge;
};

class IfsAlgorithm
{
public:
  IfsAlgorithm(rclcpp::Node::SharedPtr node, const std::string & contact_sensor_topic);

  void run();
  bool set_ContactShape(const std::string & filename, Transformation transform);
  std::vector<float> get_ContactPoint(const std::vector<float> & ft);
  void draw_arrow(
    const geometry_msgs::msg::PointStamped & contact_point,
    const geometry_msgs::msg::Wrench & force);
  void delete_arrow(
    const geometry_msgs::msg::PointStamped & contact_point,
    const geometry_msgs::msg::Wrench & force);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr cp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ftsensor_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_magnitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr corrected_wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_state_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  StlLoader surface_shape_;
  sensor_msgs::msg::PointCloud pc_data_;

  bool ft_measure_flag_{false};
  // 저역통과 계수를 샘플 간격에서 만들기 위한 직전 샘플 시각 [s]
  double last_sample_sec_{-1.0};
  double force_magnitude_{0.0};
  double contact_force_threshold_{1.0};
  // 힘의 감소율로 손을 떼는 구간을 검출해 불안정한 접촉점 갱신을 막는다.
  double prev_force_{0.0};
  double force_rate_{0.0};
  // 힘이 급격히 줄 때 접촉점을 얼려두는 상태
  bool position_frozen_{false};
  bool freeze_expired_{false};
  int freeze_ticks_{0};
  // 한 번의 접촉 동안 최종 접촉점 계산이 표면 교차점을 찾지 못한 횟수를 센다.
  bool contact_session_active_{false};
  std::uint64_t contact_point_attempts_{0};
  std::uint64_t contact_point_zero_results_{0};
  std::uint64_t consecutive_zero_results_{0};
  std::uint64_t max_consecutive_zero_results_{0};
  int count_{0};
  // 모서리 넘어가기 대책에서 이번 틱에 쓴 면 확률 (로그용)
  FaceWeights face_weights_{1.0, 0.0, 0.0};
  bool face_blend_used_{false};
  // 센서2 배율 x 대칭 (ifs_algorithm.cpp 의 kMirrorGainX). -1 이면 뒤집음
  double gain_x_sign_{1.0};
  // 센서2 전용 배율 묶음을 쓰는지 (ifs_algorithm.cpp 의 kSensor2OwnGains, kGainsSensor2)
  bool sensor2_gains_{false};
  // 이 노드에서 접촉점 보정을 쓰는지 (kContactCorrection, kNoCorrectionSensor2)
  bool contact_correction_{true};
  // kEdgeTransitionFix, kSensor2NoEdgeTransition
  bool edge_transition_active_{true};
  // 마찰원뿔 cos (kContactConeMinCos, kSensor2NoFrictionCone). 0 이면 원뿔 없음
  float contact_cone_min_cos_{0.0F};
  // 옆면 배율을 안 거는지 (kSensor2NoSideGains)
  bool side_gains_off_{false};
  // 배율 위치를 원본 렌치의 면 평면 교점으로 한 번에 정하는지 (kSensor2RawPositionGains)
  bool raw_position_gains_{false};
  // 0 보다 크면 접촉점을 가까운 표면점으로, 이 거리[m] 까지 (kSensor2NearestSurface)
  double nearest_max_dist_{0.0};
  // 옆면 테두리 점일 때의 거리 한도 [m] (kSensor2RimMaxDist). 0 이면 넓히지 않음
  double rim_max_dist_{0.0};
  bool first_draw_{true};
  bool use_wrench_tf_{false};
  double wrench_tf_timeout_sec_{0.05};

  geometry_msgs::msg::PointStamped cp_pubdata_;
  std::vector<float> ft_value_;
  std::vector<float> ft_corrected_;
  std::vector<float> cp_;
  std::vector<float> cp_temp_;
  std::string filename_;
  std::string contact_frame_{"base_link"};
  std::string wrench_source_frame_;
  std::ofstream out_file_;

  void apply_FaceGains(
    const std::vector<float> & raw, const std::vector<float> & point,
    std::vector<float> & out) const;
  void apply_EdgeTransitionGains(std::vector<float> & probe);
  std::vector<float> get_NearestSurfacePoint(
    const std::vector<float> & ft, double max_distance, double rim_max_distance = 0.0);
  void ftsensor_cb(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr message);
  void get_WrenchAxis(const std::vector<float> & ft, WrenchAxis & out);
  int get_ClosestSurface(WrenchAxis wrench_axis);
  float get_DistanceLinePoint(WrenchAxis wrench_axis, float x, float y, float z);
  int get_NextTriangle(int index, std::vector<int> & vertex_indices);
  bool is_SurfaceContact(
    int index, WrenchAxis wrench_axis, std::vector<int> & vertex_indices,
    std::vector<float> & contact_point);
};
