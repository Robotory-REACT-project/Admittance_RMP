#include "ifs_algorithm.hpp"

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace
{
// 접촉 해제 감도 [N/s]. 작을수록 민감하며, 0.0이면 해제 감지 기능을 끈다.
constexpr double kReleaseRateThreshold = 0;

// ---- 모멘트 채널 보정 ----
// 접촉 위치는 모멘트에서 나온다. 면에 수직으로 누를 때
//     가로 x = -Mz/|F| ,   세로 z = +Mx/|F|
// 이므로 모멘트 채널이 어긋나면 위치가 그대로 일그러진다.
//
// 2026-09-08 서포터 앞면 9지점 실측(각 30~47 N)으로 맞춘 값이다. 보정 전에는
// 가로가 11% 늘어나고 세로가 31% 줄면서 세로가 가로에 끌려갔다(평균 33 mm,
// 최대 72 mm 오차). 아래 계수를 넣으면 평균 9 mm, 최대 22 mm 로 줄어든다.
//
// 계수는 supporter 좌표계 기준이라 TF 변환 뒤에 적용한다. 위치에 의존하지
// 않으므로 순환도 반복 계산도 없다.
// 끄려면 kMomentCorrectionEnabled 를 false 로.
constexpr bool kMomentCorrectionEnabled = true;
//                              Mx         My         Mz        |F|
constexpr double kMxCorr[4] = { 1.50866,  1.16372, -0.24319, -0.00601};
constexpr double kMzCorr[4] = { 0.01875,  0.14197,  0.80077,  0.01849};
}  // namespace

IfsAlgorithm::IfsAlgorithm(
  rclcpp::Node::SharedPtr node,
  const std::string & contact_sensor_topic)
: node_(std::move(node)),
  ft_value_(6, 0.0F),
  cp_(3, 0.0F),
  cp_temp_(6, 0.0F)
{
  node_->get_parameter_or("use_wrench_tf", use_wrench_tf_, false);
  node_->get_parameter_or("contact_frame", contact_frame_, std::string("base_link"));
  node_->get_parameter_or("wrench_source_frame", wrench_source_frame_, std::string(""));
  node_->get_parameter_or("wrench_tf_timeout_sec", wrench_tf_timeout_sec_, 0.05);

  if (use_wrench_tf_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
  }

  cp_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/contactsensing/cp", 1000);
  arrow_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "/contactsensing/force_arrow", 1000);
  ftsensor_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    contact_sensor_topic, 100,
    std::bind(&IfsAlgorithm::ftsensor_cb, this, std::placeholders::_1));
  pointcloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud>("/mesh_pc", 100);
  force_magnitude_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/contactsensing/force_magnitude", 100);

  node_->get_parameter_or("filename", filename_, std::string("contact_path"));
  node_->get_parameter_or("contact_force_threshold", contact_force_threshold_, 1.0);
  cp_pubdata_.header.frame_id = contact_frame_;

  RCLCPP_INFO(
    node_->get_logger(),
    "Wrench coordinates: TF=%s, source=%s, target=%s, timeout=%.3f s",
    use_wrench_tf_ ? "enabled" : "disabled",
    wrench_source_frame_.empty() ? "<message header>" : wrench_source_frame_.c_str(),
    contact_frame_.c_str(), wrench_tf_timeout_sec_);
}

void IfsAlgorithm::ftsensor_cb(const geometry_msgs::msg::WrenchStamped::ConstSharedPtr message)
{
  if (ft_measure_flag_) {
    return;
  }

  geometry_msgs::msg::WrenchStamped wrench_in = *message;
  if (!wrench_source_frame_.empty()) {
    wrench_in.header.frame_id = wrench_source_frame_;
  }

  geometry_msgs::msg::Wrench transformed_wrench = wrench_in.wrench;
  if (use_wrench_tf_) {
    if (wrench_in.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Cannot transform wrench: message frame is empty and wrench_source_frame is not set");
      return;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        contact_frame_, wrench_in.header.frame_id,
        tf2_ros::fromMsg(wrench_in.header.stamp),
        tf2::durationFromSec(wrench_tf_timeout_sec_));
      tf2::doTransform(wrench_in.wrench, transformed_wrench, transform);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Cannot transform wrench from '%s' to '%s': %s",
        wrench_in.header.frame_id.c_str(), contact_frame_.c_str(), exception.what());
      return;
    }
  }

  if (kMomentCorrectionEnabled) {
    const double mx = transformed_wrench.torque.x;
    const double my = transformed_wrench.torque.y;
    const double mz = transformed_wrench.torque.z;
    const double f_norm = std::sqrt(
      transformed_wrench.force.x * transformed_wrench.force.x +
      transformed_wrench.force.y * transformed_wrench.force.y +
      transformed_wrench.force.z * transformed_wrench.force.z);
    transformed_wrench.torque.x =
      kMxCorr[0] * mx + kMxCorr[1] * my + kMxCorr[2] * mz + kMxCorr[3] * f_norm;
    transformed_wrench.torque.z =
      kMzCorr[0] * mx + kMzCorr[1] * my + kMzCorr[2] * mz + kMzCorr[3] * f_norm;
  }

  constexpr double algorithm_sign = 1.0;
  constexpr double lambda = 0.05;
  const auto & wrench = transformed_wrench;

  ft_value_[0] = lambda * algorithm_sign * wrench.force.x + (1.0 - lambda) * ft_value_[0];
  ft_value_[1] = lambda * algorithm_sign * wrench.force.y + (1.0 - lambda) * ft_value_[1];
  ft_value_[2] = lambda * algorithm_sign * wrench.force.z + (1.0 - lambda) * ft_value_[2];
  ft_value_[3] = lambda * algorithm_sign * wrench.torque.x + (1.0 - lambda) * ft_value_[3];
  ft_value_[4] = lambda * algorithm_sign * wrench.torque.y + (1.0 - lambda) * ft_value_[4];
  ft_value_[5] = lambda * algorithm_sign * wrench.torque.z + (1.0 - lambda) * ft_value_[5];
  force_magnitude_ = std::sqrt(
    ft_value_[0] * ft_value_[0] +
    ft_value_[1] * ft_value_[1] +
    ft_value_[2] * ft_value_[2]);
  ft_measure_flag_ = true;
}

int IfsAlgorithm::get_ClosestSurface(WrenchAxis wrench_axis)
{
  double min_value = std::numeric_limits<double>::infinity();
  int closest_index = 0;

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  for (int i = 0; i < surface_shape_.get_Nvertex(); ++i) {
    surface_shape_.get_Normal(i, x, y, z);
    if ((x * wrench_axis.f.at[0] + y * wrench_axis.f.at[1] + z * wrench_axis.f.at[2]) >= 0) {
      continue;
    }

    surface_shape_.get_CenterPoint(i, x, y, z);
    const double distance = get_DistanceLinePoint(wrench_axis, x, y, z);
    if (distance < min_value) {
      min_value = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

float IfsAlgorithm::get_DistanceLinePoint(
  WrenchAxis wrench_axis, float x, float y, float z)
{
  const float force_norm = std::sqrt(
    wrench_axis.f.at[0] * wrench_axis.f.at[0] +
    wrench_axis.f.at[1] * wrench_axis.f.at[1] +
    wrench_axis.f.at[2] * wrench_axis.f.at[2]);
  if (force_norm <= std::numeric_limits<float>::epsilon()) {
    return std::numeric_limits<float>::infinity();
  }

  const std::array<float, 3> point = {
    x - wrench_axis.r0.at[0],
    y - wrench_axis.r0.at[1],
    z - wrench_axis.r0.at[2]};
  const std::array<float, 3> cross = {
    wrench_axis.f.at[1] * point[2] - wrench_axis.f.at[2] * point[1],
    wrench_axis.f.at[2] * point[0] - wrench_axis.f.at[0] * point[2],
    wrench_axis.f.at[0] * point[1] - wrench_axis.f.at[1] * point[0]};

  return std::sqrt(
    cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]) / force_norm;
}

bool IfsAlgorithm::set_ContactShape(
  const std::string & filename, Transformation transform)
{
  surface_shape_.STLImport(filename, transform);
  if (surface_shape_.get_Npoint() <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "STL loader could not read: %s", filename.c_str());
    return false;
  }

  pc_data_.header.frame_id = contact_frame_;
  pc_data_.points.reserve(static_cast<std::size_t>(surface_shape_.get_Npoint()));
  for (int i = 0; i < surface_shape_.get_Npoint(); ++i) {
    geometry_msgs::msg::Point32 point;
    surface_shape_.get_Point(i, point.x, point.y, point.z);
    pc_data_.points.push_back(point);
  }
  RCLCPP_INFO(
    node_->get_logger(), "Loaded mesh '%s' (%d points, %d triangles)",
    filename.c_str(), surface_shape_.get_Npoint(), surface_shape_.get_Nvertex());
  return true;
}

void IfsAlgorithm::get_WrenchAxis(const std::vector<float> & ft, WrenchAxis & out)
{
  const float force_norm = std::sqrt(ft[0] * ft[0] + ft[1] * ft[1] + ft[2] * ft[2]);
  if (force_norm <= std::numeric_limits<float>::epsilon()) {
    insert_wrench(out.f, 0.0F, 0.0F, 0.0F);
    insert_wrench(out.r0, 0.0F, 0.0F, 0.0F);
    return;
  }

  insert_wrench(out.f, ft[0] / force_norm, ft[1] / force_norm, ft[2] / force_norm);
  const float squared_norm = force_norm * force_norm;
  insert_wrench(
    out.r0,
    (ft[1] * ft[5] - ft[2] * ft[4]) / squared_norm,
    (ft[2] * ft[3] - ft[0] * ft[5]) / squared_norm,
    (ft[0] * ft[4] - ft[1] * ft[3]) / squared_norm);
}

bool IfsAlgorithm::is_SurfaceContact(
  int index, WrenchAxis wrench_axis, std::vector<int> & vertex_indices,
  std::vector<float> & contact_point)
{
  vertex_indices.clear();

  int f1 = 0;
  int f2 = 0;
  int f3 = 0;
  std::array<float, 3> v1 {};
  std::array<float, 3> v2 {};
  std::array<float, 3> v3 {};
  std::array<float, 3> normal {};
  surface_shape_.get_TriangleIndex(index, f1, f2, f3);
  surface_shape_.get_Triangle(index, v1.data(), v2.data(), v3.data(), normal.data());

  const float denominator =
    wrench_axis.f.at[0] * normal[0] +
    wrench_axis.f.at[1] * normal[1] +
    wrench_axis.f.at[2] * normal[2];
  if (std::abs(denominator) <= std::numeric_limits<float>::epsilon()) {
    return false;
  }

  const float lambda =
    ((v1[0] * normal[0] + v1[1] * normal[1] + v1[2] * normal[2]) -
    (wrench_axis.r0.at[0] * normal[0] + wrench_axis.r0.at[1] * normal[1] +
    wrench_axis.r0.at[2] * normal[2])) / denominator;

  std::array<float, 3> center {};
  std::array<float, 3> p1 {};
  std::array<float, 3> p2 {};
  std::array<float, 3> p3 {};
  std::array<float, 3> l1 {};
  std::array<float, 3> l2 {};
  std::array<float, 3> l3 {};
  for (std::size_t i = 0; i < 3; ++i) {
    contact_point[i] = wrench_axis.r0.at[i] + lambda * wrench_axis.f.at[i];
    center[i] = (v1[i] + v2[i] + v3[i]) / 3.0F;
    p1[i] = 0.5F * (v2[i] + v3[i]);
    p2[i] = 0.5F * (v1[i] + v3[i]);
    p3[i] = 0.5F * (v2[i] + v1[i]);
    l1[i] = v2[i] - v3[i];
    l2[i] = v1[i] - v3[i];
    l3[i] = v1[i] - v2[i];
  }

  const auto norm = [](const std::array<float, 3> & vector) {
      return std::sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
    };
  const float l1_norm = norm(l1);
  const float l2_norm = norm(l2);
  const float l3_norm = norm(l3);
  if (l1_norm <= 0.0F || l2_norm <= 0.0F || l3_norm <= 0.0F) {
    return false;
  }

  for (std::size_t i = 0; i < 3; ++i) {
    l1[i] /= l1_norm;
    l2[i] /= l2_norm;
    l3[i] /= l3_norm;
  }

  std::array<float, 3> n1 {};
  std::array<float, 3> n2 {};
  std::array<float, 3> n3 {};
  const auto projection = [](const std::array<float, 3> & a, const std::array<float, 3> & b) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
  std::array<float, 3> center_to_p1 {};
  std::array<float, 3> center_to_p2 {};
  std::array<float, 3> center_to_p3 {};
  for (std::size_t i = 0; i < 3; ++i) {
    center_to_p1[i] = center[i] - p1[i];
    center_to_p2[i] = center[i] - p2[i];
    center_to_p3[i] = center[i] - p3[i];
  }
  const float projection1 = projection(l1, center_to_p1);
  const float projection2 = projection(l2, center_to_p2);
  const float projection3 = projection(l3, center_to_p3);
  for (std::size_t i = 0; i < 3; ++i) {
    n1[i] = center_to_p1[i] - projection1 * l1[i];
    n2[i] = center_to_p2[i] - projection2 * l2[i];
    n3[i] = center_to_p3[i] - projection3 * l3[i];
  }

  const auto is_inside_edge = [&contact_point](
    const std::array<float, 3> & direction, const std::array<float, 3> & midpoint) {
      return direction[0] * (contact_point[0] - midpoint[0]) +
             direction[1] * (contact_point[1] - midpoint[1]) +
             direction[2] * (contact_point[2] - midpoint[2]) > 0.0F;
    };

  int inside_count = 0;
  if (is_inside_edge(n1, p1)) {
    vertex_indices.push_back(f1);
    ++inside_count;
  }
  if (is_inside_edge(n2, p2)) {
    vertex_indices.push_back(f2);
    ++inside_count;
  }
  if (is_inside_edge(n3, p3)) {
    vertex_indices.push_back(f3);
    ++inside_count;
  }
  return inside_count == 3;
}

int IfsAlgorithm::get_NextTriangle(int index, std::vector<int> & vertex_indices)
{
  if (vertex_indices.empty()) {
    return index;
  }

  std::array<int, 3> current {};
  surface_shape_.get_TriangleIndex(index, current[0], current[1], current[2]);

  for (int i = 0; i < surface_shape_.get_Nvertex(); ++i) {
    if (i == index) {
      continue;
    }
    std::array<int, 3> candidate {};
    surface_shape_.get_TriangleIndex(i, candidate[0], candidate[1], candidate[2]);
    const auto contains = [&candidate](int vertex) {
        return candidate[0] == vertex || candidate[1] == vertex || candidate[2] == vertex;
      };
    if (!contains(vertex_indices[0])) {
      continue;
    }

    if (vertex_indices.size() == 2 && contains(vertex_indices[1])) {
      return i;
    }
    if (vertex_indices.size() == 1) {
      int shared_vertices = 0;
      for (int candidate_vertex : candidate) {
        for (int current_vertex : current) {
          if (candidate_vertex == current_vertex) {
            ++shared_vertices;
          }
        }
      }
      if (shared_vertices == 1) {
        return i;
      }
    }
  }
  return index;
}

std::vector<float> IfsAlgorithm::get_ContactPoint(const std::vector<float> & ft)
{
  std::vector<float> contact_point(3, 0.0F);
  float absolute_sum = 0.0F;
  for (float value : ft) {
    absolute_sum += std::abs(value);
  }
  if (absolute_sum <= std::numeric_limits<float>::epsilon()) {
    return contact_point;
  }

  WrenchAxis wrench_axis {};
  get_WrenchAxis(ft, wrench_axis);
  int index = get_ClosestSurface(wrench_axis);
  std::vector<int> vertex_indices;
  for (int iteration = 0; iteration <= 100; ++iteration) {
    if (is_SurfaceContact(index, wrench_axis, vertex_indices, contact_point)) {
      return contact_point;
    }
    const int next_index = get_NextTriangle(index, vertex_indices);
    if (next_index == index) {
      break;
    }
    index = next_index;
  }
  return std::vector<float>(3, 0.0F);
}

void IfsAlgorithm::draw_arrow(
  const geometry_msgs::msg::PointStamped & contact_point,
  const geometry_msgs::msg::Wrench & force)
{
  visualization_msgs::msg::Marker arrow;
  arrow.header.stamp = node_->now();
  arrow.header.frame_id = contact_frame_;
  arrow.ns = "contact_force";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;

  const double magnitude = std::sqrt(
    force.force.x * force.force.x +
    force.force.y * force.force.y +
    force.force.z * force.force.z);
  if (magnitude <= std::numeric_limits<double>::epsilon()) {
    arrow.action = visualization_msgs::msg::Marker::DELETE;
    arrow_pub_->publish(arrow);
    return;
  }

  first_draw_ = false;
  arrow.action = visualization_msgs::msg::Marker::MODIFY;
  arrow.color.r = 1.0F;
  arrow.color.g = 0.0F;
  arrow.color.b = 0.0F;
  arrow.color.a = 1.0F;

  constexpr double length_ratio = 0.007;
  arrow.scale.x = magnitude * length_ratio;
  arrow.scale.y = magnitude * length_ratio * 0.1;
  arrow.scale.z = magnitude * length_ratio * 0.1;
  arrow.pose.position.x = contact_point.point.x - force.force.x * length_ratio;
  arrow.pose.position.y = contact_point.point.y - force.force.y * length_ratio;
  arrow.pose.position.z = contact_point.point.z - force.force.z * length_ratio;

  const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
  const Eigen::Vector3d force_vector(force.force.x, force.force.y, force.force.z);
  const Eigen::Quaterniond quaternion =
    Eigen::Quaterniond::FromTwoVectors(x_axis, force_vector.normalized());
  arrow.pose.orientation.w = quaternion.w();
  arrow.pose.orientation.x = quaternion.x();
  arrow.pose.orientation.y = quaternion.y();
  arrow.pose.orientation.z = quaternion.z();
  arrow_pub_->publish(arrow);
}

void IfsAlgorithm::delete_arrow(
  const geometry_msgs::msg::PointStamped & contact_point,
  const geometry_msgs::msg::Wrench & force)
{
  (void)contact_point;
  (void)force;
  visualization_msgs::msg::Marker arrow;
  arrow.header.stamp = node_->now();
  arrow.header.frame_id = contact_frame_;
  arrow.ns = "contact_force";
  arrow.id = 0;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.action = visualization_msgs::msg::Marker::DELETE;
  arrow_pub_->publish(arrow);
}

void IfsAlgorithm::run()
{
  constexpr double loop_hz = 100.0;
  constexpr double rate_alpha = 0.2;
  rclcpp::Rate loop_rate(loop_hz);
  while (rclcpp::ok()) {
    pc_data_.header.stamp = node_->now();
    pointcloud_pub_->publish(pc_data_);

    if (ft_measure_flag_) {
      geometry_msgs::msg::Wrench force;

      std_msgs::msg::Float64 magnitude_message;
      magnitude_message.data = force_magnitude_;
      force_magnitude_pub_->publish(magnitude_message);

      if (force_magnitude_ > contact_force_threshold_) {
        const double raw_rate = (force_magnitude_ - prev_force_) * loop_hz;
        prev_force_ = force_magnitude_;
        force_rate_ += (raw_rate - force_rate_) * rate_alpha;

        // 손을 떼는 동안에는 |F| 가 줄면서 접촉점 r0 = (f x t)/|f|^2 의 분모가
        // 작아진다. 센서 영점 오차가 남아 있으면 그 사이 접촉점이 원점 쪽으로
        // 끌려간다. 과거 최대 힘이 아니라 현재 힘의 감소율로 떼는 동작을 검출해
        // 마지막 접촉점을 유지하고 화살표를 지운다.
        const bool releasing =
          kReleaseRateThreshold > 0.0 && force_rate_ < -kReleaseRateThreshold;

        if (releasing) {
          delete_arrow(cp_pubdata_, geometry_msgs::msg::Wrench());
        } else if (count_ > 3) {
          cp_ = get_ContactPoint(ft_value_);
          constexpr double lambda = 1.0;
          const bool contact_was_zero =
            cp_pubdata_.point.x == 0.0 && cp_pubdata_.point.y == 0.0 && cp_pubdata_.point.z == 0.0;
          const bool result_is_zero = cp_[0] == 0.0F && cp_[1] == 0.0F && cp_[2] == 0.0F;

          if (contact_was_zero) {
            cp_pubdata_.point.x = cp_[0];
            cp_pubdata_.point.y = cp_[1];
            cp_pubdata_.point.z = cp_[2] + 0.0005;
          } else if (!result_is_zero) {
            cp_pubdata_.point.x = lambda * cp_[0] + (1.0 - lambda) * cp_pubdata_.point.x;
            cp_pubdata_.point.y = lambda * cp_[1] + (1.0 - lambda) * cp_pubdata_.point.y;
            cp_pubdata_.point.z =
              lambda * (cp_[2] + 0.001) + (1.0 - lambda) * cp_pubdata_.point.z;
          }

          force.force.x = ft_value_[0];
          force.force.y = ft_value_[1];
          force.force.z = ft_value_[2];
          draw_arrow(cp_pubdata_, force);
          if (result_is_zero) {
            delete_arrow(cp_pubdata_, force);
          }

          RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 500,
            "Contact: [%.4f %.4f %.4f] m, Force: [%.3f %.3f %.3f] N, "
            "|F|=%.3f N",
            cp_pubdata_.point.x, cp_pubdata_.point.y, cp_pubdata_.point.z,
            force.force.x, force.force.y, force.force.z,
            force_magnitude_);
        }
        ++count_;
      } else {
        cp_.assign(3, 0.0F);
        cp_pubdata_.point.x = 0.0;
        cp_pubdata_.point.y = 0.0;
        cp_pubdata_.point.z = 0.0;
        delete_arrow(cp_pubdata_, force);
        count_ = 0;
        prev_force_ = 0.0;
        force_rate_ = 0.0;
      }

      cp_pubdata_.header.stamp = node_->now();
      cp_pub_->publish(cp_pubdata_);
      ft_measure_flag_ = false;
    }

#ifdef LOG_DATA
    if (out_file_.is_open()) {
      for (float value : ft_value_) {
        out_file_ << value << '\t';
      }
      out_file_ << cp_[0] << '\t' << cp_[1] << '\t' << cp_[2] << '\n';
    }
#endif

    rclcpp::spin_some(node_);
    loop_rate.sleep();
  }
}
