#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <Eigen/Dense>

#include "rb10_rmpflow_rviz/ur10_model.hpp"

namespace ur10_intent_admittance_rmp_v2
{

using Model = rb10_rmpflow_rviz::UR10Model;
using JointVector = Model::JointVector;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

struct ContactGeometry
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 3, 6> jacobian{Eigen::Matrix<double, 3, 6>::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d curvature{Eigen::Vector3d::Zero()};
};

inline ContactGeometry contact_geometry(
  const JointVector & q, const JointVector & qd,
  std::size_t link, const Eigen::Vector3d & local_point)
{
  if (link >= Model::LINK_COUNT || !q.allFinite() || !qd.allFinite() ||
    !local_point.allFinite())
  {
    throw std::invalid_argument("invalid contact geometry input");
  }
  const auto context = Model::forward_context(
    q, rb10_rmpflow_rviz::ControlPointJacobianMode::kNone);
  ContactGeometry result;
  result.position = context.link_positions[link] +
    context.link_rotations[link] * local_point;
  result.jacobian = Model::point_jacobian(
    result.position, context.joint_origins, context.joint_axes,
    Model::active_joint_count_for_link(link));
  result.velocity = result.jacobian * qd;

  // Jdot*qdot is the task-map curvature. The centred directional derivative
  // uses the same UR10 model and a fixed link-local contact point.
  if (qd.squaredNorm() > 0.0) {
    constexpr double h = 1e-5;
    const auto jacobian_at = [&](const JointVector & q_sample) {
      const auto sample = Model::forward_context(
        q_sample, rb10_rmpflow_rviz::ControlPointJacobianMode::kNone);
      const Eigen::Vector3d point = sample.link_positions[link] +
        sample.link_rotations[link] * local_point;
      return Model::point_jacobian(
        point, sample.joint_origins, sample.joint_axes,
        Model::active_joint_count_for_link(link));
    };
    result.curvature =
      ((jacobian_at(q + h * qd) - jacobian_at(q - h * qd)) / (2.0 * h)) * qd;
  }
  return result;
}

struct NaturalPair
{
  Matrix6 metric{Matrix6::Zero()};
  JointVector force{JointVector::Zero()};
};

inline NaturalPair contact_natural_pair(
  const ContactGeometry & geometry, const Eigen::Vector3d & force_world,
  const Eigen::Vector3d & virtual_mass, const Eigen::Vector3d & damping,
  const Eigen::Vector3d & metric_weight, double transition)
{
  if (!force_world.allFinite() || !virtual_mass.allFinite() ||
    !damping.allFinite() || !metric_weight.allFinite() ||
    !std::isfinite(transition) || transition < 0.0 || transition > 1.0 ||
    (virtual_mass.array() <= 0.0).any() ||
    (damping.array() < 0.0).any() || (metric_weight.array() < 0.0).any())
  {
    throw std::invalid_argument("invalid contact policy parameter");
  }
  const Eigen::Matrix3d weight = metric_weight.asDiagonal();
  const Eigen::Vector3d inv_mass_force = force_world.cwiseQuotient(virtual_mass);
  const Eigen::Vector3d inv_mass_damping =
    damping.cwiseProduct(geometry.velocity).cwiseQuotient(virtual_mass);
  const double s = transition;
  NaturalPair result;
  result.metric = s * s * geometry.jacobian.transpose() * weight * geometry.jacobian;
  result.force = geometry.jacobian.transpose() * weight *
    (s * s * (inv_mass_force - geometry.curvature) - s * inv_mass_damping);
  return result;
}

inline JointVector resolve_pair(const NaturalPair & pair, double relative_offset)
{
  if (!std::isfinite(relative_offset) || relative_offset <= 0.0) {
    throw std::invalid_argument("root offset must be positive");
  }
  const double scale = std::max(1.0, pair.metric.cwiseAbs().maxCoeff() * 0.01);
  const Matrix6 regularized = pair.metric / scale +
    relative_offset * Matrix6::Identity();
  const JointVector acceleration = regularized.ldlt().solve(pair.force / scale);
  if (!acceleration.allFinite()) {
    throw std::runtime_error("non-finite root acceleration");
  }
  return acceleration;
}

inline JointVector resolve_pair_damped(
  const NaturalPair & pair, double singular_value_damping_ratio,
  double eigenvalue_floor)
{
  if (!std::isfinite(singular_value_damping_ratio) ||
    singular_value_damping_ratio < 0.0 || !std::isfinite(eigenvalue_floor) ||
    eigenvalue_floor <= 0.0)
  {
    throw std::invalid_argument("invalid damped resolve parameter");
  }

  Eigen::SelfAdjointEigenSolver<Matrix6> eigensolver(pair.metric);
  if (eigensolver.info() != Eigen::Success) {
    throw std::runtime_error("contact metric eigendecomposition failed");
  }
  const auto eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0);
  const Matrix6 basis = eigensolver.eigenvectors();
  const double maximum_eigenvalue = eigenvalues.maxCoeff();
  const double regularizer = std::max(
    eigenvalue_floor,
    singular_value_damping_ratio * singular_value_damping_ratio * maximum_eigenvalue);

  JointVector acceleration = JointVector::Zero();
  for (int index = 0; index < 6; ++index) {
    if (eigenvalues[index] > eigenvalue_floor) {
      const auto direction = basis.col(index);
      acceleration += direction * direction.dot(pair.force) /
        (eigenvalues[index] + regularizer);
    }
  }
  if (!acceleration.allFinite()) {
    throw std::runtime_error("non-finite damped contact acceleration");
  }
  return acceleration;
}

inline JointVector direction_preserving_joint_limit(
  const JointVector & value, double maximum_absolute_component)
{
  if (!value.allFinite() || !std::isfinite(maximum_absolute_component) ||
    maximum_absolute_component <= 0.0)
  {
    throw std::invalid_argument("invalid direction-preserving joint limit input");
  }
  const double peak = value.cwiseAbs().maxCoeff();
  if (peak <= maximum_absolute_component) {
    return value;
  }
  return value * (maximum_absolute_component / peak);
}

inline JointVector direction_preserving_joint_limit(
  const JointVector & value, const JointVector & maximum_absolute_components)
{
  if (!value.allFinite() || !maximum_absolute_components.allFinite() ||
    (maximum_absolute_components.array() <= 0.0).any())
  {
    throw std::invalid_argument("invalid direction-preserving per-joint limit input");
  }
  double scale = 1.0;
  for (int joint = 0; joint < value.size(); ++joint) {
    const double magnitude = std::abs(value[joint]);
    if (magnitude > maximum_absolute_components[joint]) {
      scale = std::min(scale, maximum_absolute_components[joint] / magnitude);
    }
  }
  return value * scale;
}

inline Eigen::Vector3d direction_preserving_task_limit(
  const Eigen::Vector3d & value, double maximum_norm)
{
  if (!value.allFinite() || !std::isfinite(maximum_norm) || maximum_norm <= 0.0) {
    throw std::invalid_argument("invalid direction-preserving task limit input");
  }
  const double norm = value.norm();
  if (norm <= maximum_norm) {
    return value;
  }
  return value * (maximum_norm / norm);
}

inline double task_alignment_cosine(
  const Eigen::Vector3d & requested, const Eigen::Vector3d & achieved)
{
  if (!requested.allFinite() || !achieved.allFinite()) {
    throw std::invalid_argument("invalid task alignment input");
  }
  const double denominator = requested.norm() * achieved.norm();
  if (denominator <= 1e-12) {
    return 0.0;
  }
  return std::clamp(requested.dot(achieved) / denominator, -1.0, 1.0);
}

}  // namespace ur10_intent_admittance_rmp_v2
