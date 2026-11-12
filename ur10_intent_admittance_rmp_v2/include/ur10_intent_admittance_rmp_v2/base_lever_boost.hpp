#pragma once

// v2 goal 1-2: base-lever compensation for the admittance flow (7).
//
// shoulder_lift and elbow rotate about parallel axes, so they can only move a
// forearm contact point inside the arm plane.  Motion along the arm-plane
// normal n (= the lift axis) comes from shoulder_pan alone, with lever
// |n . J_pan|.  Near the base axis that lever is a few centimetres, so the
// admittance metric pulled back to the pan joint (w * lever^2) falls below
// the joint damping flow and the base barely turns.
//
// The compensation multiplies the admittance task metric along n by
// factor = clamp((reference_lever / lever)^2, 1, max_factor).  Because
// n . J_lift = n . J_elbow = 0 (and wrist columns are zero for a forearm
// contact), J^T (k n n^T) J only touches the pan-pan entry: the joint-space
// admittance metric changes for the base joint only.  The flow's desired task
// acceleration is unchanged; only its weight against the other flows grows.

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"

namespace ur10_intent_admittance_rmp_v2
{

struct BaseLeverWeight
{
  Eigen::Matrix3d weight{Eigen::Matrix3d::Zero()};
  double factor{1.0};
  double lever{0.0};
  bool applied{false};
};

// unreachable_lever (2026-11-07, merged items 5/7): when > 0 the factor is
// multiplied by lever^2 / (lever^2 + unreachable_lever^2), so the weight along
// the arm-plane normal goes smoothly to 0 as that direction becomes
// unreachable (lever -> 0).  This replaces the hard alignment rejection.
// 0 reproduces the goal 1-2 behaviour exactly.
inline BaseLeverWeight base_lever_weight(
  const Eigen::Matrix<double, 3, 6> & jacobian, const Eigen::Vector3d & metric_weight,
  double reference_lever, double max_factor, double unreachable_lever = 0.0,
  double degenerate_tolerance = 1e-6)
{
  if (!jacobian.allFinite() || !metric_weight.allFinite() ||
    (metric_weight.array() < 0.0).any() || !std::isfinite(reference_lever) ||
    reference_lever <= 0.0 || !std::isfinite(max_factor) || max_factor < 1.0 ||
    !std::isfinite(unreachable_lever) || unreachable_lever < 0.0)
  {
    throw std::invalid_argument("invalid base lever compensation input");
  }
  BaseLeverWeight result;
  result.weight = metric_weight.asDiagonal();
  const Eigen::Vector3d normal_raw = jacobian.col(1).cross(jacobian.col(2));
  const double normal_norm = normal_raw.norm();
  if (!(normal_norm > degenerate_tolerance)) {
    return result;  // contact point collinear with lift/elbow axes: leave unchanged
  }
  const Eigen::Vector3d normal = normal_raw / normal_norm;
  result.lever = std::abs(normal.dot(jacobian.col(0)));
  const double ratio = reference_lever / std::max(result.lever, 1e-9);
  result.factor = std::clamp(ratio * ratio, 1.0, max_factor);
  if (unreachable_lever > 0.0) {
    const double l2 = result.lever * result.lever;
    result.factor *= l2 / (l2 + unreachable_lever * unreachable_lever);
  }
  if (result.factor != 1.0) {
    const double along = normal.dot(result.weight * normal);
    result.weight += (result.factor - 1.0) * along * normal * normal.transpose();
    result.applied = true;
  }
  return result;
}

// contact_natural_pair with a full (symmetric PSD) task weight instead of a
// diagonal one.  Identical formula; with weight = diag(w) it reproduces
// contact_natural_pair exactly.
inline NaturalPair contact_natural_pair_weighted(
  const ContactGeometry & geometry, const Eigen::Vector3d & force_world,
  const Eigen::Vector3d & virtual_mass, const Eigen::Vector3d & damping,
  const Eigen::Matrix3d & weight, double transition)
{
  if (!force_world.allFinite() || !virtual_mass.allFinite() ||
    !damping.allFinite() || !weight.allFinite() ||
    !std::isfinite(transition) || transition < 0.0 || transition > 1.0 ||
    (virtual_mass.array() <= 0.0).any() || (damping.array() < 0.0).any())
  {
    throw std::invalid_argument("invalid contact policy parameter");
  }
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

}  // namespace ur10_intent_admittance_rmp_v2
