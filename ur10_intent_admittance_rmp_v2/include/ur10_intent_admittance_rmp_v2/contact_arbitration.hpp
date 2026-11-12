#pragma once

// v2 (2026-11-07, "GM"): contact arbitration between the admittance flow (7)
// and the position / SO(3) targets (1, 2) without switching metrics on/off.
//
// 1. force_authority: the admittance transition s and the target reduction
//    follow the measured contact force through a smoothstep (low -> 0,
//    high -> 1) instead of a 0.3 s time ramp started by a threshold.
// 2. push_gradient_projection: instead of scaling the whole target metric by
//    (1 - a), the target pairs are pulled back through P = I - c g g^T with
//    g = normalize(Jc^T F_hat).  Joint motions orthogonal to g do not move
//    the contact point along the push to first order, so with c = 1 the
//    targets can neither resist nor assist the push, while they keep their
//    full authority in every other joint direction (orientation, height...).
//    A pullback through a fixed linear map keeps the RMP algebra intact:
//    M' = P^T M P, f' = P^T f.

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"

namespace ur10_intent_admittance_rmp_v2
{

inline double force_authority(double force_norm, double low, double high)
{
  if (!std::isfinite(force_norm) || !std::isfinite(low) || !std::isfinite(high) ||
    !(low >= 0.0) || !(high > low))
  {
    throw std::invalid_argument("invalid force authority input");
  }
  const double t = std::clamp((force_norm - low) / (high - low), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

struct PushProjection
{
  Matrix6 projection{Matrix6::Identity()};
  JointVector gradient{JointVector::Zero()};
  double strength{0.0};
  bool valid{false};
};

inline PushProjection push_gradient_projection(
  const Eigen::Matrix<double, 3, 6> & contact_jacobian, const Eigen::Vector3d & force,
  double strength, double tolerance = 1e-9)
{
  if (!contact_jacobian.allFinite() || !force.allFinite() || !std::isfinite(strength) ||
    strength < 0.0 || strength > 1.0)
  {
    throw std::invalid_argument("invalid push projection input");
  }
  PushProjection result;
  const double force_norm = force.norm();
  if (!(force_norm > tolerance) || !(strength > 0.0)) {
    return result;
  }
  JointVector gradient = contact_jacobian.transpose() * (force / force_norm);
  const double gradient_norm = gradient.norm();
  if (!(gradient_norm > tolerance)) {
    return result;  // the contact point cannot move along the push at all
  }
  gradient /= gradient_norm;
  result.gradient = gradient;
  result.strength = strength;
  result.projection = Matrix6::Identity() - strength * gradient * gradient.transpose();
  result.valid = true;
  return result;
}

// Pull a joint-space natural pair back through the (symmetric) projection.
inline void project_joint_pair(const Matrix6 & projection, Matrix6 & metric, JointVector & force)
{
  Matrix6 projected = projection.transpose() * metric * projection;
  metric = 0.5 * (projected + projected.transpose());
  force = projection.transpose() * force;
}

}  // namespace ur10_intent_admittance_rmp_v2
