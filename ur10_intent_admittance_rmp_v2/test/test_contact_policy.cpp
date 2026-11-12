#include <cmath>

#include <gtest/gtest.h>

#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"

namespace ur10_intent_admittance_rmp_v2
{

TEST(ContactPolicy, JacobianMatchesFiniteDifferenceAtOffsetPoint)
{
  JointVector q;
  q << 0.2, -1.0, 1.3, -1.5, -1.2, 0.4;
  const JointVector qd = JointVector::Zero();
  const Eigen::Vector3d local_point(-0.24, 0.035, 0.06);
  const auto geometry = contact_geometry(q, qd, Model::FOREARM_LINK, local_point);
  constexpr double h = 1e-6;
  for (int j = 0; j < 6; ++j) {
    JointVector plus = q;
    JointVector minus = q;
    plus[j] += h;
    minus[j] -= h;
    const Eigen::Vector3d numerical =
      (contact_geometry(plus, qd, Model::FOREARM_LINK, local_point).position -
       contact_geometry(minus, qd, Model::FOREARM_LINK, local_point).position) / (2.0 * h);
    EXPECT_LT((numerical - geometry.jacobian.col(j)).norm(), 1e-5);
  }
}

TEST(ContactPolicy, GateEndpointsAndForceDirection)
{
  JointVector q;
  q << 0.2, -1.0, 1.3, -1.5, -1.2, 0.4;
  const JointVector qd = JointVector::Zero();
  const auto geometry = contact_geometry(
    q, qd, Model::FOREARM_LINK, Eigen::Vector3d(-0.24, 0.035, 0.06));
  const Eigen::Vector3d force(0.0, 10.0, 0.0);
  const Eigen::Vector3d mass = Eigen::Vector3d::Constant(2.0);
  const Eigen::Vector3d damping = Eigen::Vector3d::Constant(20.0);
  const Eigen::Vector3d metric = Eigen::Vector3d::Constant(4.0);
  const auto off = contact_natural_pair(geometry, force, mass, damping, metric, 0.0);
  EXPECT_DOUBLE_EQ(off.metric.norm(), 0.0);
  EXPECT_DOUBLE_EQ(off.force.norm(), 0.0);
  const auto on = contact_natural_pair(geometry, force, mass, damping, metric, 1.0);
  EXPECT_LT((on.metric - 4.0 * geometry.jacobian.transpose() *
    geometry.jacobian).norm(), 1e-10);
  EXPECT_LT((on.force - geometry.jacobian.transpose() * (2.0 * force)).norm(), 1e-10);
  const JointVector qdd = resolve_pair(on, 1e-6);
  EXPECT_GT((geometry.jacobian * qdd).dot(force), 0.0);
}

TEST(ContactPolicy, ZeroForceDampsExistingContactVelocity)
{
  JointVector q;
  q << 0.2, -1.0, 1.3, -1.5, -1.2, 0.4;
  JointVector qd;
  qd << 0.1, -0.2, 0.15, 0.0, 0.0, 0.0;
  const auto geometry = contact_geometry(
    q, qd, Model::FOREARM_LINK, Eigen::Vector3d(-0.24, 0.035, 0.06));
  const auto pair = contact_natural_pair(
    geometry, Eigen::Vector3d::Zero(), Eigen::Vector3d::Constant(2.0),
    Eigen::Vector3d::Constant(20.0), Eigen::Vector3d::Constant(4.0), 1.0);
  const JointVector qdd = resolve_pair(pair, 1e-6);
  EXPECT_LT(geometry.velocity.dot(geometry.jacobian * qdd + geometry.curvature), 0.0);
}

TEST(ContactPolicy, JointLimitPreservesDirectionInsteadOfClippingComponents)
{
  JointVector raw;
  raw << -8.0, 3.0, 1.0, 0.0, 0.0, 0.0;
  const JointVector limited = direction_preserving_joint_limit(raw, 1.0);
  EXPECT_DOUBLE_EQ(limited.cwiseAbs().maxCoeff(), 1.0);
  EXPECT_LT((limited - raw / 8.0).norm(), 1e-12);
  EXPECT_GT(limited.dot(raw), 0.0);
}

TEST(ContactPolicy, PerJointLimitRaisesLateralAuthorityWithoutChangingDirection)
{
  JointVector raw;
  raw << -8.0, 3.0, 1.0, 0.0, 0.0, 0.0;
  JointVector limits;
  limits << 2.5, 1.5, 1.5, 1.5, 1.5, 1.5;
  const JointVector limited = direction_preserving_joint_limit(raw, limits);
  EXPECT_NEAR(limited[0], -2.5, 1e-12);
  EXPECT_LT((limited - raw * (2.5 / 8.0)).norm(), 1e-12);
  EXPECT_TRUE((limited.cwiseAbs().array() <= limits.array()).all());
  EXPECT_GT(limited.dot(raw), 0.0);
}

TEST(ContactPolicy, TaskLimitPreservesForceDirection)
{
  const Eigen::Vector3d raw(3.0, 4.0, 0.0);
  const Eigen::Vector3d limited = direction_preserving_task_limit(raw, 2.0);
  EXPECT_NEAR(limited.norm(), 2.0, 1e-12);
  EXPECT_LT(limited.cross(raw).norm(), 1e-12);
  EXPECT_GT(limited.dot(raw), 0.0);
}

TEST(ContactPolicy, DampedResolveSuppressesNearSingularExplosion)
{
  NaturalPair pair;
  pair.metric.setZero();
  pair.metric.diagonal() << 100.0, 10.0, 1e-6, 0.0, 0.0, 0.0;
  pair.force << 100.0, 10.0, 1.0, 0.0, 0.0, 0.0;
  const JointVector raw = resolve_pair(pair, 1e-12);
  const JointVector damped = resolve_pair_damped(pair, 0.05, 1e-8);
  EXPECT_TRUE(damped.allFinite());
  EXPECT_LT(damped.norm(), raw.norm());
  EXPECT_GT(damped.head<2>().dot(pair.force.head<2>()), 0.0);
}

TEST(ContactPolicy, TaskAlignmentDetectsOrthogonalAndAlignedMotion)
{
  const Eigen::Vector3d requested(1.0, 0.0, 0.0);
  EXPECT_NEAR(task_alignment_cosine(requested, Eigen::Vector3d(2.0, 0.0, 0.0)), 1.0, 1e-12);
  EXPECT_NEAR(task_alignment_cosine(requested, Eigen::Vector3d(0.0, 2.0, 0.0)), 0.0, 1e-12);
  EXPECT_NEAR(task_alignment_cosine(requested, Eigen::Vector3d(-2.0, 0.0, 0.0)), -1.0, 1e-12);
}

}  // namespace ur10_intent_admittance_rmp_v2
