#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "ur10_intent_admittance_rmp_v2/base_lever_boost.hpp"
#include "ur10_intent_admittance_rmp_v2/contact_arbitration.hpp"

namespace ur10_intent_admittance_rmp_v2
{
namespace
{

JointVector random_joints(std::mt19937 & rng, double low, double high)
{
  std::uniform_real_distribution<double> dist(low, high);
  JointVector q;
  for (int j = 0; j < 6; ++j) {
    q[j] = dist(rng);
  }
  return q;
}

// A random symmetric positive semi-definite "target" pair of rank <= 3, like
// J_tcp^T M J_tcp with M = 1e5 I.
void random_target_pair(std::mt19937 & rng, Matrix6 & metric, JointVector & force)
{
  std::normal_distribution<double> n(0.0, 1.0);
  Eigen::Matrix<double, 3, 6> j;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 6; ++c) {
      j(r, c) = n(rng);
    }
  }
  metric = 1e5 * j.transpose() * j;
  Eigen::Vector3d a(n(rng), n(rng), n(rng));
  force = 1e5 * j.transpose() * a;
}

}  // namespace

TEST(ContactArbitration, ForceAuthorityIsASmoothMonotoneRamp)
{
  EXPECT_DOUBLE_EQ(force_authority(0.0, 2.0, 6.0), 0.0);
  EXPECT_DOUBLE_EQ(force_authority(2.0, 2.0, 6.0), 0.0);
  EXPECT_DOUBLE_EQ(force_authority(4.0, 2.0, 6.0), 0.5);
  EXPECT_DOUBLE_EQ(force_authority(6.0, 2.0, 6.0), 1.0);
  EXPECT_DOUBLE_EQ(force_authority(30.0, 2.0, 6.0), 1.0);
  double previous = 0.0;
  for (double f = 0.0; f <= 8.0; f += 0.01) {
    const double a = force_authority(f, 2.0, 6.0);
    EXPECT_GE(a, previous - 1e-15);
    EXPECT_LE(a - previous, 0.01 * 1.5 / 4.0 + 1e-12);  // bounded slope: no jumps
    previous = a;
  }
  EXPECT_THROW(force_authority(1.0, 6.0, 2.0), std::invalid_argument);
}

// Core property: with full strength the projected target pair cannot move the
// contact point along the push (neither resist nor assist), for random arm
// configurations, contact points and push directions.
TEST(ContactArbitration, ProjectedTargetCannotMoveContactPointAlongPush)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> offset(-0.4, 0.05);
  std::normal_distribution<double> n(0.0, 1.0);
  int checked = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const JointVector q = random_joints(rng, -3.0, 3.0);
    const Eigen::Vector3d point(offset(rng), 0.06 * n(rng), 0.06 * n(rng));
    const auto geometry = contact_geometry(q, JointVector::Zero(), Model::FOREARM_LINK, point);
    const Eigen::Vector3d force(10 * n(rng), 10 * n(rng), 10 * n(rng));
    const auto push = push_gradient_projection(geometry.jacobian, force, 1.0);
    if (!push.valid) {
      continue;
    }
    Matrix6 metric;
    JointVector target_force;
    random_target_pair(rng, metric, target_force);
    project_joint_pair(push.projection, metric, target_force);
    // the projected metric does not see motion along g, and its force has no g part
    EXPECT_LT((metric * push.gradient).norm(), 1e-6 * metric.norm() + 1e-9) << trial;
    EXPECT_LT(std::abs(push.gradient.dot(target_force)), 1e-6 * target_force.norm() + 1e-9);
    // the target's own resolved acceleration does not move the contact point along F
    const Eigen::JacobiSVD<Matrix6> svd(metric, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const JointVector qdd = svd.solve(target_force);
    const double along = force.normalized().dot(geometry.jacobian * qdd);
    EXPECT_LT(std::abs(along), 1e-6 * (geometry.jacobian * qdd).norm() + 1e-9) << trial;
    // and it stays a valid (symmetric PSD) metric
    EXPECT_LT((metric - metric.transpose()).norm(), 1e-9 * metric.norm());
    Eigen::SelfAdjointEigenSolver<Matrix6> eig(metric);
    EXPECT_GT(eig.eigenvalues().minCoeff(), -1e-6 * metric.norm());
    ++checked;
  }
  EXPECT_GT(checked, 250);
}

TEST(ContactArbitration, ZeroStrengthOrForceLeavesTargetUnchanged)
{
  std::mt19937 rng(3);
  const JointVector q = random_joints(rng, -2.0, 2.0);
  const auto geometry = contact_geometry(
    q, JointVector::Zero(), Model::FOREARM_LINK, Eigen::Vector3d(-0.24, 0.035, 0.06));
  EXPECT_FALSE(push_gradient_projection(geometry.jacobian, Eigen::Vector3d(0, 10, 0), 0.0).valid);
  EXPECT_FALSE(push_gradient_projection(geometry.jacobian, Eigen::Vector3d::Zero(), 1.0).valid);
  const auto none = push_gradient_projection(geometry.jacobian, Eigen::Vector3d::Zero(), 1.0);
  Matrix6 metric;
  JointVector force;
  random_target_pair(rng, metric, force);
  const Matrix6 m0 = metric;
  const JointVector f0 = force;
  project_joint_pair(none.projection, metric, force);
  EXPECT_LT((metric - m0).norm(), 1e-9 * m0.norm());
  EXPECT_LT((force - f0).norm(), 1e-9 * f0.norm());
  EXPECT_THROW(
    push_gradient_projection(geometry.jacobian, Eigen::Vector3d(0, 1, 0), 1.5),
    std::invalid_argument);
}

TEST(ContactArbitration, PartialStrengthScalesOnlyTheGradientDirection)
{
  std::mt19937 rng(5);
  const JointVector q = random_joints(rng, -2.0, 2.0);
  const auto geometry = contact_geometry(
    q, JointVector::Zero(), Model::FOREARM_LINK, Eigen::Vector3d(-0.3, 0.0, 0.06));
  const auto push = push_gradient_projection(geometry.jacobian, Eigen::Vector3d(1, -2, 0.5), 0.4);
  ASSERT_TRUE(push.valid);
  EXPECT_LT((push.projection * push.gradient - 0.6 * push.gradient).norm(), 1e-12);
  JointVector other = JointVector::Random();
  other -= other.dot(push.gradient) * push.gradient;
  EXPECT_LT((push.projection * other - other).norm(), 1e-12);
}

TEST(ContactArbitration, UnreachableLeverTailGoesToZeroAndDefaultsToGoal12)
{
  std::mt19937 rng(9);
  const Eigen::Vector3d metric = Eigen::Vector3d::Constant(1200.0);
  int seen_short = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const JointVector q = random_joints(rng, -3.0, 3.0);
    const auto geometry = contact_geometry(
      q, JointVector::Zero(), Model::FOREARM_LINK, Eigen::Vector3d(-0.24, 0.035, 0.06));
    const auto plain = base_lever_weight(geometry.jacobian, metric, 0.28, 20.0);
    const auto zero_tail = base_lever_weight(geometry.jacobian, metric, 0.28, 20.0, 0.0);
    EXPECT_DOUBLE_EQ(plain.factor, zero_tail.factor);
    EXPECT_LT((plain.weight - zero_tail.weight).norm(), 1e-12);
    const auto tail = base_lever_weight(geometry.jacobian, metric, 0.28, 20.0, 0.01);
    const double l2 = tail.lever * tail.lever;
    EXPECT_NEAR(tail.factor, plain.factor * l2 / (l2 + 1e-4), 1e-12);
    if (tail.lever < 0.005) {
      ++seen_short;
      EXPECT_LT(tail.factor, 0.2 * plain.factor + 1e-12);
    }
  }
  EXPECT_GT(seen_short, 0);
  // limit: lever -> 0 makes the normal weight vanish
  const double l = 1e-5;  // 20 * l^2 / l0^2 = 2e-5
  EXPECT_LT(20.0 * l * l / (l * l + 1e-4), 3e-5);
}

}  // namespace ur10_intent_admittance_rmp_v2
