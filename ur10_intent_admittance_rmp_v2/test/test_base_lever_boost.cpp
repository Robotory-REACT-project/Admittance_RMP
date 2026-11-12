#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "ur10_intent_admittance_rmp_v2/base_lever_boost.hpp"

namespace ur10_intent_admittance_rmp_v2
{
namespace
{

// Folded test pose from the 2026-11-06 hardware run (cover ~7.5 cm lever).
JointVector folded_pose()
{
  JointVector q;
  q << 2.931, -0.955, -2.030, -1.711, 1.482, -3.081;
  return q;
}

JointVector far_pose()
{
  JointVector q;
  q << 2.40, -1.58, -1.32, -1.69, 1.48, -3.08;
  return q;
}

const Eigen::Vector3d kLocalPoint(-0.24, 0.035, 0.06);
const Eigen::Vector3d kMetric = Eigen::Vector3d::Constant(1200.0);

}  // namespace

TEST(BaseLeverBoost, UnitWeightReproducesContactNaturalPair)
{
  JointVector qd;
  qd << 0.1, -0.05, 0.08, 0.02, -0.03, 0.01;
  const auto geometry = contact_geometry(folded_pose(), qd, Model::FOREARM_LINK, kLocalPoint);
  const Eigen::Vector3d force(3.0, -12.0, 4.0);
  const Eigen::Vector3d mass = Eigen::Vector3d::Constant(3.0);
  const Eigen::Vector3d damping = Eigen::Vector3d::Constant(25.0);
  const auto reference = contact_natural_pair(geometry, force, mass, damping, kMetric, 0.7);
  const auto weighted = contact_natural_pair_weighted(
    geometry, force, mass, damping, Eigen::Matrix3d(kMetric.asDiagonal()), 0.7);
  EXPECT_LT((reference.metric - weighted.metric).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((reference.force - weighted.force).cwiseAbs().maxCoeff(), 1e-9);
}

TEST(BaseLeverBoost, FoldedPoseLeverAndFactor)
{
  const auto geometry = contact_geometry(
    folded_pose(), JointVector::Zero(), Model::FOREARM_LINK, kLocalPoint);
  const auto boost = base_lever_weight(geometry.jacobian, kMetric, 0.28, 20.0);
  EXPECT_TRUE(boost.applied);
  EXPECT_GT(boost.lever, 0.0);
  EXPECT_LT(boost.lever, 0.15);
  const double expected = std::clamp(std::pow(0.28 / boost.lever, 2.0), 1.0, 20.0);
  EXPECT_NEAR(boost.factor, expected, 1e-12);
}

TEST(BaseLeverBoost, LongLeverLeavesWeightUnchanged)
{
  const auto geometry = contact_geometry(
    far_pose(), JointVector::Zero(), Model::FOREARM_LINK, kLocalPoint);
  const auto boost = base_lever_weight(geometry.jacobian, kMetric, 0.10, 20.0);
  EXPECT_GT(boost.lever, 0.10);
  EXPECT_FALSE(boost.applied);
  EXPECT_DOUBLE_EQ(boost.factor, 1.0);
  EXPECT_LT((boost.weight - Eigen::Matrix3d(kMetric.asDiagonal())).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(BaseLeverBoost, FactorIsCappedNearTheAxis)
{
  const auto geometry = contact_geometry(
    folded_pose(), JointVector::Zero(), Model::FOREARM_LINK, kLocalPoint);
  const auto boost = base_lever_weight(geometry.jacobian, kMetric, 5.0, 20.0);
  EXPECT_DOUBLE_EQ(boost.factor, 20.0);
}

// The core claim: only the pan-pan entry of the joint-space admittance metric
// (and only the pan entry of its force) changes; every other entry is
// identical, for random arm configurations and contact points on the forearm.
TEST(BaseLeverBoost, OnlyThePanEntryOfTheJointMetricChanges)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> angle(-3.0, 3.0);
  std::uniform_real_distribution<double> offset(-0.3, 0.05);
  std::uniform_real_distribution<double> speed(-0.3, 0.3);
  const Eigen::Vector3d mass = Eigen::Vector3d::Constant(3.0);
  const Eigen::Vector3d damping = Eigen::Vector3d::Constant(25.0);
  int applied = 0;
  for (int trial = 0; trial < 200; ++trial) {
    JointVector q;
    JointVector qd;
    for (int j = 0; j < 6; ++j) {
      q[j] = angle(rng);
      qd[j] = speed(rng);
    }
    const Eigen::Vector3d point(offset(rng), 0.06 * std::cos(trial), 0.06 * std::sin(trial));
    const auto geometry = contact_geometry(q, qd, Model::FOREARM_LINK, point);
    const auto boost = base_lever_weight(geometry.jacobian, kMetric, 0.28, 20.0);
    const Eigen::Vector3d force(10.0 * std::cos(trial), 8.0, -5.0);
    const auto base = contact_natural_pair(geometry, force, mass, damping, kMetric, 0.9);
    const auto boosted = contact_natural_pair_weighted(
      geometry, force, mass, damping, boost.weight, 0.9);
    Matrix6 diff = boosted.metric - base.metric;
    const double pan_delta = diff(0, 0);
    diff(0, 0) = 0.0;
    const double scale = std::max(1.0, base.metric.cwiseAbs().maxCoeff());
    EXPECT_LT(diff.cwiseAbs().maxCoeff() / scale, 1e-9) << "trial " << trial;
    JointVector force_diff = boosted.force - base.force;
    force_diff[0] = 0.0;
    EXPECT_LT(force_diff.cwiseAbs().maxCoeff() / std::max(1.0, base.force.cwiseAbs().maxCoeff()),
      1e-9) << "trial " << trial;
    if (boost.applied) {
      ++applied;
      // pan entry grows by exactly s^2 * (factor - 1) * w * lever^2
      EXPECT_NEAR(
        pan_delta, 0.81 * (boost.factor - 1.0) * 1200.0 * boost.lever * boost.lever,
        1e-6 * scale);
      EXPECT_GT(pan_delta, 0.0);
    }
  }
  EXPECT_GT(applied, 20);
}

TEST(BaseLeverBoost, RejectsInvalidInput)
{
  const auto geometry = contact_geometry(
    folded_pose(), JointVector::Zero(), Model::FOREARM_LINK, kLocalPoint);
  EXPECT_THROW(base_lever_weight(geometry.jacobian, kMetric, 0.0, 20.0), std::invalid_argument);
  EXPECT_THROW(base_lever_weight(geometry.jacobian, kMetric, 0.28, 0.5), std::invalid_argument);
}

}  // namespace ur10_intent_admittance_rmp_v2
