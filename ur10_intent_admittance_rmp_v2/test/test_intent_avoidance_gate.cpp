// Offline proof that the v2 intent-gated avoidance reproduces the production
// collision leaf.  Uses the real production solver (linked from
// rb10_rmpflow_rviz, read-only) and the same header the fork controller uses.

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rb10_rmpflow_rviz/pinocchio_direct_solver.hpp"
#include "ur10_intent_admittance_rmp_v2/intent_avoidance_gate.hpp"

namespace
{

using rb10_rmpflow_rviz::BodyObstacle;
using rb10_rmpflow_rviz::EigenRmpConfig;
using rb10_rmpflow_rviz::ExternalRmpFeature;
using rb10_rmpflow_rviz::ObstacleSphere;
using rb10_rmpflow_rviz::PinocchioDirectRmpSolver;
using rb10_rmpflow_rviz::RmpSolveResult;
using JointVector = rb10_rmpflow_rviz::UR10Model::JointVector;
namespace gate = ur10_intent_admittance_rmp_v2::intent_avoidance;

const char * kKey = "collision_gated";

std::string urdf_path()
{
  return ament_index_cpp::get_package_share_directory("rb10_rmpflow_rviz") + "/urdf/ur10.urdf";
}

// Production-like graph: joint limits, velocity cap, damping, body guards and
// the external-obstacle collision leaf, plus the root external leaf the fork
// uses for the gated proximity avoidance.  Collision values follow Stage 3.
EigenRmpConfig make_main_config()
{
  EigenRmpConfig config;
  config.solve_method = "rmp2";
  config.rmp_type = "canonical";
  config.solve_offset = 1e-3;
  auto & c = config.collision;
  c.policy = "lula_canonical";
  c.margin = 0.0;
  c.damping_gain = 30.0;
  c.damping_std_dev = 0.01;
  c.damping_robustness_eps = 0.1;
  c.damping_velocity_gate_length_scale = 0.03;
  c.repulsion_gain = 2800.0;
  c.repulsion_std_dev = 0.12;
  c.metric_modulation_radius = 0.50;
  c.metric_modulation_power = 1.5;
  c.metric_scalar = 1e5;
  c.metric_exploder_std_dev = 0.02;
  c.metric_exploder_eps = 1.5;
  for (auto & node : config.graph_nodes) {
    node.enabled =
      node.name == "joint_limits" || node.name == "joint_velocity_cap" ||
      node.name == "tcp_position" || node.name == "control_points" ||
      node.name == "collision" || node.name == "damping";
  }
  config.graph_nodes.push_back(rb10_rmpflow_rviz::make_rmp_node_config(
      "collision_gated_external", "root", "identity", "external", true, kKey));
  BodyObstacle guard;
  guard.type = "ball";
  guard.link_name = "shoulder_link";
  guard.center = Eigen::Vector3d(0.0, 0.0, -0.05);
  guard.radius = 0.075;
  config.body_obstacles.push_back(guard);
  return config;
}

ObstacleSphere sphere(const Eigen::Vector3d & center, double radius, int proximity_index)
{
  ObstacleSphere obstacle{center, radius};
  obstacle.proximity_control_point_index = proximity_index;
  return obstacle;
}

double relative_error(const Eigen::MatrixXd & a, const Eigen::MatrixXd & b)
{
  return (a - b).norm() / std::max({a.norm(), b.norm(), 1e-12});
}

class IntentAvoidanceGateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    main_config_ = make_main_config();
    main_ = std::make_unique<PinocchioDirectRmpSolver>(main_config_, urdf_path());
    avoidance_ = std::make_unique<PinocchioDirectRmpSolver>(
      gate::make_collision_only_config(main_config_), urdf_path());
    targets_.emplace("goal", Eigen::Vector3d(0.6, 0.2, 0.5));
  }

  // The fork's cycle: proximity obstacles go to the collision-only solver and
  // come back as a weighted root external leaf; the rest stay in the main solve.
  RmpSolveResult gated_solve(
    const JointVector & q, const JointVector & qd,
    const std::vector<ObstacleSphere> & obstacles, double weight,
    Eigen::MatrixXd * proximity_metric = nullptr,
    const std::unordered_set<std::int64_t> & proximity_ids = {}) const
  {
    const auto split = gate::split_obstacles(obstacles, proximity_ids);
    std::unordered_map<std::string, ExternalRmpFeature> external;
    if (!split.proximity.empty()) {
      const std::unordered_map<std::string, Eigen::Vector3d> none;
      const auto part = avoidance_->solve(q, qd, none, split.proximity);
      if (proximity_metric != nullptr) {
        *proximity_metric = part.metric;
      }
      if (const auto feature = gate::weighted_root_feature(part.metric, part.force, weight)) {
        external.emplace(kKey, *feature);
      }
    }
    return main_->solve(q, qd, targets_, split.others, external);
  }

  EigenRmpConfig main_config_;
  std::unique_ptr<PinocchioDirectRmpSolver> main_;
  std::unique_ptr<PinocchioDirectRmpSolver> avoidance_;
  std::unordered_map<std::string, Eigen::Vector3d> targets_;
};

// Random states and obstacles around the arm; several must actually engage the
// collision leaf for the comparison to mean anything.
TEST_F(IntentAvoidanceGateTest, FullWeightReproducesProductionCollision)
{
  std::mt19937 rng(20261105);
  std::uniform_real_distribution<double> joint(-2.5, 2.5);
  std::uniform_real_distribution<double> speed(-0.3, 0.3);
  std::uniform_real_distribution<double> coord(-0.9, 0.9);
  std::uniform_real_distribution<double> height(0.0, 1.2);
  int engaged = 0;
  for (int trial = 0; trial < 60; ++trial) {
    JointVector q;
    JointVector qd;
    for (int i = 0; i < 6; ++i) {
      q[i] = joint(rng);
      qd[i] = speed(rng);
    }
    std::vector<ObstacleSphere> obstacles{
      sphere(Eigen::Vector3d(coord(rng), coord(rng), height(rng)), 0.04, 0),
      sphere(Eigen::Vector3d(coord(rng), coord(rng), height(rng)), 0.04, 1),
      sphere(Eigen::Vector3d(coord(rng), coord(rng), height(rng)), 0.04, 3),
      // Fixed/manual obstacle: must stay with the main solver.
      sphere(Eigen::Vector3d(coord(rng), coord(rng), height(rng)), 0.10, -1),
    };
    const auto production = main_->solve(q, qd, targets_, obstacles);
    Eigen::MatrixXd proximity_metric = Eigen::MatrixXd::Zero(6, 6);
    const auto gated = gated_solve(q, qd, obstacles, 1.0, &proximity_metric);
    if (proximity_metric.norm() > 1e-6) {
      ++engaged;
    }
    EXPECT_LT(relative_error(production.metric, gated.metric), 1e-9) << "trial " << trial;
    EXPECT_LT(relative_error(production.force, gated.force), 1e-6) << "trial " << trial;
    EXPECT_LT(relative_error(production.qdd, gated.qdd), 1e-6) << "trial " << trial;
  }
  EXPECT_GE(engaged, 10) << "too few trials engaged the proximity collision rows";
}

TEST_F(IntentAvoidanceGateTest, ZeroWeightRemovesOnlyProximityAvoidance)
{
  JointVector q;
  q << 0.3, -1.2, 1.6, -1.8, -1.5, 0.0;
  JointVector qd;
  qd << 0.1, -0.05, 0.08, 0.0, 0.02, 0.0;
  const auto context_obstacle = sphere(Eigen::Vector3d(0.4, 0.3, 0.6), 0.10, -1);
  std::vector<ObstacleSphere> obstacles{
    sphere(Eigen::Vector3d(0.5, 0.1, 0.5), 0.04, 0), context_obstacle};
  const auto without_proximity = main_->solve(q, qd, targets_, {context_obstacle});
  const auto gated = gated_solve(q, qd, obstacles, 0.0);
  EXPECT_LT(relative_error(without_proximity.metric, gated.metric), 1e-12);
  EXPECT_LT(relative_error(without_proximity.qdd, gated.qdd), 1e-12);
}

TEST_F(IntentAvoidanceGateTest, PartialWeightScalesTheProximityMetric)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> joint(-2.0, 2.0);
  std::uniform_real_distribution<double> coord(-0.6, 0.6);
  int checked = 0;
  for (int trial = 0; trial < 40 && checked < 5; ++trial) {
    JointVector q;
    for (int i = 0; i < 6; ++i) {
      q[i] = joint(rng);
    }
    const JointVector qd = JointVector::Zero();
    std::vector<ObstacleSphere> obstacles{
      sphere(Eigen::Vector3d(coord(rng), coord(rng), 0.3 + coord(rng)), 0.04, 2)};
    Eigen::MatrixXd proximity_metric = Eigen::MatrixXd::Zero(6, 6);
    const auto base = gated_solve(q, qd, obstacles, 0.0);
    const auto partial = gated_solve(q, qd, obstacles, 0.4, &proximity_metric);
    if (proximity_metric.norm() < 1e-6) {
      continue;
    }
    ++checked;
    EXPECT_LT(
      relative_error(partial.metric - base.metric, 0.4 * proximity_metric), 1e-9);
  }
  EXPECT_GE(checked, 3);
}

// The real UR10 case: sensor markers arrive with proximity_control_point_index
// -1 (UR10Model::sensor_control_points is empty) and are recognised only by
// their marker source_id.  This is the case the first sensor dry-run exposed.
TEST_F(IntentAvoidanceGateTest, SensorMarkersWithoutIndexAreGatedBySourceId)
{
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> joint(-2.0, 2.0);
  std::uniform_real_distribution<double> coord(-0.7, 0.7);
  const std::unordered_set<std::int64_t> proximity_ids{101, 102};
  int engaged = 0;
  for (int trial = 0; trial < 40; ++trial) {
    JointVector q;
    for (int i = 0; i < 6; ++i) {
      q[i] = joint(rng);
    }
    const JointVector qd = JointVector::Zero();
    auto a = sphere(Eigen::Vector3d(coord(rng), coord(rng), 0.4 + coord(rng)), 0.04, -1);
    a.source_id = 101;
    auto b = sphere(Eigen::Vector3d(coord(rng), coord(rng), 0.4 + coord(rng)), 0.04, -1);
    b.source_id = 102;
    auto fixed = sphere(Eigen::Vector3d(coord(rng), coord(rng), 0.4 + coord(rng)), 0.10, -1);
    fixed.source_id = 999;
    const std::vector<ObstacleSphere> obstacles{a, b, fixed};

    const auto split = gate::split_obstacles(obstacles, proximity_ids);
    ASSERT_EQ(split.proximity.size(), 2U);
    ASSERT_EQ(split.others.size(), 1U);
    EXPECT_EQ(split.others.front().source_id, 999);

    Eigen::MatrixXd proximity_metric = Eigen::MatrixXd::Zero(6, 6);
    const auto production = main_->solve(q, qd, targets_, obstacles);
    const auto full = gated_solve(q, qd, obstacles, 1.0, &proximity_metric, proximity_ids);
    const auto off = gated_solve(q, qd, obstacles, 0.0, nullptr, proximity_ids);
    const auto fixed_only = main_->solve(q, qd, targets_, {fixed});
    if (proximity_metric.norm() > 1e-6) {
      ++engaged;
    }
    EXPECT_LT(relative_error(production.metric, full.metric), 1e-9) << "trial " << trial;
    EXPECT_LT(relative_error(production.qdd, full.qdd), 1e-6) << "trial " << trial;
    EXPECT_LT(relative_error(fixed_only.metric, off.metric), 1e-12) << "trial " << trial;
  }
  EXPECT_GE(engaged, 5);
}

// v2 stage 5: the SO(3) orientation leaf moved into a contact-gated root
// external leaf must reproduce the production leaf at scale 1 and shrink
// proportionally at the contact floor.
TEST(IntentOrientationGate, ScaleOneReproducesProductionSo3AndFloorShrinksIt)
{
  auto config = make_main_config();
  config.graph_nodes.push_back(rb10_rmpflow_rviz::make_rmp_node_config(
      "tcp_orientation_so3", "root", "link_orientation", "so3_orientation_target", true,
      "orientation_goal", "tool0"));
  config.graph_nodes.push_back(rb10_rmpflow_rviz::make_rmp_node_config(
      "orientation_gated_external", "root", "identity", "external", true, "orientation_gated"));
  const PinocchioDirectRmpSolver with_so3(config, urdf_path());
  const PinocchioDirectRmpSolver without_so3(
    gate::without_node(config, "tcp_orientation_so3"), urdf_path());
  const PinocchioDirectRmpSolver orientation_only(
    gate::make_orientation_only_config(config), urdf_path());
  EXPECT_NO_THROW(gate::require_root_external_leaf(config, "orientation_gated"));

  std::mt19937 rng(5);
  std::uniform_real_distribution<double> joint(-2.0, 2.0);
  std::uniform_real_distribution<double> speed(-0.2, 0.2);
  std::uniform_real_distribution<double> angle(-3.0, 3.0);
  const std::vector<ObstacleSphere> obstacles{
    sphere(Eigen::Vector3d(0.4, 0.2, 0.5), 0.10, -1)};
  int engaged = 0;
  for (int trial = 0; trial < 30; ++trial) {
    JointVector q;
    JointVector qd;
    for (int i = 0; i < 6; ++i) {
      q[i] = joint(rng);
      qd[i] = speed(rng);
    }
    const Eigen::Matrix3d goal_rotation =
      (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(angle(rng) / 3.0, Eigen::Vector3d::UnitY())).toRotationMatrix();
    std::unordered_map<std::string, Eigen::Vector3d> targets{
      {"goal", Eigen::Vector3d(0.6, 0.2, 0.5)},
      {"orientation_goal_x", goal_rotation.col(0)},
      {"orientation_goal_y", goal_rotation.col(1)},
      {"orientation_goal_z", goal_rotation.col(2)}};

    const auto production = with_so3.solve(q, qd, targets, obstacles);
    const auto part = orientation_only.solve(q, qd, targets, {ObstacleSphere{}});
    if (part.metric.norm() > 1e-6) {
      ++engaged;
    }
    std::unordered_map<std::string, ExternalRmpFeature> full_external;
    if (const auto feature = gate::weighted_root_feature(part.metric, part.force, 1.0)) {
      full_external.emplace("orientation_gated", *feature);
    }
    const auto gated_full = without_so3.solve(q, qd, targets, obstacles, full_external);
    EXPECT_LT(relative_error(production.metric, gated_full.metric), 1e-9) << "trial " << trial;
    EXPECT_LT(relative_error(production.force, gated_full.force), 1e-6) << "trial " << trial;
    EXPECT_LT(relative_error(production.qdd, gated_full.qdd), 1e-6) << "trial " << trial;

    constexpr double floor_scale = 0.00025;
    std::unordered_map<std::string, ExternalRmpFeature> floor_external;
    if (const auto feature = gate::weighted_root_feature(part.metric, part.force, floor_scale)) {
      floor_external.emplace("orientation_gated", *feature);
    }
    const auto gated_floor = without_so3.solve(q, qd, targets, obstacles, floor_external);
    const auto none = without_so3.solve(q, qd, targets, obstacles);
    EXPECT_LT(
      relative_error(gated_floor.metric - none.metric, floor_scale * part.metric), 1e-6)
      << "trial " << trial;
  }
  EXPECT_GE(engaged, 20);
}

TEST(IntentAvoidanceGateConfig, SplitKeepsAPlaceholderForTheMainSolver)
{
  const auto split = gate::split_obstacles({sphere(Eigen::Vector3d::Zero(), 0.04, 0)});
  ASSERT_EQ(split.proximity.size(), 1U);
  ASSERT_EQ(split.others.size(), 1U);
  EXPECT_LT(split.others.front().proximity_control_point_index, 0);
  EXPECT_GT(split.others.front().center.norm(), 10.0);
}

TEST(IntentAvoidanceGateConfig, CollisionOnlyConfigKeepsJustTheCollisionPath)
{
  const auto config = gate::make_collision_only_config(make_main_config());
  for (const auto & node : config.graph_nodes) {
    const bool expected = node.name == "control_points" || node.name == "collision";
    EXPECT_EQ(node.enabled, expected) << node.name;
  }
  EXPECT_TRUE(config.body_obstacles.empty());
}

TEST(IntentAvoidanceGateConfig, RejectsGraphsThatWouldDropAvoidance)
{
  auto no_collision = make_main_config();
  for (auto & node : no_collision.graph_nodes) {
    if (node.name == "collision") {
      node.enabled = false;
    }
  }
  EXPECT_THROW(gate::make_collision_only_config(no_collision), std::invalid_argument);

  auto no_external = make_main_config();
  no_external.graph_nodes.pop_back();
  EXPECT_THROW(gate::require_root_external_leaf(no_external, kKey), std::invalid_argument);
  EXPECT_NO_THROW(gate::require_root_external_leaf(make_main_config(), kKey));
}

}  // namespace
