#pragma once

// Intent-gated proximity avoidance (experimental v2 only).
//
// The production collision leaf is NOT re-implemented here.  Instead, a second
// instance of the production solver is configured so that only the
// external-obstacle collision path (root -> control_points -> collision) is
// enabled, and it is given only the proximity-sensor obstacles.  Its root sum
// (M_c, f_c) is exactly the proximity part of the production collision leaf.
// That pair is scaled by the avoidance weight w and handed to the main solver
// as a root external leaf, while the main solver receives every other obstacle
// unchanged.  With w = 1 the two paths add up to the production result.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "rb10_rmpflow_rviz/rmp_eigen_solver.hpp"

namespace ur10_intent_admittance_rmp_v2
{
namespace intent_avoidance
{

using rb10_rmpflow_rviz::EigenRmpConfig;
using rb10_rmpflow_rviz::ExternalRmpFeature;
using rb10_rmpflow_rviz::ObstacleSphere;
using rb10_rmpflow_rviz::RmpNodeConfig;
using GateJointVector = rb10_rmpflow_rviz::UR10Model::JointVector;
using GateMatrix6 = Eigen::Matrix<double, 6, 6>;

// A proximity-sensor obstacle is recognised by the source_id of a marker in the
// "proximity_obstacles" namespace.  The controller's own
// proximity_control_point_index cannot be relied on for UR10: it is filled
// from UR10Model::sensor_control_points, which is still empty, so it stays -1
// for every real sensor marker (found in the 2026-11-05 sensor dry-run).
// A non-negative index is still accepted for when those points are defined.
inline bool is_proximity_obstacle(
  const ObstacleSphere & obstacle,
  const std::unordered_set<std::int64_t> & proximity_source_ids)
{
  return obstacle.proximity_control_point_index >= 0 ||
         proximity_source_ids.count(obstacle.source_id) > 0;
}

struct ObstacleSplit
{
  std::vector<ObstacleSphere> proximity;
  // Everything the main solver keeps (fixed/manual obstacles).  Never empty:
  // like the production marker callback, a far placeholder is inserted when
  // nothing else remains so the collision task map keeps a valid shape.
  std::vector<ObstacleSphere> others;
};

inline ObstacleSplit split_obstacles(
  const std::vector<ObstacleSphere> & obstacles,
  const std::unordered_set<std::int64_t> & proximity_source_ids = {})
{
  ObstacleSplit split;
  for (const auto & obstacle : obstacles) {
    if (is_proximity_obstacle(obstacle, proximity_source_ids)) {
      split.proximity.push_back(obstacle);
    } else {
      split.others.push_back(obstacle);
    }
  }
  if (split.others.empty()) {
    split.others.push_back(ObstacleSphere{});
  }
  return split;
}

inline const RmpNodeConfig * find_node(
  const std::vector<RmpNodeConfig> & nodes, const std::string & name)
{
  for (const auto & node : nodes) {
    if (node.name == name) {
      return &node;
    }
  }
  return nullptr;
}

// A copy of `base` in which only `node_name` and its ancestors are enabled.
// Body guards, escape and diagnostics are removed so the returned root sum
// contains nothing but that one leaf's rows.  The node must be enabled in
// `base` with the given leaf and task map, so a gate can never introduce a
// leaf that the launched configuration had switched off.
inline EigenRmpConfig make_single_path_config(
  const EigenRmpConfig & base, const std::string & node_name,
  const std::string & leaf_type, const std::string & task_map_type)
{
  const auto * target = find_node(base.graph_nodes, node_name);
  if (target == nullptr || !target->enabled ||
    target->leaf_rmp_type != leaf_type || target->task_map_type != task_map_type)
  {
    throw std::invalid_argument(
            "gate requires an enabled '" + node_name + "' node (task_map " +
            task_map_type + ", leaf " + leaf_type + ") in the main graph");
  }
  std::unordered_set<std::string> keep;
  std::vector<std::string> pending{node_name};
  while (!pending.empty()) {
    const std::string name = pending.back();
    pending.pop_back();
    if (name == "root" || !keep.insert(name).second) {
      continue;
    }
    const auto * node = find_node(base.graph_nodes, name);
    if (node == nullptr) {
      throw std::invalid_argument("intent avoidance gate: missing graph node " + name);
    }
    for (const auto & parent : node->parents) {
      pending.push_back(parent);
    }
  }
  EigenRmpConfig config = base;
  for (auto & node : config.graph_nodes) {
    node.enabled = keep.count(node.name) > 0;
  }
  config.body_obstacles.clear();
  config.tangent_escape.enabled = false;
  config.escape_energy_certificate.enabled = false;
  config.enable_tangent_escape_diagnostics = false;
  config.enable_tangent_escape_dual_solve_diagnostics = false;
  config.enable_leaf_ablation_diagnostics = false;
  config.enable_stability_certificate_diagnostics = false;
  return config;
}

// Proximity avoidance: control_points -> collision (external obstacles).
inline EigenRmpConfig make_collision_only_config(
  const EigenRmpConfig & base, const std::string & collision_node = "collision")
{
  return make_single_path_config(base, collision_node, "collision", "collision_distance");
}

// SO(3) TCP orientation target (v2 stage 5, contact-gated like the target).
inline EigenRmpConfig make_orientation_only_config(
  const EigenRmpConfig & base, const std::string & orientation_node = "tcp_orientation_so3")
{
  return make_single_path_config(
    base, orientation_node, "so3_orientation_target", "link_orientation");
}

// `base` with one node switched off, for the main solver once that leaf is
// supplied through a gated external leaf instead.
inline EigenRmpConfig without_node(EigenRmpConfig base, const std::string & node_name)
{
  bool found = false;
  for (auto & node : base.graph_nodes) {
    if (node.name == node_name) {
      node.enabled = false;
      found = true;
    }
  }
  if (!found) {
    throw std::invalid_argument("gate: cannot disable missing graph node " + node_name);
  }
  return base;
}

// The main graph must contain an enabled root/identity external leaf that
// consumes `key`; otherwise the weighted avoidance would be silently dropped
// while the proximity obstacles are already removed from the main solver.
inline void require_root_external_leaf(const EigenRmpConfig & config, const std::string & key)
{
  for (const auto & node : config.graph_nodes) {
    if (node.enabled && node.leaf_rmp_type == "external" && node.target_key == key &&
      node.task_map_type == "identity" && node.parents.size() == 1U &&
      node.parents.front() == "root")
    {
      return;
    }
  }
  throw std::invalid_argument(
          "intent avoidance gate requires an enabled root/identity external leaf with "
          "target_key '" + key + "' in the main graph");
}

// External feature whose accumulation reproduces (w * metric, w * force) at
// the root:  metric_sqrt * metric_sqrt^T = w * metric  and
// w * metric * acceleration = w * force  (force lies in range(metric)).
// The threshold only guards against round-off: every direction with a real
// positive eigenvalue must be kept, because the solver multiplies the
// acceleration back by the same metric, so a small eigenvalue does not inflate
// the root force, while dropping it would discard part of the force.
inline std::optional<ExternalRmpFeature> weighted_root_feature(
  const GateMatrix6 & metric, const GateJointVector & force, double weight,
  double relative_threshold = 1e-15)
{
  const double w = std::clamp(weight, 0.0, 1.0);
  if (!(w > 0.0) || !metric.allFinite() || !force.allFinite()) {
    return std::nullopt;
  }
  const GateMatrix6 symmetric = 0.5 * (metric + metric.transpose());
  Eigen::SelfAdjointEigenSolver<GateMatrix6> eigensolver(symmetric);
  if (eigensolver.info() != Eigen::Success) {
    throw std::runtime_error("intent avoidance metric eigendecomposition failed");
  }
  const auto eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0);
  const double largest = eigenvalues.maxCoeff();
  if (!(largest > 0.0)) {
    return std::nullopt;
  }
  const GateMatrix6 basis = eigensolver.eigenvectors();
  const double threshold = largest * relative_threshold;
  GateJointVector acceleration = GateJointVector::Zero();
  for (int index = 0; index < 6; ++index) {
    if (eigenvalues[index] > threshold) {
      const auto direction = basis.col(index);
      acceleration += direction * direction.dot(force) / eigenvalues[index];
    }
  }
  const GateMatrix6 metric_sqrt =
    std::sqrt(w) * basis * eigenvalues.cwiseSqrt().asDiagonal() * basis.transpose();
  if (!metric_sqrt.allFinite() || !acceleration.allFinite()) {
    throw std::runtime_error("non-finite intent avoidance feature");
  }
  return ExternalRmpFeature{Eigen::MatrixXd(metric_sqrt), Eigen::VectorXd(acceleration)};
}

inline double slew_toward(double current, double target, double max_step)
{
  return current + std::clamp(target - current, -max_step, max_step);
}

}  // namespace intent_avoidance
}  // namespace ur10_intent_admittance_rmp_v2
