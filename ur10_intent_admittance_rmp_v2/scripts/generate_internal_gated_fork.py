#!/usr/bin/env python3
"""Generate an isolated controller/launch fork without editing production files."""

import argparse
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one source anchor, found {count}")
    return text.replace(old, new, 1)


def generate_controller(source_root: Path, output: Path) -> None:
    source = (source_root / "src" / "rmpflow_controller_node.cpp").read_text()
    source = replace_once(
        source,
        "    config.command_guard_max_velocity_rad_s = command_guard_max_velocity_rad_s_;\n",
        "    config.command_guard_max_velocity_rad_s = command_guard_max_velocity_rad_s_;\n"
        "    // v2: avoidance-only guard raise (0 / normal value = production behaviour).\n"
        "    // Step and following-error limits scale with it so the raised velocity\n"
        "    // is actually reachable at the same control rate.\n"
        "    {\n"
        "      const double raised = internal_guard_velocity_.load();\n"
        "      if (command_guard_max_velocity_rad_s_ > 0.0 &&\n"
        "        raised > command_guard_max_velocity_rad_s_)\n"
        "      {\n"
        "        const double ratio = raised / command_guard_max_velocity_rad_s_;\n"
        "        config.command_guard_max_velocity_rad_s = raised;\n"
        "        config.command_guard_max_step_rad = command_guard_max_step_rad_ * ratio;\n"
        "        config.command_guard_max_following_error_rad =\n"
        "          command_guard_max_following_error_rad_ * ratio;\n"
        "      }\n"
        "    }\n",
        "v2 avoidance guard in command map config",
    )
    source = replace_once(
        source,
        '#include "geometry_msgs/msg/point.hpp"\n',
        '#include "geometry_msgs/msg/point.hpp"\n'
        '#include "geometry_msgs/msg/point_stamped.hpp"\n'
        '#include "geometry_msgs/msg/wrench_stamped.hpp"\n',
        "contact message includes",
    )
    source = replace_once(
        source,
        '#include "std_msgs/msg/color_rgba.hpp"\n',
        '#include "std_msgs/msg/color_rgba.hpp"\n#include "std_msgs/msg/float64.hpp"\n',
        "Float64 include",
    )
    source = replace_once(
        source,
        '#include "rb10_rmpflow_rviz/ur10_self_collision_guard.hpp"\n',
        '#include "rb10_rmpflow_rviz/ur10_self_collision_guard.hpp"\n'
        '#include "ur10_intent_admittance_rmp_v2/contact_policy.hpp"\n'
        '#include "ur10_intent_admittance_rmp_v2/intent_avoidance_gate.hpp"\n'
        '#include "ur10_intent_admittance_rmp_v2/base_lever_boost.hpp"\n'
        '#include "ur10_intent_admittance_rmp_v2/contact_arbitration.hpp"\n'
        '#include "ur10_intent_admittance_rmp_v2/contact_intent.hpp"\n',
        "contact policy include",
    )
    source = replace_once(
        source,
        '    declare_parameter("latency_compensation_debug_rate", 20.0);\n',
        '''    declare_parameter("latency_compensation_debug_rate", 20.0);
    // Experimental fork only: gate the target metric inside the same control
    // loop and with the same q_ctrl/qd_ctrl used by every other RMP.
    declare_parameter("internal_contact_target_gate_enabled", false);
    declare_parameter(
      "internal_contact_target_gate_topic", std::string("/admittance_rmp/activation"));
    declare_parameter("internal_contact_target_metric_floor", 0.01);
    declare_parameter("internal_contact_target_activation_timeout_sec", 0.20);
    declare_parameter(
      "internal_contact_target_key", std::string("target_gated"));
    declare_parameter("internal_admittance_enabled", false);
    declare_parameter(
      "internal_admittance_contact_point_topic",
      std::string("/admittance_rmp/contact_point"));
    declare_parameter(
      "internal_admittance_wrench_topic", std::string("/admittance_rmp/wrench"));
    declare_parameter(
      "internal_admittance_intent_topic",
      std::string("/worker_intent/compliance_probability"));
    declare_parameter("internal_admittance_key", std::string("admittance"));
    declare_parameter("internal_admittance_virtual_mass", std::vector<double>{3.0, 3.0, 3.0});
    declare_parameter("internal_admittance_damping", std::vector<double>{25.0, 25.0, 25.0});
    declare_parameter("internal_admittance_metric", std::vector<double>{300.0, 300.0, 300.0});
    // v2 goal 1-2: raise the admittance metric along the arm-plane normal
    // (moved by shoulder_pan only) when the pan lever is short.
    declare_parameter("internal_admittance_base_lever_compensation_enabled", false);
    declare_parameter("internal_admittance_base_lever_reference_m", 0.28);
    declare_parameter("internal_admittance_base_lever_max_factor", 20.0);
    // v2 "GM" (2026-11-07): RMP-consistent contact arbitration.  Defaults
    // reproduce the previous behaviour exactly.
    //  - unreachable lever: weight along the arm-plane normal -> 0 as lever -> 0
    //  - alignment gate: false removes the hard alignment rejection
    //  - authority source "force": s = smoothstep(|F|; low, high), no time ramp
    //  - target mode "push_gradient": targets pulled back through
    //    P = I - c g g^T, g = normalize(Jc^T F_hat), instead of x (1 - a)
    declare_parameter("internal_admittance_base_lever_unreachable_m", 0.0);
    declare_parameter("internal_admittance_alignment_gate_enabled", true);
    declare_parameter("internal_admittance_authority_source", std::string("time_ramp"));
    declare_parameter("internal_admittance_force_authority_low_n", 2.0);
    declare_parameter("internal_admittance_force_authority_high_n", 6.0);
    // 2026-11-07 (run 10, return wobble): with the force authority, the
    // activation may fall no faster than 1 / release_time per second, so the
    // targets come back over ~release_time after the hand lets go instead of
    // within the ~0.1 s it takes the force to vanish.  Engaging stays
    // instant.  0 = follow the force exactly (GM rev 2).
    declare_parameter("internal_admittance_release_time_sec", 0.0);
    // 2026-11-07 diagnostics only: log the average time of each v2 part of
    // compute_acceleration every 2 s (default off; no effect on control).
    declare_parameter("internal_profile_log_enabled", false);
    // v2 goal 2 (2026-11-07): "evidence" replaces the binary contact latch by a
    // continuous intent lambda_eff = max(lambda, m) where the memory m is held
    // by contact force and fades without it (contact_intent.hpp); compliance
    // is gated by smoothstep on lambda_eff instead of p >= enter_probability,
    // and the avoidance weight is 1 - lambda_eff.  "latch" = goal 1 behaviour.
    declare_parameter("internal_intent_mode", std::string("latch"));
    declare_parameter("internal_intent_hold_force_low_n", 1.0);
    declare_parameter("internal_intent_hold_force_high_n", 3.0);
    declare_parameter("internal_intent_release_tau_sec", 0.5);
    declare_parameter("internal_intent_compliance_low", 0.5);
    declare_parameter("internal_intent_compliance_high", 0.8);
    // "geometric": target scale = floor^a (intermediate activation gives an
    // intermediate competition); "linear" = floor + (1 - a)(1 - floor).
    declare_parameter("internal_contact_target_scale_mode", std::string("linear"));
    declare_parameter("internal_contact_target_mode", std::string("isotropic"));
    declare_parameter("internal_admittance_enter_force_n", 4.0);
    declare_parameter("internal_admittance_release_force_n", 2.0);
    declare_parameter("internal_admittance_enter_probability", 0.60);
    declare_parameter("internal_admittance_release_probability", 0.35);
    declare_parameter("internal_admittance_transition_sec", 0.30);
    declare_parameter("internal_admittance_wrench_timeout_sec", 0.25);
    declare_parameter("internal_admittance_intent_timeout_sec", 0.50);
    declare_parameter("internal_admittance_max_force_n", 30.0);
    declare_parameter("internal_admittance_max_task_accel_m_s2", 6.0);
    declare_parameter(
      "internal_admittance_max_joint_accel_rad_s2",
      std::vector<double>{2.5, 1.5, 1.5, 1.5, 1.5, 1.5});
    declare_parameter("internal_admittance_singular_damping_ratio", 0.05);
    declare_parameter("internal_admittance_minimum_alignment_cosine", 0.50);
    declare_parameter("internal_admittance_eigenvalue_floor", 1e-8);
    declare_parameter("internal_admittance_diagnostics_enabled", true);
    declare_parameter("internal_admittance_diagnostics_rate_hz", 20.0);
    // v2: contact latch and intent-gated proximity avoidance.  Both default
    // to off, so the Stage 2 behaviour is unchanged unless a yaml enables them.
    declare_parameter("internal_admittance_latch_enabled", false);
    declare_parameter("internal_avoidance_gate_enabled", false);
    declare_parameter("internal_avoidance_key", std::string("collision_gated"));
    declare_parameter("internal_avoidance_transition_sec", 0.30);
    declare_parameter("internal_avoidance_weight_floor", 0.0);
    // 2026-11-07: while the gated proximity avoidance is active (w > 0.5 and a
    // proximity obstacle is present) the command guard velocity (and with it
    // the per-cycle step and following-error limits) is raised to this value,
    // slewed at max_joint_accel; everything else keeps the normal guard.
    // 0 = off (normal guard always).  Must be within [normal guard, 0.4].
    declare_parameter("internal_avoidance_guard_velocity_rad_s", 0.0);
    // v2 stage 5: move the SO(3) orientation target into a root external leaf
    // scaled by the same contact gate as the position target (default off).
    declare_parameter("internal_orientation_gate_enabled", false);
    declare_parameter("internal_orientation_key", std::string("orientation_gated"));
    declare_parameter("internal_orientation_node", std::string("tcp_orientation_so3"));
''',
        "gate parameter declarations",
    )
    source = replace_once(
        source,
        '''    target_metric_params_ = solver_config.target;
    collision_metric_params_ = solver_config.collision;
    configure_external_rmp_inputs(solver_config.graph_nodes);
    solver_ = build_solver(solver_config);
''',
        '''    target_metric_params_ = solver_config.target;
    collision_metric_params_ = solver_config.collision;
    configure_external_rmp_inputs(solver_config.graph_nodes);
    internal_contact_target_gate_enabled_ =
      get_parameter("internal_contact_target_gate_enabled").as_bool();
    internal_contact_target_metric_floor_ = std::clamp(
      get_parameter("internal_contact_target_metric_floor").as_double(), 0.0, 1.0);
    internal_contact_target_activation_timeout_sec_ = std::max(
      get_parameter("internal_contact_target_activation_timeout_sec").as_double(), 0.01);
    internal_contact_target_key_ = get_parameter("internal_contact_target_key").as_string();
    if (internal_contact_target_gate_enabled_) {
      const auto package_share = ament_index_cpp::get_package_share_directory(
        "rb10_rmpflow_rviz");
      internal_contact_target_model_ = std::make_unique<PinocchioModel>(
        package_share + "/urdf/ur10.urdf");
      internal_contact_target_activation_sub_ =
        create_subscription<std_msgs::msg::Float64>(
        get_parameter("internal_contact_target_gate_topic").as_string(),
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64::SharedPtr message) {
          if (!std::isfinite(message->data)) {
            return;
          }
          internal_contact_target_activation_.store(
            std::clamp(message->data, 0.0, 1.0));
          const auto stamp = std::chrono::steady_clock::now().time_since_epoch();
          internal_contact_target_activation_stamp_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count());
        });
      internal_contact_target_scale_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/admittance_rmp/internal_target_metric_scale", rclcpp::QoS(10));
      internal_contact_target_diagnostic_timer_ = create_wall_timer(
        std::chrono::milliseconds(50), [this]() {
          std_msgs::msg::Float64 message;
          message.data = internal_contact_target_metric_scale();
          internal_contact_target_scale_pub_->publish(message);
        });
      RCLCPP_WARN(
        get_logger(),
        "EXPERIMENTAL INTERNAL target gate enabled (floor=%.4f, key=%s). "
        "Target and root solve share the same q_ctrl/qd_ctrl snapshot.",
        internal_contact_target_metric_floor_, internal_contact_target_key_.c_str());
    }
    internal_admittance_enabled_ = get_parameter("internal_admittance_enabled").as_bool();
    if (internal_admittance_enabled_) {
      const auto vector3 = [this](const char * name) {
          const auto values = get_parameter(name).as_double_array();
          if (values.size() != 3U) {
            throw std::invalid_argument(std::string(name) + " must have 3 values");
          }
          return Eigen::Vector3d(values[0], values[1], values[2]);
        };
      const auto vector6 = [this](const char * name) {
          const auto values = get_parameter(name).as_double_array();
          if (values.size() != 6U) {
            throw std::invalid_argument(std::string(name) + " must have 6 values");
          }
          JointVector result;
          for (int index = 0; index < 6; ++index) {
            result[index] = values[static_cast<std::size_t>(index)];
          }
          return result;
        };
      internal_admittance_virtual_mass_ = vector3("internal_admittance_virtual_mass");
      internal_admittance_damping_ = vector3("internal_admittance_damping");
      internal_admittance_metric_ = vector3("internal_admittance_metric");
      internal_admittance_base_lever_enabled_ =
        get_parameter("internal_admittance_base_lever_compensation_enabled").as_bool();
      internal_admittance_base_lever_reference_ =
        get_parameter("internal_admittance_base_lever_reference_m").as_double();
      internal_admittance_base_lever_max_factor_ =
        get_parameter("internal_admittance_base_lever_max_factor").as_double();
      internal_admittance_base_lever_unreachable_ =
        get_parameter("internal_admittance_base_lever_unreachable_m").as_double();
      internal_admittance_alignment_gate_enabled_ =
        get_parameter("internal_admittance_alignment_gate_enabled").as_bool();
      {
        const auto source = get_parameter("internal_admittance_authority_source").as_string();
        if (source != "time_ramp" && source != "force") {
          throw std::invalid_argument(
                  "internal_admittance_authority_source must be time_ramp or force");
        }
        internal_admittance_force_authority_ = source == "force";
        const auto mode = get_parameter("internal_contact_target_mode").as_string();
        if (mode != "isotropic" && mode != "push_gradient") {
          throw std::invalid_argument(
                  "internal_contact_target_mode must be isotropic or push_gradient");
        }
        internal_contact_target_push_gradient_ = mode == "push_gradient";
      }
      internal_admittance_force_authority_low_ =
        get_parameter("internal_admittance_force_authority_low_n").as_double();
      internal_admittance_force_authority_high_ =
        get_parameter("internal_admittance_force_authority_high_n").as_double();
      internal_admittance_release_time_sec_ =
        get_parameter("internal_admittance_release_time_sec").as_double();
      internal_profile_log_enabled_ =
        get_parameter("internal_profile_log_enabled").as_bool();
      {
        const auto intent_mode = get_parameter("internal_intent_mode").as_string();
        if (intent_mode != "latch" && intent_mode != "evidence") {
          throw std::invalid_argument("internal_intent_mode must be latch or evidence");
        }
        internal_intent_evidence_mode_ = intent_mode == "evidence";
        internal_intent_evidence_params_.hold_force_low =
          get_parameter("internal_intent_hold_force_low_n").as_double();
        internal_intent_evidence_params_.hold_force_high =
          get_parameter("internal_intent_hold_force_high_n").as_double();
        internal_intent_evidence_params_.release_tau =
          get_parameter("internal_intent_release_tau_sec").as_double();
        internal_intent_evidence_params_.compliance_low =
          get_parameter("internal_intent_compliance_low").as_double();
        internal_intent_evidence_params_.compliance_high =
          get_parameter("internal_intent_compliance_high").as_double();
        internal_intent_evidence_params_.validate();
        const auto scale_mode = get_parameter("internal_contact_target_scale_mode").as_string();
        if (scale_mode != "linear" && scale_mode != "geometric") {
          throw std::invalid_argument(
                  "internal_contact_target_scale_mode must be linear or geometric");
        }
        internal_contact_target_geometric_ = scale_mode == "geometric";
      }
      internal_admittance_max_joint_accel_ =
        vector6("internal_admittance_max_joint_accel_rad_s2");
      internal_admittance_key_ = get_parameter("internal_admittance_key").as_string();
      internal_admittance_enter_force_n_ =
        get_parameter("internal_admittance_enter_force_n").as_double();
      internal_admittance_release_force_n_ =
        get_parameter("internal_admittance_release_force_n").as_double();
      internal_admittance_enter_probability_ =
        get_parameter("internal_admittance_enter_probability").as_double();
      internal_admittance_release_probability_ =
        get_parameter("internal_admittance_release_probability").as_double();
      internal_admittance_transition_sec_ =
        get_parameter("internal_admittance_transition_sec").as_double();
      internal_admittance_wrench_timeout_sec_ =
        get_parameter("internal_admittance_wrench_timeout_sec").as_double();
      internal_admittance_intent_timeout_sec_ =
        get_parameter("internal_admittance_intent_timeout_sec").as_double();
      internal_admittance_max_force_n_ =
        get_parameter("internal_admittance_max_force_n").as_double();
      internal_admittance_max_task_accel_ =
        get_parameter("internal_admittance_max_task_accel_m_s2").as_double();
      internal_admittance_singular_damping_ratio_ =
        get_parameter("internal_admittance_singular_damping_ratio").as_double();
      internal_admittance_minimum_alignment_cosine_ =
        get_parameter("internal_admittance_minimum_alignment_cosine").as_double();
      internal_admittance_eigenvalue_floor_ =
        get_parameter("internal_admittance_eigenvalue_floor").as_double();
      internal_admittance_diagnostics_enabled_ =
        get_parameter("internal_admittance_diagnostics_enabled").as_bool();
      internal_admittance_diagnostics_period_ns_ = static_cast<std::int64_t>(1e9 / std::max(
        get_parameter("internal_admittance_diagnostics_rate_hz").as_double(), 1.0));
      if ((internal_admittance_virtual_mass_.array() <= 0.0).any() ||
        (internal_admittance_damping_.array() < 0.0).any() ||
        (internal_admittance_metric_.array() < 0.0).any() ||
        (internal_admittance_max_joint_accel_.array() <= 0.0).any() ||
        internal_admittance_transition_sec_ <= 0.0 ||
        internal_admittance_enter_force_n_ <= internal_admittance_release_force_n_ ||
        internal_admittance_enter_probability_ <= internal_admittance_release_probability_ ||
        internal_admittance_max_task_accel_ <= 0.0 ||
        internal_admittance_eigenvalue_floor_ <= 0.0 ||
        !(internal_admittance_base_lever_reference_ > 0.0) ||
        !(internal_admittance_base_lever_max_factor_ >= 1.0) ||
        !(internal_admittance_base_lever_unreachable_ >= 0.0) ||
        !(internal_admittance_force_authority_low_ >= 0.0) ||
        !(internal_admittance_force_authority_high_ > internal_admittance_force_authority_low_) ||
        !(internal_admittance_release_time_sec_ >= 0.0) ||
        (internal_intent_evidence_mode_ && !internal_admittance_force_authority_))
      {
        throw std::invalid_argument("invalid internal admittance parameters");
      }
      RCLCPP_WARN(
        get_logger(),
        "v2 contact arbitration: authority=%s (%.1f..%.1f N), target=%s, alignment gate=%s, "
        "unreachable lever=%.3f m, release time=%.2f s, intent=%s, target scale=%s",
        internal_admittance_force_authority_ ? "force" : "time_ramp",
        internal_admittance_force_authority_low_, internal_admittance_force_authority_high_,
        internal_contact_target_push_gradient_ ? "push_gradient" : "isotropic",
        internal_admittance_alignment_gate_enabled_ ? "on" : "off",
        internal_admittance_base_lever_unreachable_, internal_admittance_release_time_sec_,
        internal_intent_evidence_mode_ ? "evidence" : "latch",
        internal_contact_target_geometric_ ? "geometric" : "linear");
      internal_admittance_point_sub_ =
        create_subscription<geometry_msgs::msg::PointStamped>(
        get_parameter("internal_admittance_contact_point_topic").as_string(),
        rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr message) {
          std::size_t link = UR10Model::LINK_COUNT;
          for (std::size_t index = 0; index < UR10Model::LINK_COUNT; ++index) {
            if (message->header.frame_id == UR10Model::link_names[index]) {
              link = index;
              break;
            }
          }
          const Eigen::Vector3d point(message->point.x, message->point.y, message->point.z);
          if (link >= UR10Model::LINK_COUNT || !point.allFinite()) {
            return;
          }
          std::scoped_lock lock(internal_admittance_mutex_);
          internal_admittance_link_ = link;
          internal_admittance_point_ = point;
          internal_admittance_have_point_ = true;
        });
      internal_admittance_wrench_sub_ =
        create_subscription<geometry_msgs::msg::WrenchStamped>(
        get_parameter("internal_admittance_wrench_topic").as_string(),
        rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
          if (message->header.frame_id != "base_link") {
            return;
          }
          Eigen::Vector3d force(
            message->wrench.force.x, message->wrench.force.y, message->wrench.force.z);
          if (!force.allFinite()) {
            return;
          }
          const double norm = force.norm();
          if (norm > internal_admittance_max_force_n_) {
            force *= internal_admittance_max_force_n_ / norm;
          }
          std::scoped_lock lock(internal_admittance_mutex_);
          internal_admittance_force_ = force;
          internal_admittance_wrench_stamp_ns_ = steady_now_ns();
          internal_admittance_have_wrench_ = true;
        });
      internal_admittance_intent_sub_ = create_subscription<std_msgs::msg::Float64>(
        get_parameter("internal_admittance_intent_topic").as_string(),
        rclcpp::QoS(10),
        [this](const std_msgs::msg::Float64::SharedPtr message) {
          if (!std::isfinite(message->data)) {
            return;
          }
          std::scoped_lock lock(internal_admittance_mutex_);
          internal_admittance_probability_ = std::clamp(message->data, 0.0, 1.0);
          internal_admittance_intent_stamp_ns_ = steady_now_ns();
          internal_admittance_have_intent_ = true;
        });
      internal_admittance_activation_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/admittance_rmp/activation", rclcpp::QoS(10));
      internal_admittance_diagnostics_pub_ =
        create_publisher<std_msgs::msg::Float64MultiArray>(
        "/admittance_rmp/internal_diagnostics", rclcpp::QoS(10));
      RCLCPP_WARN(
        get_logger(),
        "EXPERIMENTAL INTERNAL admittance enabled (key=%s). Contact geometry, "
        "target arbitration, and root solve share one q_ctrl/qd_ctrl snapshot.",
        internal_admittance_key_.c_str());
    }
    internal_admittance_latch_enabled_ =
      get_parameter("internal_admittance_latch_enabled").as_bool();
    internal_avoidance_gate_enabled_ =
      get_parameter("internal_avoidance_gate_enabled").as_bool();
    if (internal_avoidance_gate_enabled_) {
      if (!internal_admittance_enabled_) {
        throw std::invalid_argument(
                "internal_avoidance_gate_enabled requires internal_admittance_enabled");
      }
      internal_avoidance_key_ = get_parameter("internal_avoidance_key").as_string();
      internal_avoidance_transition_sec_ =
        get_parameter("internal_avoidance_transition_sec").as_double();
      internal_avoidance_weight_floor_ = std::clamp(
        get_parameter("internal_avoidance_weight_floor").as_double(), 0.0, 1.0);
      if (internal_avoidance_transition_sec_ <= 0.0) {
        throw std::invalid_argument("internal_avoidance_transition_sec must be positive");
      }
      internal_avoidance_guard_velocity_ =
        get_parameter("internal_avoidance_guard_velocity_rad_s").as_double();
      if (internal_avoidance_guard_velocity_ != 0.0 &&
        !(internal_avoidance_guard_velocity_ >= command_guard_max_velocity_rad_s_ &&
        internal_avoidance_guard_velocity_ <= 0.4))
      {
        throw std::invalid_argument(
                "internal_avoidance_guard_velocity_rad_s must be 0 (off) or within "
                "[command guard velocity, 0.4]");
      }
      internal_guard_velocity_.store(command_guard_max_velocity_rad_s_);
      RCLCPP_WARN(
        get_logger(),
        "v2 avoidance guard: %s (normal %.3f rad/s, while avoiding %.3f rad/s)",
        internal_avoidance_guard_velocity_ > 0.0 ? "ON" : "off",
        command_guard_max_velocity_rad_s_,
        internal_avoidance_guard_velocity_ > 0.0 ?
        internal_avoidance_guard_velocity_ : command_guard_max_velocity_rad_s_);
      namespace gate = ur10_intent_admittance_rmp_v2::intent_avoidance;
      gate::require_root_external_leaf(solver_config, internal_avoidance_key_);
      internal_avoidance_solver_ = build_solver(gate::make_collision_only_config(solver_config));
      internal_avoidance_weight_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/admittance_rmp/avoidance_weight", rclcpp::QoS(10));
      internal_intent_effective_pub_ = create_publisher<std_msgs::msg::Float64>(
        "/admittance_rmp/intent_effective", rclcpp::QoS(10));
      // [w, p_eff, proximity obstacles seen by the gate, trace(M_c) before w,
      //  contact-gate scale applied to the position/orientation targets,
      //  base-lever factor and lever (m, -1 before the first contact)]
      internal_avoidance_diagnostics_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/admittance_rmp/avoidance_diagnostics", rclcpp::QoS(10));
      internal_avoidance_diagnostic_timer_ = create_wall_timer(
        std::chrono::milliseconds(20), [this]() {
          std_msgs::msg::Float64 weight;
          weight.data = internal_avoidance_weight_.load();
          internal_avoidance_weight_pub_->publish(weight);
          std_msgs::msg::Float64 intent;
          intent.data = internal_intent_effective_.load();
          internal_intent_effective_pub_->publish(intent);
          std_msgs::msg::Float64MultiArray diagnostics;
          diagnostics.data = {
            weight.data, intent.data, internal_avoidance_proximity_count_.load(),
            internal_avoidance_metric_trace_.load(), internal_contact_target_metric_scale(),
            internal_admittance_base_lever_factor_.load(), internal_admittance_base_lever_.load(),
            internal_push_projection_strength_.load(), internal_guard_velocity_.load(),
            internal_intent_compliance_gate_.load()};
          internal_avoidance_diagnostics_pub_->publish(diagnostics);
        });
      RCLCPP_WARN(
        get_logger(),
        "EXPERIMENTAL v2 intent-gated proximity avoidance enabled (key=%s, floor=%.3f, "
        "transition=%.2fs, latch=%s). Proximity obstacles are solved by a collision-only "
        "copy of the production solver and weighted by w = max(floor, 1 - p_eff).",
        internal_avoidance_key_.c_str(), internal_avoidance_weight_floor_,
        internal_avoidance_transition_sec_,
        internal_admittance_latch_enabled_ ? "on" : "off");
    }
    // The main solver may lose the SO(3) leaf (moved to a gated external leaf).
    EigenRmpConfig internal_main_solver_config = solver_config;
    internal_orientation_gate_enabled_ =
      get_parameter("internal_orientation_gate_enabled").as_bool();
    if (internal_orientation_gate_enabled_) {
      if (!internal_contact_target_gate_enabled_) {
        throw std::invalid_argument(
                "internal_orientation_gate_enabled requires internal_contact_target_gate_enabled "
                "(the orientation leaf uses the same contact gate as the position target)");
      }
      namespace gate = ur10_intent_admittance_rmp_v2::intent_avoidance;
      internal_orientation_key_ = get_parameter("internal_orientation_key").as_string();
      const auto orientation_node = get_parameter("internal_orientation_node").as_string();
      gate::require_root_external_leaf(solver_config, internal_orientation_key_);
      internal_orientation_solver_ =
        build_solver(gate::make_orientation_only_config(solver_config, orientation_node));
      internal_main_solver_config = gate::without_node(internal_main_solver_config, orientation_node);
      RCLCPP_WARN(
        get_logger(),
        "EXPERIMENTAL v2 contact-gated SO(3) orientation enabled (node=%s, key=%s, floor=%.5f). "
        "The orientation leaf is solved by an orientation-only copy of the production solver "
        "and scaled by the same contact gate as the position target.",
        orientation_node.c_str(), internal_orientation_key_.c_str(),
        internal_contact_target_metric_floor_);
    }
    solver_ = build_solver(internal_main_solver_config);
''',
        "gate initialization",
    )
    helper = r'''
  static std::int64_t steady_now_ns()
  {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(stamp).count();
  }

  static double internal_smoothstep(double value)
  {
    const double x = std::clamp(value, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
  }

  double internal_contact_target_activation() const
  {
    if (!internal_contact_target_gate_enabled_) {
      return 0.0;
    }
    if (internal_admittance_enabled_) {
      return std::clamp(internal_admittance_effective_activation_.load(), 0.0, 1.0);
    }
    const auto now_stamp = std::chrono::steady_clock::now().time_since_epoch();
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now_stamp).count();
    const auto sample_ns = internal_contact_target_activation_stamp_ns_.load();
    const double age_sec = static_cast<double>(now_ns - sample_ns) * 1e-9;
    if (sample_ns <= 0 || age_sec < 0.0 ||
      age_sec > internal_contact_target_activation_timeout_sec_)
    {
      // Fail toward the ordinary target, never toward an unrestrained robot.
      return 0.0;
    }
    return std::clamp(internal_contact_target_activation_.load(), 0.0, 1.0);
  }

  double internal_contact_target_metric_scale() const
  {
    const double activation = internal_contact_target_activation();
    return ur10_intent_admittance_rmp_v2::contact_target_scale(
      activation, internal_contact_target_metric_floor_, internal_contact_target_geometric_);
  }

  void publish_internal_admittance_activation(double value) const
  {
    internal_admittance_effective_activation_.store(std::clamp(value, 0.0, 1.0));
    if (internal_admittance_activation_pub_) {
      std_msgs::msg::Float64 message;
      message.data = internal_admittance_effective_activation_.load();
      internal_admittance_activation_pub_->publish(message);
    }
  }

  // Diagnostic array layout:
  // [status, point_valid, wrench_fresh, intent_fresh, force_N, probability,
  //  request_phase, requested_transition, effective_activation, alignment,
  //  requested_ax, requested_ay, requested_az,
  //  achieved_ax, achieved_ay, achieved_az, |qd_ctrl|, target_metric_scale]
  // status: 0=idle/invalid, 1=accepted, 2=alignment_rejected, 3=exception.
  void publish_internal_admittance_diagnostics(
    double status, bool point_valid, bool wrench_fresh, bool intent_fresh,
    double force_norm, double probability, double request_phase,
    double requested_transition, double alignment,
    const Eigen::Vector3d & requested_acceleration,
    const Eigen::Vector3d & achieved_acceleration,
    double qd_norm) const
  {
    if (!internal_admittance_diagnostics_enabled_ || !internal_admittance_diagnostics_pub_) {
      return;
    }
    const auto now_ns = steady_now_ns();
    auto previous = internal_admittance_last_diagnostics_ns_.load();
    if (now_ns - previous < internal_admittance_diagnostics_period_ns_ ||
      !internal_admittance_last_diagnostics_ns_.compare_exchange_strong(previous, now_ns))
    {
      return;
    }
    std_msgs::msg::Float64MultiArray message;
    message.data = {
      status, point_valid ? 1.0 : 0.0, wrench_fresh ? 1.0 : 0.0,
      intent_fresh ? 1.0 : 0.0, force_norm, probability, request_phase,
      requested_transition, internal_admittance_effective_activation_.load(), alignment,
      requested_acceleration.x(), requested_acceleration.y(), requested_acceleration.z(),
      achieved_acceleration.x(), achieved_acceleration.y(), achieved_acceleration.z(),
      qd_norm, internal_contact_target_metric_scale()};
    internal_admittance_diagnostics_pub_->publish(message);
  }

  std::optional<ExternalRmpFeature> make_internal_admittance(
    const RobotState & state) const
  {
    if (!internal_admittance_enabled_) {
      return std::nullopt;
    }
    // GM: the push projection is rebuilt every cycle; anything but an
    // accepted contact leaves the targets on their ordinary (isotropic) path.
    store_push_projection(ur10_intent_admittance_rmp_v2::PushProjection{});
    Eigen::Vector3d force = Eigen::Vector3d::Zero();
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    std::size_t link = UR10Model::LINK_COUNT;
    bool requested_active_now = false;
    double probability = 0.0;
    bool inputs_valid = false;
    bool point_valid = false;
    bool wrench_fresh = false;
    bool intent_fresh = false;
    // Weight applied to the contact pair.  Equals `probability` unless the v2
    // latch is holding the contact, in which case it is 1.
    double authority = 0.0;
    {
      std::scoped_lock lock(internal_admittance_mutex_);
      const auto now_ns = steady_now_ns();
      point_valid = internal_admittance_have_point_ &&
        internal_admittance_link_ < UR10Model::LINK_COUNT;
      wrench_fresh = internal_admittance_have_wrench_ &&
        static_cast<double>(now_ns - internal_admittance_wrench_stamp_ns_) * 1e-9 <=
        internal_admittance_wrench_timeout_sec_;
      intent_fresh = internal_admittance_have_intent_ &&
        static_cast<double>(now_ns - internal_admittance_intent_stamp_ns_) * 1e-9 <=
        internal_admittance_intent_timeout_sec_;
      // v2 latch: intent is required to ENTER, but once engaged only the
      // contact itself (point, wrench freshness, force level) can release it.
      // With the latch off every expression below reduces to the original.
      // evidence mode keeps the latch's input rule (fresh intent is not needed
      // to keep a contact valid; the intent memory decides instead).
      const bool latch = internal_admittance_latch_enabled_ || internal_intent_evidence_mode_;
      inputs_valid = point_valid && wrench_fresh && (latch || intent_fresh);
      if (inputs_valid) {
        force = internal_admittance_force_;
        point = internal_admittance_point_;
        link = internal_admittance_link_;
      }
      if (intent_fresh && (inputs_valid || latch)) {
        probability = internal_admittance_probability_;
      }
      const double force_norm = inputs_valid ? force.norm() : 0.0;
      if (!internal_admittance_requested_active_ && inputs_valid && intent_fresh &&
        force_norm >= internal_admittance_enter_force_n_ &&
        probability >= internal_admittance_enter_probability_)
      {
        internal_admittance_requested_active_ = true;
      } else if (internal_admittance_requested_active_ &&
        (!inputs_valid || force_norm <= internal_admittance_release_force_n_ ||
        (!latch && probability <= internal_admittance_release_probability_)))
      {
        internal_admittance_requested_active_ = false;
      }
      requested_active_now = internal_admittance_requested_active_;
      const double requested = internal_admittance_requested_active_ ? 1.0 : 0.0;
      const double max_delta =
        (1.0 / std::max(control_rate_hz_, 1e-9)) / internal_admittance_transition_sec_;
      internal_admittance_request_phase_ += std::clamp(
        requested - internal_admittance_request_phase_, -max_delta, max_delta);
      internal_admittance_request_phase_ = std::clamp(
        internal_admittance_request_phase_, 0.0, 1.0);
      // p_eff: 1 while the latch holds the contact (engaged or ramping out),
      // otherwise the fresh vision intent, otherwise 0.  Computed every cycle,
      // before contact, so the avoidance weight can relax ahead of the touch.
      const bool latch_hold = latch &&
        (internal_admittance_requested_active_ || internal_admittance_request_phase_ > 0.0);
      if (internal_intent_evidence_mode_) {
        // goal 2: continuous intent.  The boolean request above is computed
        // but not used; lambda_eff (held by contact force) replaces p_eff and
        // the compliance permission becomes a smooth gate on it.
        const double lambda_in =
          intent_fresh ? std::clamp(internal_admittance_probability_, 0.0, 1.0) : 0.0;
        const double contact_force =
          (point_valid && wrench_fresh) ? internal_admittance_force_.norm() : 0.0;
        const double lambda_eff = ur10_intent_admittance_rmp_v2::update_intent_evidence(
          internal_intent_evidence_state_, lambda_in, contact_force,
          1.0 / std::max(control_rate_hz_, 1e-9), internal_intent_evidence_params_);
        internal_intent_effective_.store(lambda_eff);
        internal_intent_compliance_gate_.store(
          ur10_intent_admittance_rmp_v2::intent_compliance_gate(
            lambda_eff, internal_intent_evidence_params_));
        authority = 1.0;
      } else {
        internal_intent_effective_.store(
          latch_hold ? 1.0 :
          (intent_fresh ? std::clamp(internal_admittance_probability_, 0.0, 1.0) : 0.0));
        authority = latch_hold ? 1.0 : probability;
      }
    }

    // GM: with the force authority source the transition is a smooth function
    // of the measured contact force (no time ramp); the latch state only says
    // whether compliance is permitted at all.
    const double transition_shape = internal_intent_evidence_mode_ ?
      (inputs_valid ?
      ur10_intent_admittance_rmp_v2::force_authority(
        force.norm(), internal_admittance_force_authority_low_,
        internal_admittance_force_authority_high_) *
      internal_intent_compliance_gate_.load() : 0.0) :
      internal_admittance_force_authority_ ?
      ((inputs_valid && requested_active_now) ?
      ur10_intent_admittance_rmp_v2::force_authority(
        force.norm(), internal_admittance_force_authority_low_,
        internal_admittance_force_authority_high_) : 0.0) :
      internal_smoothstep(internal_admittance_request_phase_);
    const double requested_transition =
      inputs_valid ? transition_shape * authority : 0.0;
    const double phase_step =
      (1.0 / std::max(control_rate_hz_, 1e-9)) / internal_admittance_transition_sec_;
    const auto slew_effective = [&](double target) {
        const double current = internal_admittance_effective_activation_.load();
        const double release_step = internal_admittance_release_time_sec_ > 0.0 ?
          (1.0 / std::max(control_rate_hz_, 1e-9)) / internal_admittance_release_time_sec_ :
          1.0;
        const double next = internal_admittance_force_authority_ ?
          std::max(std::clamp(target, 0.0, 1.0), current - release_step) :
          current + std::clamp(target - current, -phase_step, phase_step);
        publish_internal_admittance_activation(next);
        return next;
      };
    if (!inputs_valid || requested_transition <= 1e-6) {
      slew_effective(0.0);
      internal_admittance_base_lever_factor_.store(1.0);
      publish_internal_admittance_diagnostics(
        0.0, point_valid, wrench_fresh, intent_fresh,
        inputs_valid ? force.norm() : 0.0, probability,
        internal_admittance_request_phase_, requested_transition, 0.0,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), state.qd.norm());
      RCLCPP_INFO_THROTTLE(
        get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 1000,
        "INTERNAL ADMITTANCE idle: point=%d wrench_fresh=%d intent_fresh=%d "
        "force=%.2fN p=%.2f phase=%.3f",
        point_valid, wrench_fresh, intent_fresh,
        inputs_valid ? force.norm() : 0.0, probability,
        internal_admittance_request_phase_);
      return std::nullopt;
    }

    try {
      using namespace ur10_intent_admittance_rmp_v2;
      const auto geometry = contact_geometry(state.q, state.qd, link, point);
      // v2 goal 1-2: full task weight (diagonal unless the base-lever
      // compensation is on and the pan lever is short).
      BaseLeverWeight lever_weight;
      lever_weight.weight = internal_admittance_metric_.asDiagonal();
      if (internal_admittance_base_lever_enabled_) {
        lever_weight = base_lever_weight(
          geometry.jacobian, internal_admittance_metric_,
          internal_admittance_base_lever_reference_,
          internal_admittance_base_lever_max_factor_,
          internal_admittance_base_lever_unreachable_);
      }
      internal_admittance_base_lever_factor_.store(lever_weight.factor);
      internal_admittance_base_lever_.store(lever_weight.lever);
      const Eigen::Vector3d raw_task_acceleration =
        force.cwiseQuotient(internal_admittance_virtual_mass_);
      const Eigen::Vector3d limited_task_acceleration = direction_preserving_task_limit(
        raw_task_acceleration, internal_admittance_max_task_accel_);
      const Eigen::Vector3d limited_force =
        limited_task_acceleration.cwiseProduct(internal_admittance_virtual_mass_);

      // Validate reachability before weakening the target.  The final pair is
      // then recomputed with the same smoothly slewed authority used by target
      // arbitration, so the two policies cannot switch on different cycles.
      const auto requested_pair = contact_natural_pair_weighted(
        geometry, limited_force, internal_admittance_virtual_mass_,
        internal_admittance_damping_, lever_weight.weight,
        requested_transition);
      const JointVector requested_raw = resolve_pair_damped(
        requested_pair, internal_admittance_singular_damping_ratio_,
        internal_admittance_eigenvalue_floor_);
      const JointVector requested_limited = direction_preserving_joint_limit(
        requested_raw, internal_admittance_max_joint_accel_);
      const Eigen::Vector3d achieved_acceleration =
        geometry.jacobian * requested_limited;
      const double alignment = task_alignment_cosine(
        limited_task_acceleration, achieved_acceleration);
      if (internal_admittance_alignment_gate_enabled_ &&
        alignment < internal_admittance_minimum_alignment_cosine_)
      {
        slew_effective(0.0);
        publish_internal_admittance_diagnostics(
          2.0, point_valid, wrench_fresh, intent_fresh, force.norm(), probability,
          internal_admittance_request_phase_, requested_transition, alignment,
          limited_task_acceleration, achieved_acceleration, state.qd.norm());
        RCLCPP_WARN_THROTTLE(
          get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 500,
          "INTERNAL ADMITTANCE REJECT alignment=%.3f < %.3f; force=%.2fN "
          "phase=%.3f requested=[%.3f %.3f %.3f] achieved=[%.3f %.3f %.3f] "
          "|qd_ctrl|=%.3f",
          alignment, internal_admittance_minimum_alignment_cosine_, force.norm(),
          internal_admittance_request_phase_, limited_task_acceleration.x(),
          limited_task_acceleration.y(), limited_task_acceleration.z(),
          achieved_acceleration.x(), achieved_acceleration.y(),
          achieved_acceleration.z(), state.qd.norm());
        return std::nullopt;
      }

      const double effective = slew_effective(requested_transition);
      if (effective <= 1e-6) {
        return std::nullopt;
      }
      if (internal_contact_target_push_gradient_) {
        store_push_projection(
          ur10_intent_admittance_rmp_v2::push_gradient_projection(
            geometry.jacobian, limited_force,
            std::clamp(effective, 0.0, 1.0)));
      }
      const auto pair = contact_natural_pair_weighted(
        geometry, limited_force, internal_admittance_virtual_mass_,
        internal_admittance_damping_, lever_weight.weight, effective);
      Eigen::SelfAdjointEigenSolver<Matrix6> eigensolver(pair.metric);
      if (eigensolver.info() != Eigen::Success) {
        throw std::runtime_error("internal admittance metric eigendecomposition failed");
      }
      const auto eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0);
      const Matrix6 basis = eigensolver.eigenvectors();
      const Matrix6 metric_sqrt =
        basis * eigenvalues.cwiseSqrt().asDiagonal() * basis.transpose();
      const JointVector raw_acceleration = resolve_pair_damped(
        pair, internal_admittance_singular_damping_ratio_,
        internal_admittance_eigenvalue_floor_);
      const JointVector acceleration = direction_preserving_joint_limit(
        raw_acceleration, internal_admittance_max_joint_accel_);
      if (!metric_sqrt.allFinite() || !acceleration.allFinite()) {
        throw std::runtime_error("non-finite internal admittance natural pair");
      }
      publish_internal_admittance_diagnostics(
        1.0, point_valid, wrench_fresh, intent_fresh, force.norm(), probability,
        internal_admittance_request_phase_, requested_transition, alignment,
        limited_task_acceleration, achieved_acceleration, state.qd.norm());
      RCLCPP_INFO_THROTTLE(
        get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 1000,
        "INTERNAL ADMITTANCE ACCEPT activation=%.3f target_scale=%.3f "
        "alignment=%.3f force=%.2fN base_lever=%.3fm base_factor=%.2f push_proj=%.2f",
        effective, internal_contact_target_metric_scale(), alignment, force.norm(),
        lever_weight.lever, lever_weight.factor, internal_push_projection_strength_.load());
      return ExternalRmpFeature{metric_sqrt, acceleration};
    } catch (const std::exception & error) {
      slew_effective(0.0);
      publish_internal_admittance_diagnostics(
        3.0, point_valid, wrench_fresh, intent_fresh, force.norm(), probability,
        internal_admittance_request_phase_, requested_transition, 0.0,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), state.qd.norm());
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 1000,
        "INTERNAL ADMITTANCE EXCEPTION: %s", error.what());
      return std::nullopt;
    }
  }

  // v2: avoidance weight w = max(floor, 1 - p_eff), slewed so the collision
  // metric never jumps between control cycles.
  double update_internal_avoidance_weight() const
  {
    const double target = std::max(
      internal_avoidance_weight_floor_, 1.0 - internal_intent_effective_.load());
    const double step =
      (1.0 / std::max(control_rate_hz_, 1e-9)) / internal_avoidance_transition_sec_;
    const double next = std::clamp(
      ur10_intent_admittance_rmp_v2::intent_avoidance::slew_toward(
        internal_avoidance_weight_.load(), target, step),
      0.0, 1.0);
    internal_avoidance_weight_.store(next);
    return next;
  }

  // v2: split the obstacles, solve the proximity part with the collision-only
  // production solver, and insert it as a weighted root external leaf.
  // Returns false if the gate could not be evaluated; the caller then passes
  // the unmodified obstacle list to the main solver (production avoidance).
  // v2: avoidance-only guard velocity, slewed at the root acceleration limit
  // so the guard never changes faster than the commanded motion may.
  // avoiding_weight: the avoidance weight w while a proximity obstacle is
  // active, 0 otherwise.  The raise follows w smoothly (full above w = 0.7).
  void update_internal_avoidance_guard(double avoiding_weight) const
  {
    if (!(internal_avoidance_guard_velocity_ > 0.0)) {
      return;
    }
    const double base = command_guard_max_velocity_rad_s_;
    const double share = ur10_intent_admittance_rmp_v2::intent_smoothstep(
      (avoiding_weight - 0.3) / 0.4);
    const double target = base + share * (internal_avoidance_guard_velocity_ - base);
    const double step = max_joint_accel_ / std::max(control_rate_hz_, 1e-9);
    const double current = internal_guard_velocity_.load();
    internal_guard_velocity_.store(std::clamp(
      current + std::clamp(target - current, -step, step), base,
      internal_avoidance_guard_velocity_));
  }

  bool apply_internal_avoidance_gate(
    const RobotState & state,
    const std::vector<ObstacleSphere> & obstacles,
    std::vector<ObstacleSphere> & main_obstacles,
    std::unordered_map<std::string, ExternalRmpFeature> & external_rmps) const
  {
    namespace gate = ur10_intent_admittance_rmp_v2::intent_avoidance;
    const double weight = update_internal_avoidance_weight();
    std::unordered_set<std::int64_t> proximity_source_ids;
    {
      std::scoped_lock lock(internal_proximity_ids_mutex_);
      proximity_source_ids = internal_proximity_source_ids_;
    }
    try {
      auto split = gate::split_obstacles(obstacles, proximity_source_ids);
      double metric_trace = 0.0;
      if (!split.proximity.empty()) {
        const std::unordered_map<std::string, Eigen::Vector3d> no_vector_targets;
        const auto contribution = internal_avoidance_solver_->solve(
          state.q, state.qd, no_vector_targets, split.proximity);
        metric_trace = contribution.metric.trace();
        if (const auto feature = gate::weighted_root_feature(
            contribution.metric, contribution.force, weight))
        {
          external_rmps.insert_or_assign(internal_avoidance_key_, *feature);
        }
      }
      internal_avoidance_proximity_count_.store(static_cast<double>(split.proximity.size()));
      internal_avoidance_metric_trace_.store(metric_trace);
      update_internal_avoidance_guard(
        (!split.proximity.empty() && metric_trace > 0.0) ? weight : 0.0);
      main_obstacles = std::move(split.others);
      return true;
    } catch (const std::exception & error) {
      update_internal_avoidance_guard(0.0);
      internal_avoidance_weight_.store(1.0);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 1000,
        "INTERNAL AVOIDANCE GATE failed; full production avoidance kept: %s",
        error.what());
      return false;
    }
  }

  // v2 stage 5: SO(3) orientation leaf from the orientation-only production
  // solver, scaled by the position target's contact gate.  On failure the
  // exception propagates so the controller's own solve-failure handling
  // (command hold) applies instead of silently dropping the orientation leaf.
  void apply_internal_orientation_gate(
    const RobotState & state,
    const std::unordered_map<std::string, Eigen::Vector3d> & vector_targets,
    std::unordered_map<std::string, ExternalRmpFeature> & external_rmps) const
  {
    namespace gate = ur10_intent_admittance_rmp_v2::intent_avoidance;
    const auto push = load_push_projection();
    // GM (rev 2): the isotropic contact scale (floor + (1 - a)(1 - floor)) always
    // applies; the projection additionally removes the push direction, which is
    // what made a non-zero floor resist pushes before.
    const double scale = internal_contact_target_metric_scale();
    internal_orientation_scale_.store(scale);
    const std::vector<ObstacleSphere> no_obstacles{ObstacleSphere{}};
    const auto contribution = internal_orientation_solver_->solve(
      state.q, state.qd, vector_targets, no_obstacles);
    ur10_intent_admittance_rmp_v2::Matrix6 metric = contribution.metric;
    ur10_intent_admittance_rmp_v2::JointVector force = contribution.force;
    if (push.valid) {
      // GM: keep the orientation authority, remove only the push direction.
      ur10_intent_admittance_rmp_v2::project_joint_pair(push.projection, metric, force);
    }
    if (const auto feature = gate::weighted_root_feature(metric, force, scale)) {
      external_rmps.insert_or_assign(internal_orientation_key_, *feature);
    }
  }

  void store_push_projection(const ur10_intent_admittance_rmp_v2::PushProjection & push) const
  {
    std::scoped_lock lock(internal_push_projection_mutex_);
    internal_push_projection_ = push;
  }

  ur10_intent_admittance_rmp_v2::PushProjection load_push_projection() const
  {
    if (!internal_contact_target_push_gradient_) {
      return ur10_intent_admittance_rmp_v2::PushProjection{};
    }
    std::scoped_lock lock(internal_push_projection_mutex_);
    return internal_push_projection_;
  }

  ExternalRmpFeature make_internal_contact_gated_target(
    const RobotState & state,
    const Eigen::Vector3d & goal) const
  {
    if (!internal_contact_target_model_) {
      throw std::runtime_error("internal target gate model is not initialized");
    }
    const auto context = internal_contact_target_model_->forward_context(state.q, state.qd);
    const Eigen::Vector3d delta = goal - context.tcp_position;
    const double delta_norm = delta.norm();
    const double soft_delta_norm =
      std::max(delta_norm, target_metric_params_.accel_norm_eps / 10.0);
    const Eigen::Vector3d delta_hat = delta / soft_delta_norm;
    const Eigen::Vector3d task_acceleration =
      target_metric_params_.accel_p_gain * delta /
      (delta_norm + target_metric_params_.accel_norm_eps) -
      target_metric_params_.accel_d_gain * context.tcp_velocity;

    const Eigen::Matrix3d shape = delta_hat * delta_hat.transpose();
    const double scaled_dist = delta_norm / target_metric_params_.metric_alpha_length_scale;
    const double alpha =
      (1.0 - target_metric_params_.min_metric_alpha) *
      std::exp(-0.5 * scaled_dist * scaled_dist) +
      target_metric_params_.min_metric_alpha;
    Eigen::Matrix3d task_metric =
      alpha * target_metric_params_.max_metric_scalar * Eigen::Matrix3d::Identity() +
      (1.0 - alpha) * target_metric_params_.min_metric_scalar * shape;
    const double boost_scaled_dist =
      delta_norm / target_metric_params_.proximity_metric_boost_length_scale;
    const double boost_alpha = std::exp(-0.5 * boost_scaled_dist * boost_scaled_dist);
    task_metric *=
      boost_alpha * target_metric_params_.proximity_metric_boost_scalar +
      (1.0 - boost_alpha);
    // GM (rev 2): isotropic contact scale as before, and with a valid push
    // projection the target is additionally pulled back through P.
    const auto push = load_push_projection();
    task_metric *= internal_contact_target_metric_scale();

    Eigen::Matrix<double, 6, 6> joint_metric =
      context.tcp_jacobian.transpose() * task_metric * context.tcp_jacobian;
    JointVector joint_force =
      context.tcp_jacobian.transpose() * task_metric *
      (task_acceleration - context.tcp_curvature);
    if (push.valid) {
      ur10_intent_admittance_rmp_v2::project_joint_pair(push.projection, joint_metric, joint_force);
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(joint_metric);
    if (eigensolver.info() != Eigen::Success) {
      throw std::runtime_error("internal target metric eigendecomposition failed");
    }
    const auto eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0);
    const auto basis = eigensolver.eigenvectors();
    const double threshold = std::max(1e-10, eigenvalues.maxCoeff() * 1e-10);
    JointVector joint_acceleration = JointVector::Zero();
    for (int index = 0; index < 6; ++index) {
      if (eigenvalues[index] > threshold) {
        const auto direction = basis.col(index);
        joint_acceleration +=
          direction * direction.dot(joint_force) / eigenvalues[index];
      }
    }
    const Eigen::Matrix<double, 6, 6> metric_sqrt =
      basis * eigenvalues.cwiseSqrt().asDiagonal() * basis.transpose();
    if (!metric_sqrt.allFinite() || !joint_acceleration.allFinite()) {
      throw std::runtime_error("non-finite internal target natural pair");
    }
    return ExternalRmpFeature{metric_sqrt, joint_acceleration};
  }

'''
    source = replace_once(
        source,
        '  JointVector compute_acceleration(\n',
        helper + '  JointVector compute_acceleration(\n',
        "internal target helper",
    )
    source = replace_once(
        source,
        '''    const auto solution = solver_->solve(
      state.q,
''',
        '''    const auto profile_t0 = std::chrono::steady_clock::now();
    bool profile_admittance_active = false;
    if (internal_admittance_enabled_) {
      if (const auto admittance = make_internal_admittance(state)) {
        profile_admittance_active = true;
        external_rmps.insert_or_assign(internal_admittance_key_, *admittance);
      }
      {
        const auto push = load_push_projection();
        internal_push_projection_strength_.store(push.valid ? push.strength : 0.0);
      }
    }
    const auto profile_t1 = std::chrono::steady_clock::now();
    if (internal_contact_target_gate_enabled_) {
      external_rmps.insert_or_assign(
        internal_contact_target_key_,
        make_internal_contact_gated_target(state, goal));
    }
    const auto profile_t2 = std::chrono::steady_clock::now();
    if (internal_orientation_gate_enabled_) {
      apply_internal_orientation_gate(state, vector_targets, external_rmps);
    }
    const auto profile_t3 = std::chrono::steady_clock::now();
    const std::vector<ObstacleSphere> * internal_solver_obstacles = &obstacles;
    std::vector<ObstacleSphere> internal_main_obstacles;
    if (internal_avoidance_gate_enabled_ &&
      apply_internal_avoidance_gate(state, obstacles, internal_main_obstacles, external_rmps))
    {
      internal_solver_obstacles = &internal_main_obstacles;
    }
    const auto profile_t4 = std::chrono::steady_clock::now();
    const auto solution = solver_->solve(
      state.q,
''',
        "internal target insertion",
    )
    source = replace_once(
        source,
        '''      vector_targets,
      obstacles,
      external_rmps);
    if (tangent_escape_rmp_data != nullptr) {
''',
        '''      vector_targets,
      *internal_solver_obstacles,
      external_rmps);
    if (internal_profile_log_enabled_) {
      const auto profile_t5 = std::chrono::steady_clock::now();
      const auto ms = [](auto a, auto b) {
          return std::chrono::duration<double, std::milli>(b - a).count();
        };
      internal_profile_sum_[0] += ms(profile_t0, profile_t1);
      internal_profile_sum_[1] += ms(profile_t1, profile_t2);
      internal_profile_sum_[2] += ms(profile_t2, profile_t3);
      internal_profile_sum_[3] += ms(profile_t3, profile_t4);
      internal_profile_sum_[4] += ms(profile_t4, profile_t5);
      internal_profile_sum_[5] += ms(profile_t0, profile_t5);
      ++internal_profile_count_;
      if (profile_admittance_active) {
        ++internal_profile_active_count_;
      }
      if (profile_t5 - internal_profile_last_ > std::chrono::seconds(2)) {
        const double n = static_cast<double>(std::max<std::int64_t>(internal_profile_count_, 1));
        RCLCPP_WARN(
          get_logger(),
          "PROFILE ms/cycle (n=%ld, admittance active %ld): admittance %.3f target %.3f "
          "orientation %.3f avoidance %.3f main_solve %.3f total %.3f",
          static_cast<long>(internal_profile_count_),
          static_cast<long>(internal_profile_active_count_),
          internal_profile_sum_[0] / n, internal_profile_sum_[1] / n,
          internal_profile_sum_[2] / n, internal_profile_sum_[3] / n,
          internal_profile_sum_[4] / n, internal_profile_sum_[5] / n);
        internal_profile_sum_.fill(0.0);
        internal_profile_count_ = 0;
        internal_profile_active_count_ = 0;
        internal_profile_last_ = profile_t5;
      }
    }
    if (tangent_escape_rmp_data != nullptr) {
''',
        "v2 gated obstacle list",
    )
    # v2: remember which obstacles came from proximity-sensor markers.  The
    # production index field stays -1 on UR10 (empty sensor_control_points),
    # so the gate keys on the marker-derived source_id instead.
    source = replace_once(
        source,
        '''    std::vector<ObstacleSphere> obstacles;
    std::vector<std::optional<ObstacleSphere>> proximity_sensor_obstacles(
''',
        '''    std::vector<ObstacleSphere> obstacles;
    std::unordered_set<std::int64_t> internal_proximity_source_ids;
    std::vector<std::optional<ObstacleSphere>> proximity_sensor_obstacles(
''',
        "v2 proximity id set declaration",
    )
    source = replace_once(
        source,
        '''      obstacle.source_id = obstacle_source_id(marker);
''',
        '''      obstacle.source_id = obstacle_source_id(marker);
      if (marker.ns == "proximity_obstacles") {
        internal_proximity_source_ids.insert(obstacle.source_id);
      }
''',
        "v2 proximity id collection",
    )
    source = replace_once(
        source,
        '''    obstacles_box_.set(obstacles);
    proximity_sensor_obstacles_box_.set(proximity_sensor_obstacles);
''',
        '''    {
      std::scoped_lock lock(internal_proximity_ids_mutex_);
      internal_proximity_source_ids_ = std::move(internal_proximity_source_ids);
    }
    obstacles_box_.set(obstacles);
    proximity_sensor_obstacles_box_.set(proximity_sensor_obstacles);
''',
        "v2 proximity id publication",
    )
    source = replace_once(
        source,
        '  TargetRmpParams target_metric_params_;\n',
        '''  TargetRmpParams target_metric_params_;
  bool internal_contact_target_gate_enabled_{false};
  double internal_contact_target_metric_floor_{0.01};
  double internal_contact_target_activation_timeout_sec_{0.20};
  std::string internal_contact_target_key_{"target_gated"};
  std::atomic<double> internal_contact_target_activation_{0.0};
  std::atomic<std::int64_t> internal_contact_target_activation_stamp_ns_{0};
  std::unique_ptr<PinocchioModel> internal_contact_target_model_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    internal_contact_target_activation_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    internal_contact_target_scale_pub_;
  rclcpp::TimerBase::SharedPtr internal_contact_target_diagnostic_timer_;
  bool internal_admittance_enabled_{false};
  std::string internal_admittance_key_{"admittance"};
  Eigen::Vector3d internal_admittance_virtual_mass_{Eigen::Vector3d::Constant(3.0)};
  Eigen::Vector3d internal_admittance_damping_{Eigen::Vector3d::Constant(25.0)};
  Eigen::Vector3d internal_admittance_metric_{Eigen::Vector3d::Constant(300.0)};
  JointVector internal_admittance_max_joint_accel_{
    (JointVector() << 2.5, 1.5, 1.5, 1.5, 1.5, 1.5).finished()};
  double internal_admittance_enter_force_n_{4.0};
  double internal_admittance_release_force_n_{2.0};
  double internal_admittance_enter_probability_{0.60};
  double internal_admittance_release_probability_{0.35};
  double internal_admittance_transition_sec_{0.30};
  double internal_admittance_wrench_timeout_sec_{0.25};
  double internal_admittance_intent_timeout_sec_{0.50};
  double internal_admittance_max_force_n_{30.0};
  double internal_admittance_max_task_accel_{6.0};
  double internal_admittance_singular_damping_ratio_{0.05};
  double internal_admittance_minimum_alignment_cosine_{0.50};
  double internal_admittance_eigenvalue_floor_{1e-8};
  bool internal_admittance_base_lever_enabled_{false};
  double internal_admittance_base_lever_reference_{0.28};
  double internal_admittance_base_lever_max_factor_{20.0};
  mutable std::atomic<double> internal_admittance_base_lever_factor_{1.0};
  mutable std::atomic<double> internal_admittance_base_lever_{-1.0};
  double internal_admittance_base_lever_unreachable_{0.0};
  bool internal_admittance_alignment_gate_enabled_{true};
  bool internal_admittance_force_authority_{false};
  double internal_admittance_force_authority_low_{2.0};
  double internal_admittance_force_authority_high_{6.0};
  double internal_admittance_release_time_sec_{0.0};
  bool internal_profile_log_enabled_{false};
  bool internal_intent_evidence_mode_{false};
  ur10_intent_admittance_rmp_v2::IntentEvidenceParams internal_intent_evidence_params_;
  mutable ur10_intent_admittance_rmp_v2::IntentEvidenceState internal_intent_evidence_state_;
  mutable std::atomic<double> internal_intent_compliance_gate_{0.0};
  bool internal_contact_target_geometric_{false};
  mutable std::array<double, 6> internal_profile_sum_{};
  mutable std::int64_t internal_profile_count_{0};
  mutable std::int64_t internal_profile_active_count_{0};
  mutable std::chrono::steady_clock::time_point internal_profile_last_{};
  bool internal_contact_target_push_gradient_{false};
  mutable std::mutex internal_push_projection_mutex_;
  mutable ur10_intent_admittance_rmp_v2::PushProjection internal_push_projection_;
  mutable std::atomic<double> internal_push_projection_strength_{0.0};
  mutable std::mutex internal_admittance_mutex_;
  mutable Eigen::Vector3d internal_admittance_force_{Eigen::Vector3d::Zero()};
  mutable Eigen::Vector3d internal_admittance_point_{Eigen::Vector3d::Zero()};
  mutable std::size_t internal_admittance_link_{UR10Model::LINK_COUNT};
  mutable double internal_admittance_probability_{0.0};
  mutable std::int64_t internal_admittance_wrench_stamp_ns_{0};
  mutable std::int64_t internal_admittance_intent_stamp_ns_{0};
  mutable bool internal_admittance_have_point_{false};
  mutable bool internal_admittance_have_wrench_{false};
  mutable bool internal_admittance_have_intent_{false};
  mutable bool internal_admittance_requested_active_{false};
  mutable double internal_admittance_request_phase_{0.0};
  mutable std::atomic<double> internal_admittance_effective_activation_{0.0};
  bool internal_admittance_diagnostics_enabled_{true};
  std::int64_t internal_admittance_diagnostics_period_ns_{50000000};
  mutable std::atomic<std::int64_t> internal_admittance_last_diagnostics_ns_{0};
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
    internal_admittance_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    internal_admittance_wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
    internal_admittance_intent_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    internal_admittance_activation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    internal_admittance_diagnostics_pub_;
  bool internal_admittance_latch_enabled_{false};
  bool internal_avoidance_gate_enabled_{false};
  std::string internal_avoidance_key_{"collision_gated"};
  double internal_avoidance_transition_sec_{0.30};
  double internal_avoidance_weight_floor_{0.0};
  double internal_avoidance_guard_velocity_{0.0};
  mutable std::atomic<double> internal_guard_velocity_{0.0};
  std::unique_ptr<RmpSolverInterface> internal_avoidance_solver_;
  mutable std::atomic<double> internal_avoidance_weight_{1.0};
  mutable std::atomic<double> internal_intent_effective_{0.0};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr internal_avoidance_weight_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr internal_intent_effective_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    internal_avoidance_diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr internal_avoidance_diagnostic_timer_;
  mutable std::atomic<double> internal_avoidance_proximity_count_{0.0};
  mutable std::atomic<double> internal_avoidance_metric_trace_{0.0};
  mutable std::mutex internal_proximity_ids_mutex_;
  std::unordered_set<std::int64_t> internal_proximity_source_ids_;
  bool internal_orientation_gate_enabled_{false};
  std::string internal_orientation_key_{"orientation_gated"};
  std::unique_ptr<RmpSolverInterface> internal_orientation_solver_;
  mutable std::atomic<double> internal_orientation_scale_{1.0};
''',
        "internal target members",
    )
    output.write_text(source)


def generate_launches(source_root: Path, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    overlay = (source_root / "launch" / "rb10_rmpflow_escape_overlay.launch.py").read_text()
    overlay = replace_once(
        overlay,
        '''                package="rb10_rmpflow_rviz",
                executable="rmpflow_controller",
''',
        '''                package="ur10_intent_admittance_rmp_v2",
                executable="rmpflow_internal_gated_controller",
''',
        "experimental controller launch substitution",
    )
    # v2 diagnostics: the production overlay hard-codes per-leaf ablation
    # data OFF.  In the fork copy only, read it from a launch configuration
    # (default "false", i.e. unchanged).  Observation only: the solver never
    # feeds this data back into the commanded qdd.
    overlay = replace_once(
        overlay,
        '''            "publish_leaf_ablation_data": False,
''',
        '''            "publish_leaf_ablation_data": _as_bool(
                LaunchConfiguration(
                    "publish_leaf_ablation_data", default="false"
                ).perform(context)
            ),
''',
        "v2 leaf ablation launch configuration",
    )
    (output_dir / "internal_gate_escape_overlay.launch.py").write_text(overlay)

    base = (source_root / "launch" / "ur10_proximity1_position_goal_avoidance_urcb2.launch.py").read_text()
    base = replace_once(
        base,
        '''    overlay_launch = os.path.join(
        package_share,
        "launch",
        "rb10_rmpflow_escape_overlay.launch.py",
    )
''',
        '''    overlay_launch = os.path.join(
        get_package_share_directory("ur10_intent_admittance_rmp_v2"),
        "launch",
        "internal_gate_escape_overlay.launch.py",
    )
''',
        "experimental overlay launch substitution",
    )
    # v2: the production launch hard-codes the SO(3) orientation leaf ON.  In
    # the fork copy only, make it a launch configuration whose default keeps
    # that behaviour (Stage 2 unchanged); stage 4 passes "false".
    base = replace_once(
        base,
        '''            "enable_so3_orientation_policy": "true",
''',
        '''            "enable_so3_orientation_policy": LaunchConfiguration(
                "enable_so3_orientation_policy", default="true"
            ),
''',
        "v2 SO3 orientation launch configuration",
    )
    (output_dir / "internal_gate_ur10_proximity_urcb2.launch.py").write_text(base)

    fused = (source_root / "launch" / "ur10_fused2_position_goal_avoidance_urcb2.launch.py").read_text()
    fused = replace_once(
        fused,
        '''    base_launch = os.path.join(
        package_share,
        "launch",
        "ur10_proximity1_position_goal_avoidance_urcb2.launch.py",
    )
''',
        '''    base_launch = os.path.join(
        get_package_share_directory("ur10_intent_admittance_rmp_v2"),
        "launch",
        "internal_gate_ur10_proximity_urcb2.launch.py",
    )
''',
        "experimental fused launch substitution",
    )
    (output_dir / "internal_gate_ur10_fused2_urcb2.launch.py").write_text(fused)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--controller-output", type=Path, required=True)
    parser.add_argument("--launch-output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.controller_output.parent.mkdir(parents=True, exist_ok=True)
    generate_controller(args.source_root, args.controller_output)
    generate_launches(args.source_root, args.launch_output_dir)


if __name__ == "__main__":
    main()
