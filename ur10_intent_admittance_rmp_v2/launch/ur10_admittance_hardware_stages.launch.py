#!/usr/bin/env python3
"""Three supervised UR10 contact-admittance hardware stages."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _stage_actions(context):
    stage = LaunchConfiguration("stage").perform(context).strip()
    if stage not in {"1", "2", "3", "4", "5", "6", "7", "8", "9"}:
        raise RuntimeError("stage must be exactly 1..9")
    velocity_cap = float(LaunchConfiguration("velocity_cap").perform(context))
    if not 0.06 < velocity_cap < 0.4:
        raise RuntimeError("velocity_cap must be in (0.06, 0.4) rad/s (command guard is 0.4)")
    share = get_package_share_directory("ur10_intent_admittance_rmp_v2")
    launch_file = os.path.join(
        share, "launch", "ur10_7rmp_intent_admittance_experimental.launch.py"
    )
    graph_files = {
        "1": "params_admittance_stage1.yaml",
        "2": "params_admittance_stage2.yaml",
        "3": "params_7rmp_experimental.yaml",
        "4": "params_admittance_stage4.yaml",
        "5": "params_admittance_stage5.yaml",
        "6": "params_admittance_stage6.yaml",
        "7": "params_admittance_stage7.yaml",
        "8": "params_admittance_stage8.yaml",
        "9": "params_admittance_stage9.yaml",
    }
    descriptions = {
        "1": "admittance + reduced damping + velocity-cap; target/avoidance and centering joint-limit RMP OFF (hard joint guards remain ON)",
        "2": "contact-gated Cartesian target ON; avoidance OFF; admittance ON",
        "3": "production target/orientation/avoidance ON; admittance ON",
        "4": "v2: avoidance + admittance, target/orientation OFF; proximity avoidance weighted by intent (w = 1 - p_eff); contact latch ON",
        "5": "v2: stage 4 + production position and SO(3) orientation targets, both weakened during contact and restored after release",
        "6": "v2: stage 5 with RMP-consistent contact arbitration (GM): force-based authority, push-gradient target projection, no alignment gate",
        "7": "v2: stage 6 + slow target restore after release (activation falls over 1 s)",
        "8": "v2: stage 6 + faster avoidance (command guard raised only while avoiding)",
        "9": "v2 goal 2: stage 8 + continuous intent (force-held lambda_eff, smooth compliance gate, geometric target scale)",
    }
    # Stages 1 and 2 intentionally suppress every proximity channel. Stage 3
    # restores the current production mask; proximity sensor 3 remains disabled
    # until its previously observed self-detection has been cleared.
    sensor_mask = "false,false,false,false" if stage in {"1", "2"} else "true,true,false,true"
    if stage in {"4", "5", "6", "7", "8", "9"}:
        # v2 stages: sensor 4 reads ~0.24 m with nothing in front of it
        # (2026-11-05 dry-run) and has misbehaved before, so only 1 and 2.
        sensor_mask = "true,true,false,false"
    # Offline 30 N dry-run tuning showed that increasing the contact metric
    # alone cannot recover Stage-1 motion. Stage 2 therefore keeps the verified
    # contact metric 300 and gates the target metric during active contact.
    contact_metric_scale = "1.0"

    launch_arguments = {
        "graph_params_file": os.path.join(share, "config", graph_files[stage]),
        "hardware_command_enabled": LaunchConfiguration("hardware_command_enabled"),
        "acknowledge_experimental_hardware_risk": LaunchConfiguration(
            "acknowledge_experimental_hardware_risk"
        ),
        "start_urcb2_driver": LaunchConfiguration("start_urcb2_driver"),
        "start_ft_acquisition": "false",
        "use_contact_pipeline": "true",
        # The former ROS-external target is intentionally never launched.
        # Stage 2 uses the isolated internal fork so target/admittance
        # arbitration is evaluated from one q_ctrl/qd_ctrl snapshot.
        "use_gated_target": "false",
        # Stages 2, 4 and 5 run the isolated controller fork; 4 and 5 need it
        # for the internal admittance latch and intent-gated avoidance.
        "use_internal_gated_target": "true" if stage in {"2", "4", "5", "6", "7", "8", "9"} else "false",
        "admittance_contact_metric_scale": contact_metric_scale,
        "use_rviz": LaunchConfiguration("use_rviz"),
        "use_interactive_goal": "false" if stage in {"1", "4"} else "true",
        "sensor_enabled": sensor_mask,
        "require_goal_change_to_arm": "false",
        "goal_settle_hold_enabled": "false",
        "max_joint_accel": LaunchConfiguration("max_joint_accel"),
        "command_guard_max_velocity_rad_s": LaunchConfiguration(
            "command_guard_max_velocity_rad_s"
        ),
        "command_guard_max_step_rad": LaunchConfiguration(
            "command_guard_max_step_rad"
        ),
        # false = stop the contact adapter's fixed p = 1.0 so p can come
        # from vision or a manual `ros2 topic pub`.
        "publish_manual_intent": LaunchConfiguration("publish_manual_intent"),
    }
    if stage == "4":
        # The production launch hard-codes the SO(3) orientation leaf ON; the
        # v2 fork copy reads this instead (default "true" keeps Stage 2 as is).
        launch_arguments["enable_so3_orientation_policy"] = "false"
    elif stage in {"5", "6", "7", "8", "9"}:
        # Stage 5 keeps the production SO(3) leaf ON; the fork then moves it
        # into the contact-gated orientation_gated external leaf.
        launch_arguments["enable_so3_orientation_policy"] = "true"
        # 2026-11-07 (goal 1-2): the fork launch declares these with defaults
        # 2.4336 / 1.2168, which override the graph yaml. Keep the RMP's own
        # joint speed below the 0.25 rad/s command guard so the return after
        # release is not clipped by the guard (pan overshoot 10-33 % in run 7).
        # 2026-11-07 (run 10): exposed as `velocity_cap` (default 0.24) because
        # the cap also limits avoidance speed; the effective command guard is
        # max(0.25, 0.002 rad x 200 Hz) = 0.4 rad/s, so keep it below 0.4.
        launch_arguments["joint_velocity_cap_max_velocity"] = LaunchConfiguration("velocity_cap")
        if stage in {"8", "9"}:
            # The RMP velocity cap must sit above the raised avoidance guard
            # (0.35 in params_admittance_stage8.yaml); outside avoidance the
            # 0.25 rad/s command guard still bounds every joint.
            launch_arguments["joint_velocity_cap_max_velocity"] = f"{max(velocity_cap, 0.38):.3f}"
        launch_arguments["joint_velocity_cap_velocity_damping_region"] = "0.06"
    included = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=launch_arguments.items(),
    )
    return [
        LogInfo(msg=f"UR10 ADMITTANCE HARDWARE STAGE {stage}: {descriptions[stage]}"),
        included,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stage", default_value="1"),
        DeclareLaunchArgument("hardware_command_enabled", default_value="false"),
        DeclareLaunchArgument(
            "acknowledge_experimental_hardware_risk", default_value="false"
        ),
        # The lab machine currently has a separately running single_arm. Keep
        # this false unless that process has first been stopped deliberately.
        DeclareLaunchArgument("start_urcb2_driver", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        # The external leaf retains per-joint limits (2.5 for shoulder_pan,
        # 1.5 otherwise). The root ceiling must admit the larger lateral term.
        # The independent 0.25 rad/s command guard remains unchanged.
        DeclareLaunchArgument("max_joint_accel", default_value="2.5"),
        DeclareLaunchArgument(
            "command_guard_max_velocity_rad_s", default_value="0.25"
        ),
        DeclareLaunchArgument("command_guard_max_step_rad", default_value="0.002"),
        DeclareLaunchArgument("publish_manual_intent", default_value="true"),
        # Stages 5-7 only: RMP joint velocity cap (rad/s). Must stay < 0.4.
        DeclareLaunchArgument("velocity_cap", default_value="0.24"),
        OpaqueFunction(function=_stage_actions),
    ])
