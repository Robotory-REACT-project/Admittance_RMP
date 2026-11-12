#!/usr/bin/env python3
"""Isolated seven-policy UR10 experiment.

The production six-policy launcher and controller are included unchanged.  A
separate node publishes a seventh, zero-by-default admittance RMP through the
controller's existing external-RMP interface.  Hardware output remains off by
default and requires a second explicit acknowledgement argument.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _validate_hardware_acknowledgement(context):
    hardware = _as_bool(LaunchConfiguration("hardware_command_enabled").perform(context))
    acknowledged = _as_bool(
        LaunchConfiguration("acknowledge_experimental_hardware_risk").perform(context)
    )
    if hardware and not acknowledged:
        raise RuntimeError(
            "Experimental seven-RMP hardware output requires both "
            "hardware_command_enabled:=true and "
            "acknowledge_experimental_hardware_risk:=true"
        )
    return []


def generate_launch_description():
    experiment_share = get_package_share_directory("ur10_intent_admittance_rmp_v2")
    production_share = get_package_share_directory("rb10_rmpflow_rviz")
    production_launch = os.path.join(
        production_share,
        "launch",
        "ur10_fused2_position_goal_avoidance_urcb2.launch.py",
    )
    internal_gate_launch = os.path.join(
        experiment_share, "launch", "internal_gate_ur10_fused2_urcb2.launch.py"
    )
    graph_params = LaunchConfiguration("graph_params_file")
    admittance_params = os.path.join(
        experiment_share, "config", "admittance_external_rmp.yaml"
    )
    adapter_params = os.path.join(
        experiment_share, "config", "contact_force_adapter.yaml"
    )
    gated_target_params = os.path.join(
        experiment_share, "config", "gated_target_external_rmp.yaml"
    )
    react_share = Path(get_package_share_directory("REACT_intrinsic"))
    react_config = str(react_share / "config" / "JSshape_config.yaml")
    contact_mesh = str(react_share / "meshes" / "JSshape_surface.stl")
    ft_share = get_package_share_directory("Y2FT_AQ")
    ft_launch = os.path.join(ft_share, "launch", "ft_get_main.launch.py")

    external_leaf = Node(
        package="ur10_intent_admittance_rmp_v2",
        executable="admittance_external_rmp",
        name="admittance_external_rmp",
        output="screen",
        parameters=[
            admittance_params,
            {
                "contact_metric_scale": ParameterValue(
                    LaunchConfiguration("admittance_contact_metric_scale"),
                    value_type=float,
                )
            },
        ],
        condition=UnlessCondition(LaunchConfiguration("use_internal_gated_target")),
        on_exit=Shutdown(
            reason="required experimental admittance RMP publisher exited"
        ),
    )

    gated_target_leaf = Node(
        package="ur10_intent_admittance_rmp_v2",
        executable="gated_target_external_rmp",
        name="gated_target_external_rmp",
        output="screen",
        parameters=[gated_target_params],
        condition=IfCondition(LaunchConfiguration("use_gated_target")),
        on_exit=Shutdown(
            reason="required experimental contact-gated target RMP publisher exited"
        ),
    )

    ft_acquisition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ft_launch),
        condition=IfCondition(LaunchConfiguration("start_ft_acquisition")),
    )

    calibration_tf_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="admittance_wrench_calibration_tf_1",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "3.14159265359", "--pitch", "0.0", "--yaw", "3.14159265359",
            "--frame-id", "ur10_jsshape_supporter1",
            "--child-frame-id", "ur10_jsshape_wrench1",
        ],
        condition=IfCondition(LaunchConfiguration("use_contact_pipeline")),
        output="screen",
    )
    calibration_tf_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="admittance_wrench_calibration_tf_2",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.0",
            "--roll", "3.14159265359", "--pitch", "0.0", "--yaw", "3.14159265359",
            "--frame-id", "ur10_jsshape_supporter2",
            "--child-frame-id", "ur10_jsshape_wrench2",
        ],
        condition=IfCondition(LaunchConfiguration("use_contact_pipeline")),
        output="screen",
    )

    def contact_localizer(sensor_index):
        return Node(
            package="REACT_intrinsic",
            executable="main_ifs",
            name=f"jsshape_sensor{sensor_index}_contactsensing",
            parameters=[
                react_config,
                {
                    "mesh_directory": contact_mesh,
                    "contact_sensor_topic": f"/ur10skku/ftdata_tcp_{sensor_index}",
                    "contact_force_threshold": ParameterValue(
                        LaunchConfiguration("contact_force_threshold"),
                        value_type=float,
                    ),
                    "use_wrench_tf": True,
                    "contact_frame": f"ur10_jsshape_supporter{sensor_index}",
                    "wrench_source_frame": f"ur10_jsshape_wrench{sensor_index}",
                    "wrench_tf_timeout_sec": 0.05,
                },
            ],
            remappings=[
                ("/contactsensing/cp", f"/sensor{sensor_index}/contactsensing/cp"),
                ("/contactsensing/force_arrow", f"/sensor{sensor_index}/contactsensing/force_arrow"),
                ("/contactsensing/force_magnitude", f"/sensor{sensor_index}/contactsensing/force_magnitude"),
                ("/contactsensing/wrench_corrected", f"/sensor{sensor_index}/contactsensing/wrench_corrected"),
                ("/contactsensing/contact_state", f"/sensor{sensor_index}/contactsensing/contact_state"),
                ("/mesh_pc", f"/sensor{sensor_index}/mesh_pc"),
            ],
            condition=IfCondition(LaunchConfiguration("use_contact_pipeline")),
            output="screen",
        )

    contact_adapter = Node(
        package="ur10_intent_admittance_rmp_v2",
        executable="contact_force_adapter",
        name="contact_force_adapter",
        parameters=[
            adapter_params,
            {
                "publish_manual_intent": ParameterValue(
                    LaunchConfiguration("publish_manual_intent"), value_type=bool
                ),
                "manual_compliance_probability": ParameterValue(
                    LaunchConfiguration("manual_compliance_probability"),
                    value_type=float,
                ),
                "force_sign": ParameterValue(
                    LaunchConfiguration("contact_force_sign"), value_type=float
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_contact_pipeline")),
        output="screen",
    )

    controller_launch_arguments = {
            "params_file": graph_params,
            "hardware_command_enabled": LaunchConfiguration(
                "hardware_command_enabled"
            ),
            "start_urcb2_driver": LaunchConfiguration("start_urcb2_driver"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_interactive_goal": LaunchConfiguration("use_interactive_goal"),
            "joint_state_topic": LaunchConfiguration("joint_state_topic"),
            "sensor_enabled": LaunchConfiguration("sensor_enabled"),
            "measured_position_feedback_blend": LaunchConfiguration(
                "measured_position_feedback_blend"
            ),
            "measured_velocity_feedback_blend": LaunchConfiguration(
                "measured_velocity_feedback_blend"
            ),
            "latency_compensation_enabled": LaunchConfiguration(
                "latency_compensation_enabled"
            ),
            "require_goal_change_to_arm": LaunchConfiguration(
                "require_goal_change_to_arm"
            ),
            "goal_settle_hold_enabled": LaunchConfiguration(
                "goal_settle_hold_enabled"
            ),
            "max_joint_accel": LaunchConfiguration("max_joint_accel"),
            "command_guard_max_velocity_rad_s": LaunchConfiguration(
                "command_guard_max_velocity_rad_s"
            ),
            "command_guard_max_step_rad": LaunchConfiguration(
                "command_guard_max_step_rad"
            ),
        }
    production_six_rmp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(production_launch),
        launch_arguments=controller_launch_arguments.items(),
        condition=UnlessCondition(LaunchConfiguration("use_internal_gated_target")),
    )
    internal_gated_rmp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(internal_gate_launch),
        launch_arguments=controller_launch_arguments.items(),
        condition=IfCondition(LaunchConfiguration("use_internal_gated_target")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_command_enabled", default_value="false"),
            DeclareLaunchArgument(
                "acknowledge_experimental_hardware_risk", default_value="false"
            ),
            DeclareLaunchArgument("start_urcb2_driver", default_value="true"),
            DeclareLaunchArgument(
                "graph_params_file",
                default_value=os.path.join(
                    experiment_share, "config", "params_7rmp_experimental.yaml"
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_interactive_goal", default_value="true"),
            DeclareLaunchArgument(
                "joint_state_topic", default_value="/UR10_right/joint_states"
            ),
            DeclareLaunchArgument(
                "sensor_enabled",
                default_value="true,true,false,true",
                description=(
                    "Preserves the production default. Enable sensor 3 only after "
                    "its self-detection geometry has been validated."
                ),
            ),
            DeclareLaunchArgument(
                "measured_position_feedback_blend", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "measured_velocity_feedback_blend", default_value="0.25"
            ),
            DeclareLaunchArgument(
                "latency_compensation_enabled", default_value="true"
            ),
            DeclareLaunchArgument("require_goal_change_to_arm", default_value="false"),
            DeclareLaunchArgument("goal_settle_hold_enabled", default_value="false"),
            DeclareLaunchArgument("max_joint_accel", default_value="1.0"),
            DeclareLaunchArgument(
                "command_guard_max_velocity_rad_s", default_value="0.25"
            ),
            DeclareLaunchArgument("command_guard_max_step_rad", default_value="0.002"),
            DeclareLaunchArgument(
                "use_contact_pipeline",
                default_value="true",
                description=(
                    "Start sensor-1/2 contact localization and the RMP input adapter"
                ),
            ),
            DeclareLaunchArgument(
                "use_gated_target",
                default_value="false",
                description=(
                    "Replace the built-in Cartesian target leaf with the "
                    "experimental contact-gated external target leaf"
                ),
            ),
            DeclareLaunchArgument(
                "use_internal_gated_target",
                default_value="false",
                description=(
                    "Use the isolated controller fork that gates the target "
                    "inside the q_ctrl/qd_ctrl solve cycle"
                ),
            ),
            DeclareLaunchArgument(
                "start_ft_acquisition",
                default_value="false",
                description=(
                    "Start Y2FT_AQ in this launch. Keep false if FTGetMain is already running"
                ),
            ),
            DeclareLaunchArgument("contact_force_threshold", default_value="1.5"),
            DeclareLaunchArgument(
                "admittance_contact_metric_scale",
                default_value="1.0",
                description=(
                    "Stage-specific multiplier for the shared admittance contact metric"
                ),
            ),
            DeclareLaunchArgument("publish_manual_intent", default_value="true"),
            DeclareLaunchArgument(
                "manual_compliance_probability", default_value="1.0"
            ),
            DeclareLaunchArgument("contact_force_sign", default_value="1.0"),
            OpaqueFunction(function=_validate_hardware_acknowledgement),
            LogInfo(
                msg=[
                    "EXPERIMENTAL 7-RMP launch: production six-policy controller is ",
                    "unchanged; admittance is an external zero-on-timeout leaf. ",
                    "Hardware commands=",
                    LaunchConfiguration("hardware_command_enabled"),
                ]
            ),
            external_leaf,
            gated_target_leaf,
            ft_acquisition,
            calibration_tf_1,
            calibration_tf_2,
            contact_localizer(1),
            contact_localizer(2),
            contact_adapter,
            production_six_rmp,
            internal_gated_rmp,
        ]
    )
