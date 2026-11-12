#!/usr/bin/env python3
"""Sensor-1/2 admittance-only RViz preview; never publishes a robot command."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    experiment_share = Path(get_package_share_directory("ur10_intent_admittance_rmp_v2"))
    react_share = Path(get_package_share_directory("REACT_intrinsic"))
    rmp_share = Path(get_package_share_directory("rb10_rmpflow_rviz"))
    robot_description = (rmp_share / "urdf" / "ur10.urdf").read_text(encoding="utf-8")

    def calibration_tf(index):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"preview_wrench_calibration_tf_{index}",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "3.14159265359", "--pitch", "0.0", "--yaw", "3.14159265359",
                "--frame-id", f"ur10_jsshape_supporter{index}",
                "--child-frame-id", f"ur10_jsshape_wrench{index}",
            ],
            output="screen",
        )

    def contact_localizer(index):
        return Node(
            package="REACT_intrinsic",
            executable="main_ifs",
            name=f"jsshape_sensor{index}_contactsensing",
            parameters=[
                str(react_share / "config" / "JSshape_config.yaml"),
                {
                    "mesh_directory": str(react_share / "meshes" / "JSshape_surface.stl"),
                    "contact_sensor_topic": f"/ur10skku/ftdata_tcp_{index}",
                    "contact_force_threshold": ParameterValue(
                        LaunchConfiguration("contact_force_threshold"), value_type=float
                    ),
                    "use_wrench_tf": True,
                    "contact_frame": f"ur10_jsshape_supporter{index}",
                    "wrench_source_frame": f"ur10_jsshape_wrench{index}",
                    "wrench_tf_timeout_sec": 0.05,
                },
            ],
            remappings=[
                ("/contactsensing/cp", f"/sensor{index}/contactsensing/cp"),
                ("/contactsensing/force_arrow", f"/sensor{index}/contactsensing/force_arrow"),
                ("/contactsensing/force_magnitude", f"/sensor{index}/contactsensing/force_magnitude"),
                ("/contactsensing/wrench_corrected", f"/sensor{index}/contactsensing/wrench_corrected"),
                ("/contactsensing/contact_state", f"/sensor{index}/contactsensing/contact_state"),
                ("/mesh_pc", f"/sensor{index}/mesh_pc"),
            ],
            output="screen",
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("contact_force_threshold", default_value="1.5"),
        DeclareLaunchArgument("contact_force_sign", default_value="1.0"),
        calibration_tf(1),
        calibration_tf(2),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="admittance_actual",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "publish_frequency": 50.0}],
            remappings=[
                ("joint_states", "/UR10_right/joint_states"),
                ("robot_description", "/admittance_actual/robot_description"),
            ],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="admittance_preview",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 50.0,
                "frame_prefix": "preview/",
            }],
            remappings=[
                ("joint_states", "/admittance_preview/joint_states"),
                ("robot_description", "/admittance_preview/robot_description"),
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="admittance_preview_base_anchor",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "preview/base_link",
            ],
            output="screen",
        ),
        contact_localizer(1),
        contact_localizer(2),
        Node(
            package="ur10_intent_admittance_rmp_v2",
            executable="contact_force_adapter",
            name="contact_force_adapter",
            parameters=[
                str(experiment_share / "config" / "contact_force_adapter.yaml"),
                {
                    "publish_manual_intent": True,
                    "manual_compliance_probability": 1.0,
                    "force_sign": ParameterValue(
                        LaunchConfiguration("contact_force_sign"), value_type=float
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="ur10_intent_admittance_rmp_v2",
            executable="admittance_preview",
            name="admittance_preview",
            parameters=[str(experiment_share / "config" / "admittance_preview.yaml")],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="admittance_preview_rviz",
            arguments=["-d", str(experiment_share / "config" / "admittance_preview.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="screen",
        ),
    ])
