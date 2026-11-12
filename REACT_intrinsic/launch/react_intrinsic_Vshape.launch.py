from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('REACT_intrinsic'))
    robot_description = (share / 'urdf' / 'react_intrinsic_Vshape.urdf').read_text(encoding='utf-8')
    visual_mesh_uri = (share / 'meshes' / 'vshape.STL').as_uri()
    robot_description = robot_description.replace(
        'package://REACT_intrinsic/meshes/vshape.STL',
        visual_mesh_uri,
    )

    sensor_topic = LaunchConfiguration('contact_sensor_topic')
    force_threshold = LaunchConfiguration('contact_force_threshold')
    contact_frame = LaunchConfiguration('contact_frame')
    wrench_source_frame = LaunchConfiguration('wrench_source_frame')
    wrench_tf_timeout = LaunchConfiguration('wrench_tf_timeout_sec')
    calibration_x = LaunchConfiguration('wrench_calibration_x')
    calibration_y = LaunchConfiguration('wrench_calibration_y')
    calibration_z = LaunchConfiguration('wrench_calibration_z')
    calibration_roll = LaunchConfiguration('wrench_calibration_roll')
    calibration_pitch = LaunchConfiguration('wrench_calibration_pitch')
    calibration_yaw = LaunchConfiguration('wrench_calibration_yaw')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'contact_sensor_topic',
            default_value='/ur10skku/ftdata',
            description='geometry_msgs/msg/WrenchStamped input topic',
        ),
        DeclareLaunchArgument(
            'contact_force_threshold',
            default_value='1.0',
            description='Minimum filtered force magnitude [N] used for contact calculation',
        ),
        DeclareLaunchArgument(
            'contact_frame',
            default_value='base_link',
            description='Frame used by the contact mesh and published contact results',
        ),
        DeclareLaunchArgument(
            'wrench_source_frame',
            default_value='vshape_wrench',
            description='Calibrated frame assigned to the incoming wrench values',
        ),
        DeclareLaunchArgument('wrench_tf_timeout_sec', default_value='0.05'),
        DeclareLaunchArgument(
            'wrench_calibration_x', default_value='0.0',
            description='Wrench reference origin X position in contact_frame [m]',
        ),
        DeclareLaunchArgument(
            'wrench_calibration_y', default_value='0.0',
            description='Wrench reference origin Y position in contact_frame [m]',
        ),
        DeclareLaunchArgument(
            'wrench_calibration_z', default_value='0.0',
            description='Wrench reference origin Z position in contact_frame [m]',
        ),
        DeclareLaunchArgument('wrench_calibration_roll', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_pitch', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='vshape_wrench_calibration_tf',
            arguments=[
                '--x', calibration_x,
                '--y', calibration_y,
                '--z', calibration_z,
                '--roll', calibration_roll,
                '--pitch', calibration_pitch,
                '--yaw', calibration_yaw,
                '--frame-id', contact_frame,
                '--child-frame-id', wrench_source_frame,
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='REACT_intrinsic',
            executable='main_ifs',
            name='nonlinear_surface_contactsensing',
            parameters=[
                str(share / 'config' / 'Vshape_config.yaml'),
                {
                    'mesh_directory': str(share / 'meshes' / 'vshape_surface.STL'),
                    'contact_sensor_topic': sensor_topic,
                    'contact_force_threshold': ParameterValue(force_threshold, value_type=float),
                    'use_wrench_tf': True,
                    'contact_frame': contact_frame,
                    'wrench_source_frame': wrench_source_frame,
                    'wrench_tf_timeout_sec': ParameterValue(wrench_tf_timeout, value_type=float),
                },
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'Vshape.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
