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
    robot_description = (share / 'urdf' / 'intrinsic_test_v4.urdf').read_text(encoding='utf-8')
    visual_mesh_uri = (share / 'meshes' / 'intrinsic_test_v4.STL').as_uri()
    robot_description = robot_description.replace(
        'package://REACT_intrinsic/meshes/intrinsic_test_v4.STL',
        visual_mesh_uri,
    )

    sensor_topic = LaunchConfiguration('contact_sensor_topic')
    force_threshold = LaunchConfiguration('contact_force_threshold')
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
        DeclareLaunchArgument('use_rviz', default_value='true'),
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
                str(share / 'config' / 'config.yaml'),
                {
                    'mesh_directory': str(share / 'meshes' / 'intrinsic_test_v4_surf.STL'),
                    'contact_sensor_topic': sensor_topic,
                    'contact_force_threshold': ParameterValue(force_threshold, value_type=float),
                },
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'urdf.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
