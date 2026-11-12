from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('REACT_intrinsic'))
    robot_description = (share / 'urdf' / 'NRS_spindle.urdf').read_text(encoding='utf-8')

    sensor_topic = LaunchConfiguration('contact_sensor_topic')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'contact_sensor_topic',
            default_value='/ur10skku/ftdata',
            description='geometry_msgs/msg/WrenchStamped input topic',
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
                    'mesh_directory': str(share / 'meshes' / 'NRS_spindle_surf.STL'),
                    'contact_sensor_topic': sensor_topic,
                },
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'NRS_spindle.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
