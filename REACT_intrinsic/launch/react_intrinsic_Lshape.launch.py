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
    robot_description = (share / 'urdf' / 'react_intrinsic_Lshape.urdf').read_text(encoding='utf-8')
    visual_mesh_uri = (share / 'meshes' / 'lshape.stl').as_uri()
    robot_description = robot_description.replace(
        'package://REACT_intrinsic/meshes/lshape.STL',
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
    proximity_raw_topic1 = LaunchConfiguration('proximity_raw_topic1')
    proximity_raw_topic2 = LaunchConfiguration('proximity_raw_topic2')
    proximity_tof_topic1 = LaunchConfiguration('proximity_tof_topic1')
    proximity_tof_topic2 = LaunchConfiguration('proximity_tof_topic2')
    proximity_sigma_topic1 = LaunchConfiguration('proximity_sigma_topic1')
    proximity_sigma_topic2 = LaunchConfiguration('proximity_sigma_topic2')
    proximity_cap_topic1 = LaunchConfiguration('proximity_cap_topic1')
    proximity_cap_topic2 = LaunchConfiguration('proximity_cap_topic2')
    proximity_distance_topic1 = LaunchConfiguration('proximity_distance_topic1')
    proximity_distance_topic2 = LaunchConfiguration('proximity_distance_topic2')
    proximity_max_distance = LaunchConfiguration('proximity_max_distance')
    proximity_ema_alpha = LaunchConfiguration('proximity_ema_alpha')
    secondary_ft_sensor_topic = LaunchConfiguration('secondary_ft_sensor_topic')
    enable_data_logging = LaunchConfiguration('enable_data_logging')
    sensor_log_directory = LaunchConfiguration('sensor_log_directory')
    use_contact_sensing = LaunchConfiguration('use_contact_sensing')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'contact_sensor_topic',
            default_value='/ur10skku/ftdata_1',
            description='Primary FT topic used for L-shape contact sensing and logging',
        ),
        DeclareLaunchArgument(
            'secondary_ft_sensor_topic',
            default_value='/ur10skku/ftdata_2',
            description='Secondary FT topic saved to a log file only',
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
            default_value='lshape_wrench',
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
        DeclareLaunchArgument(
            'proximity_raw_topic1',
            default_value='/filter_raw1',
            description='Sensor 1 filtered capacitive raw-count topic',
        ),
        DeclareLaunchArgument(
            'proximity_raw_topic2',
            default_value='/filter_raw2',
            description='Sensor 2 filtered capacitive raw-count topic',
        ),
        DeclareLaunchArgument(
            'proximity_tof_topic1',
            default_value='/tof_distance1',
            description='Sensor 1 ToF distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_tof_topic2',
            default_value='/tof_distance2',
            description='Sensor 2 ToF distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_sigma_topic1',
            default_value='/cap_sigma1',
            description='Sensor 1 capacitive sigma topic from the front half of CAN 0x43',
        ),
        DeclareLaunchArgument(
            'proximity_sigma_topic2',
            default_value='/cap_sigma2',
            description='Sensor 2 capacitive sigma topic from the back half of CAN 0x43',
        ),
        DeclareLaunchArgument(
            'proximity_cap_topic1',
            default_value='/cap_distance1',
            description='Sensor 1 capacitance-only distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_cap_topic2',
            default_value='/cap_distance2',
            description='Sensor 2 capacitance-only distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_distance_topic1',
            default_value='/fused_distance1',
            description='sensor_msgs/msg/Range input mapped to L-shape proximity frame 1',
        ),
        DeclareLaunchArgument(
            'proximity_distance_topic2',
            default_value='/fused_distance2',
            description='sensor_msgs/msg/Range input mapped to L-shape proximity frame 2',
        ),
        DeclareLaunchArgument(
            'proximity_max_distance',
            default_value='0.5',
            description='Show proximity markers only at or below this distance [m]',
        ),
        DeclareLaunchArgument(
            'proximity_ema_alpha',
            default_value='1.0',
            description='EMA new-sample weight used only by proximity marker visualization',
        ),
        DeclareLaunchArgument(
            'enable_data_logging',
            default_value='true',
            description='Automatically save FT and proximity topic samples as text files',
        ),
        DeclareLaunchArgument(
            'sensor_log_directory',
            default_value=str(
                Path.home() / 'ros2_ws' / 'src' / 'REACT_intrinsic' / 'sensor_data_logs'
            ),
            description='Root directory for timestamped sensor log sessions',
        ),
        DeclareLaunchArgument(
            'use_contact_sensing',
            default_value='true',
            description='Start the force/torque contact-sensing node',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lshape_wrench_calibration_tf',
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
            condition=IfCondition(use_contact_sensing),
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
                str(share / 'config' / 'Lshape_config.yaml'),
                {
                    'mesh_directory': str(share / 'meshes' / 'lshape_surface.stl'),
                    'contact_sensor_topic': sensor_topic,
                    'contact_force_threshold': ParameterValue(force_threshold, value_type=float),
                    'use_wrench_tf': True,
                    'contact_frame': contact_frame,
                    'wrench_source_frame': wrench_source_frame,
                    'wrench_tf_timeout_sec': ParameterValue(wrench_tf_timeout, value_type=float),
                },
            ],
            condition=IfCondition(use_contact_sensing),
            output='screen',
        ),
        Node(
            package='REACT_intrinsic',
            executable='proximity_visualizer',
            name='proximity_visualizer',
            parameters=[{
                'distance_topic1': proximity_distance_topic1,
                'distance_topic2': proximity_distance_topic2,
                'sensor_frame1': 'proximity1',
                'sensor_frame2': 'proximity2',
                'marker_topic1': '/proximity_sensor/distance_marker1',
                'marker_topic2': '/proximity_sensor/distance_marker2',
                'marker_diameter': 0.04,
                'ema_alpha': ParameterValue(proximity_ema_alpha, value_type=float),
                'max_visualization_range': ParameterValue(
                    proximity_max_distance, value_type=float
                ),
            }],
            output='screen',
        ),
        Node(
            package='REACT_intrinsic',
            executable='sensor_data_logger',
            name='sensor_data_logger',
            parameters=[{
                'ft_topic1': sensor_topic,
                'ft_topic2': secondary_ft_sensor_topic,
                'raw_topic1': proximity_raw_topic1,
                'raw_topic2': proximity_raw_topic2,
                'tof_topic1': proximity_tof_topic1,
                'tof_topic2': proximity_tof_topic2,
                'sigma_topic1': proximity_sigma_topic1,
                'sigma_topic2': proximity_sigma_topic2,
                'cap_topic1': proximity_cap_topic1,
                'cap_topic2': proximity_cap_topic2,
                'fused_topic1': proximity_distance_topic1,
                'fused_topic2': proximity_distance_topic2,
                'log_root_directory': sensor_log_directory,
            }],
            condition=IfCondition(enable_data_logging),
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'Lshape.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
