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
    robot_description = (share / 'urdf' / 'react_intrinsic_JSshape.urdf').read_text(encoding='utf-8')
    visual_mesh_uri = (share / 'meshes' / 'JSshape.stl').as_uri()
    robot_description = robot_description.replace(
        'package://REACT_intrinsic/meshes/JSshape.stl',
        visual_mesh_uri,
    )

    contact_force_threshold = LaunchConfiguration('contact_force_threshold')
    contact_frame_1 = LaunchConfiguration('contact_frame_1')
    contact_frame_2 = LaunchConfiguration('contact_frame_2')
    wrench_tf_timeout = LaunchConfiguration('wrench_tf_timeout_sec')

    sensor_topic_1 = LaunchConfiguration('contact_sensor_topic_1')
    sensor_topic_2 = LaunchConfiguration('contact_sensor_topic_2')
    wrench_source_frame_1 = LaunchConfiguration('wrench_source_frame_1')
    wrench_source_frame_2 = LaunchConfiguration('wrench_source_frame_2')

    proximity_raw_topic_1 = LaunchConfiguration('proximity_raw_topic_1')
    proximity_raw_topic_2 = LaunchConfiguration('proximity_raw_topic_2')
    proximity_tof_topic_1 = LaunchConfiguration('proximity_tof_topic_1')
    proximity_tof_topic_2 = LaunchConfiguration('proximity_tof_topic_2')
    proximity_sigma_topic_1 = LaunchConfiguration('proximity_sigma_topic_1')
    proximity_sigma_topic_2 = LaunchConfiguration('proximity_sigma_topic_2')
    proximity_cap_topic_1 = LaunchConfiguration('proximity_cap_topic_1')
    proximity_cap_topic_2 = LaunchConfiguration('proximity_cap_topic_2')
    proximity_distance_topic_1 = LaunchConfiguration('proximity_distance_topic_1')
    proximity_distance_topic_2 = LaunchConfiguration('proximity_distance_topic_2')
    enable_data_logging = LaunchConfiguration('enable_data_logging')
    sensor_log_directory = LaunchConfiguration('sensor_log_directory')

    calibration_x_1 = LaunchConfiguration('wrench_calibration_x_1')
    calibration_y_1 = LaunchConfiguration('wrench_calibration_y_1')
    calibration_z_1 = LaunchConfiguration('wrench_calibration_z_1')
    calibration_roll_1 = LaunchConfiguration('wrench_calibration_roll_1')
    calibration_pitch_1 = LaunchConfiguration('wrench_calibration_pitch_1')
    calibration_yaw_1 = LaunchConfiguration('wrench_calibration_yaw_1')

    calibration_x_2 = LaunchConfiguration('wrench_calibration_x_2')
    calibration_y_2 = LaunchConfiguration('wrench_calibration_y_2')
    calibration_z_2 = LaunchConfiguration('wrench_calibration_z_2')
    calibration_roll_2 = LaunchConfiguration('wrench_calibration_roll_2')
    calibration_pitch_2 = LaunchConfiguration('wrench_calibration_pitch_2')
    calibration_yaw_2 = LaunchConfiguration('wrench_calibration_yaw_2')

    use_contact_sensing = LaunchConfiguration('use_contact_sensing')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'contact_force_threshold', default_value='1.5',
            description='Minimum filtered force magnitude [N] used for contact calculation',
        ),
        DeclareLaunchArgument(
            'contact_frame_1', default_value='ur10_jsshape_supporter1',
            description='Frame containing sensor 1 contact surface and results',
        ),
        DeclareLaunchArgument(
            'contact_frame_2', default_value='ur10_jsshape_supporter2',
            description='Frame containing sensor 2 contact surface and results',
        ),
        DeclareLaunchArgument('wrench_tf_timeout_sec', default_value='0.05'),

        DeclareLaunchArgument(
            'contact_sensor_topic_1', default_value='/ur10skku/ftdata_tcp_1',
            description=(
                'Gravity-compensated TCP/local-frame wrench for the first sensor in '
                "Y2FT_AQ's can_ids list (ft_sensors.yaml) - top sensor, CAN 0x01/0x02"
            ),
        ),
        DeclareLaunchArgument(
            'contact_sensor_topic_2', default_value='/ur10skku/ftdata_tcp_2',
            description=(
                'Gravity-compensated TCP/local-frame wrench for the second sensor in '
                "Y2FT_AQ's can_ids list (ft_sensors.yaml) - bottom sensor, CAN 0x03/0x04"
            ),
        ),
        DeclareLaunchArgument(
            'proximity_raw_topic_1', default_value='/filter_raw1',
            description='Proximity sensor 1 filtered capacitive raw-count topic',
        ),
        DeclareLaunchArgument(
            'proximity_raw_topic_2', default_value='/filter_raw2',
            description='Proximity sensor 2 filtered capacitive raw-count topic',
        ),
        DeclareLaunchArgument(
            'proximity_tof_topic_1', default_value='/tof_distance1',
            description='Proximity sensor 1 ToF distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_tof_topic_2', default_value='/tof_distance2',
            description='Proximity sensor 2 ToF distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_sigma_topic_1', default_value='/cap_sigma1',
            description='Proximity sensor 1 capacitive sigma topic',
        ),
        DeclareLaunchArgument(
            'proximity_sigma_topic_2', default_value='/cap_sigma2',
            description='Proximity sensor 2 capacitive sigma topic',
        ),
        DeclareLaunchArgument(
            'proximity_cap_topic_1', default_value='/cap_distance1',
            description='Proximity sensor 1 capacitance-only distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_cap_topic_2', default_value='/cap_distance2',
            description='Proximity sensor 2 capacitance-only distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_distance_topic_1', default_value='/fused_distance1',
            description='Proximity sensor 1 fused distance topic',
        ),
        DeclareLaunchArgument(
            'proximity_distance_topic_2', default_value='/fused_distance2',
            description='Proximity sensor 2 fused distance topic',
        ),
        DeclareLaunchArgument(
            'enable_data_logging', default_value='false',
            description='Automatically save FT and proximity topic samples as text files',
        ),
        DeclareLaunchArgument(
            'sensor_log_directory',
            default_value=str(
                Path.home() / 'ros2_ws' / 'src' / 'REACT_intrinsic' / 'sensor_data_logs'
            ),
            description='Root directory for timestamped FT and proximity log sessions',
        ),
        DeclareLaunchArgument('wrench_source_frame_1', default_value='ur10_jsshape_wrench1'),
        DeclareLaunchArgument('wrench_source_frame_2', default_value='ur10_jsshape_wrench2'),

        DeclareLaunchArgument('wrench_calibration_x_1', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_y_1', default_value='0.0'),
        # Initial origin assumption: each wrench is measured at its
        # supporter's CAD origin. Replace these translations with the measured
        # F/T sensing-origin offsets later.
        DeclareLaunchArgument('wrench_calibration_z_1', default_value='0.0'),
        # Interpret /ftdata_tcp_1 in the validated sensor-1 wrench axes while
        # keeping the result attached to the moving supporter frame.
        DeclareLaunchArgument('wrench_calibration_roll_1', default_value='3.14159265359'),
        DeclareLaunchArgument('wrench_calibration_pitch_1', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_yaw_1', default_value='3.14159265359'),

        DeclareLaunchArgument('wrench_calibration_x_2', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_y_2', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_z_2', default_value='0.0'),
        # Sensor 2 uses the corresponding validated local-wrench orientation.
        DeclareLaunchArgument('wrench_calibration_roll_2', default_value='3.14159265359'),
        DeclareLaunchArgument('wrench_calibration_pitch_2', default_value='0.0'),
        DeclareLaunchArgument('wrench_calibration_yaw_2', default_value='3.14159265359'),

        DeclareLaunchArgument(
            'use_contact_sensing', default_value='true',
            description='Start both force/torque contact-sensing (main_ifs) nodes',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='jsshape_wrench_calibration_tf_1',
            arguments=[
                '--x', calibration_x_1,
                '--y', calibration_y_1,
                '--z', calibration_z_1,
                '--roll', calibration_roll_1,
                '--pitch', calibration_pitch_1,
                '--yaw', calibration_yaw_1,
                '--frame-id', contact_frame_1,
                '--child-frame-id', wrench_source_frame_1,
            ],
            condition=IfCondition(use_contact_sensing),
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='jsshape_wrench_calibration_tf_2',
            arguments=[
                '--x', calibration_x_2,
                '--y', calibration_y_2,
                '--z', calibration_z_2,
                '--roll', calibration_roll_2,
                '--pitch', calibration_pitch_2,
                '--yaw', calibration_yaw_2,
                '--frame-id', contact_frame_2,
                '--child-frame-id', wrench_source_frame_2,
            ],
            condition=IfCondition(use_contact_sensing),
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='jsshape_visualization',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            output='screen',
        ),

        # FT sensors are started separately, same as react_intrinsic_Lshape.launch.py:
        #   ros2 launch Y2FT_AQ ft_get_main.launch.py
        # Contact sensing uses the 100 Hz gravity-compensated local-frame topics.
        # FTGetMain reads shoulder_lift + elbow directly from the UR joint-state
        # topic, so ur_pose_bridge/currentP is not part of this pipeline.
        # /ftdata_N carries the same local-frame values at the full publish rate.
        Node(
            package='REACT_intrinsic',
            executable='main_ifs',
            name='jsshape_sensor1_contactsensing',
            parameters=[
                str(share / 'config' / 'JSshape_config.yaml'),
                {
                    'mesh_directory': str(share / 'meshes' / 'JSshape_surface.stl'),
                    'contact_sensor_topic': sensor_topic_1,
                    'contact_force_threshold': ParameterValue(contact_force_threshold, value_type=float),
                    'use_wrench_tf': True,
                    'contact_frame': contact_frame_1,
                    'wrench_source_frame': wrench_source_frame_1,
                    'wrench_tf_timeout_sec': ParameterValue(wrench_tf_timeout, value_type=float),
                },
            ],
            remappings=[
                ('/contactsensing/cp', '/sensor1/contactsensing/cp'),
                ('/contactsensing/force_arrow', '/sensor1/contactsensing/force_arrow'),
                ('/contactsensing/force_magnitude', '/sensor1/contactsensing/force_magnitude'),
                ('/contactsensing/wrench_corrected', '/sensor1/contactsensing/wrench_corrected'),
                ('/contactsensing/contact_state', '/sensor1/contactsensing/contact_state'),
                ('/mesh_pc', '/sensor1/mesh_pc'),
            ],
            condition=IfCondition(use_contact_sensing),
            output='screen',
        ),

        Node(
            package='REACT_intrinsic',
            executable='main_ifs',
            name='jsshape_sensor2_contactsensing',
            parameters=[
                str(share / 'config' / 'JSshape_config.yaml'),
                {
                    'mesh_directory': str(share / 'meshes' / 'JSshape_surface.stl'),
                    'contact_sensor_topic': sensor_topic_2,
                    'contact_force_threshold': ParameterValue(contact_force_threshold, value_type=float),
                    'use_wrench_tf': True,
                    'contact_frame': contact_frame_2,
                    'wrench_source_frame': wrench_source_frame_2,
                    'wrench_tf_timeout_sec': ParameterValue(wrench_tf_timeout, value_type=float),
                },
            ],
            remappings=[
                ('/contactsensing/cp', '/sensor2/contactsensing/cp'),
                ('/contactsensing/force_arrow', '/sensor2/contactsensing/force_arrow'),
                ('/contactsensing/force_magnitude', '/sensor2/contactsensing/force_magnitude'),
                ('/contactsensing/wrench_corrected', '/sensor2/contactsensing/wrench_corrected'),
                ('/contactsensing/contact_state', '/sensor2/contactsensing/contact_state'),
                ('/mesh_pc', '/sensor2/mesh_pc'),
            ],
            condition=IfCondition(use_contact_sensing),
            output='screen',
        ),

        Node(
            package='REACT_intrinsic',
            executable='sensor_data_logger',
            name='jsshape_sensor_data_logger',
            parameters=[{
                'ft_topic1': sensor_topic_1,
                'ft_topic2': sensor_topic_2,
                'raw_topic1': proximity_raw_topic_1,
                'raw_topic2': proximity_raw_topic_2,
                'tof_topic1': proximity_tof_topic_1,
                'tof_topic2': proximity_tof_topic_2,
                'sigma_topic1': proximity_sigma_topic_1,
                'sigma_topic2': proximity_sigma_topic_2,
                'cap_topic1': proximity_cap_topic_1,
                'cap_topic2': proximity_cap_topic_2,
                'fused_topic1': proximity_distance_topic_1,
                'fused_topic2': proximity_distance_topic_2,
                'log_root_directory': sensor_log_directory,
            }],
            condition=IfCondition(enable_data_logging),
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'JSshape.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
