from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_odom = LaunchConfiguration('use_odom')
    use_imu = LaunchConfiguration('use_imu')
    use_lidar_subscriber = LaunchConfiguration('use_lidar_subscriber')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_odom',
            default_value='false',
            description='If true: use /odom via ego_speed_from_odom_node. If false: use ego_speed_constant_node.'
        ),

        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='If true: start ego_yaw_from_imu_node.'
        ),

        DeclareLaunchArgument(
            'use_lidar_subscriber',
            default_value='false',
            description='If true: start optional lidar_subscriber debug node.'
        ),

        Node(
            package='lidar_object_detection',
            executable='lidar_detector',
            name='lidar_detector',
            output='screen',
            parameters=[{
                'debug': False,
                'debug_every_n_frames': 10,
                'debug_objects': True,
                'publish_markers': True,
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='object_tracker',
            name='object_tracker',
            output='screen',
            parameters=[{
                'debug': False,
                'publish_markers': True,
                'frame_id': 'velodyne',
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='ego_speed_constant_node',
            name='ego_speed_constant_node',
            output='screen',
            condition=UnlessCondition(use_odom),
            parameters=[{
                'speed_mps': 1.3889,
                'publish_rate_hz': 50.0,
                'ego_speed_topic': '/ego_speed',
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='ego_speed_from_odom_node',
            name='ego_speed_from_odom_node',
            output='screen',
            condition=IfCondition(use_odom),
            parameters=[{
                'odom_topic': '/odom',
                'ego_speed_topic': '/ego_speed',
                'use_speed_magnitude': False,
                'smoothing_alpha': 0.35,
                'max_speed_mps': 5.0,
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='ego_yaw_from_imu_node',
            name='ego_yaw_from_imu_node',
            output='screen',
            condition=IfCondition(use_imu),
            parameters=[{
                'imu_topic': '/imu/data',
                'ego_yaw_rate_topic': '/ego_yaw_rate',
                'smoothing_alpha': 0.35,
                'max_yaw_rate_rps': 10.0,
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            parameters=[{
                'debug': True,
                'debug_every_n_frames': 1,
                'input_topic': '/tracked_objects',
                'output_topic': '/safety_signal',
                'ego_speed_topic': '/ego_speed',
                'corridor_half_width': 1.0,
                'emergency_distance': 1.5,
                'hazard_distance': 2.5,
                'emergency_ttc': 0.6,
                'hazard_ttc': 1.5,
                'stale_signal': 0,
                'sensor_offset_x': -0.5,
                'sensor_offset_y': 0.0,
                'sensor_offset_z': 1.8,
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{
                'input_topic': '/safety_signal',
                'output_topic': '/cmd_vel',
                'speed_free_mps': 1.3889,
                'speed_hazard_mps': 0.50,
                'speed_emergency_mps': 0.0,
                'max_accel_step_mps': 0.20,
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
            parameters=[{
                'tracked_objects_topic': '/tracked_objects',
                'safety_signal_topic': '/safety_signal',
                'ego_speed_topic': '/ego_speed',
                'print_rate_hz': 2.0,
            }]
        ),

        Node(
            package='lidar_object_detection',
            executable='lidar_subscriber',
            name='lidar_subscriber',
            output='screen',
            condition=IfCondition(use_lidar_subscriber),
        ),
    ])
