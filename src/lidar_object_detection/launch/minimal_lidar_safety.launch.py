from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

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
            parameters=[{
                'speed_mps': 1.3889,
                'publish_rate_hz': 50.0,
                'ego_speed_topic': '/ego_speed',
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
    ])
