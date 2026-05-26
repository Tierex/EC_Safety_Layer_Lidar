import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                get_package_share_directory('lidar_object_detection'),
                'config',
                'lidar_pipeline_tuning.yaml'
            ),
            description='Path to the lidar_object_detection YAML parameter file',
        ),

        Node(
            package='lidar_object_detection',
            executable='lidar_detector',
            name='lidar_detector',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='lidar_object_detection',
            executable='object_tracker',
            name='object_tracker',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='lidar_object_detection',
            executable='ego_speed_constant_node',
            name='ego_speed_constant_node',
            output='screen',
            parameters=[config_file]
        ),

        Node(
            package='lidar_object_detection',
            executable='safety_supervisor',
            name='safety_supervisor',
            output='screen',
            parameters=[config_file]
        ),
    ])