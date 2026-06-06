import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_object_detection')

    config_file = os.path.join(
        pkg_dir,
        'config',
        'safety_supervisor.yaml'
    )

    lidar_detector = Node(
        package='lidar_object_detection',
        executable='lidar_detector',
        name='lidar_detector',
        output='screen'
    )

    object_tracker = Node(
        package='lidar_object_detection',
        executable='object_tracker',
        name='object_tracker',
        output='screen'
    )

    ego_speed_node = Node(
        package='lidar_object_detection',
        executable='ego_speed_constant_node',
        name='ego_speed_constant_node',
        output='screen'
    )

    safety_supervisor = Node(
        package='lidar_object_detection',
        executable='safety_supervisor',
        name='safety_supervisor',
        output='screen',
        parameters=[
            config_file
        ]
    )

    return LaunchDescription([
        lidar_detector,
        object_tracker,
        ego_speed_node,
        safety_supervisor
    ])