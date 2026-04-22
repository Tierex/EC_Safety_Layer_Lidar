import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path to the official Velodyne launch file
    velodyne_launch_dir = os.path.join(get_package_share_directory('velodyne'), 'launch')
    
    # Start default velodyne launch
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_launch_dir, 'velodyne-all-nodes-VLP16-launch.py')
        )
    )


    rviz_config_dir = os.path.join(get_package_share_directory('lidar_safety_layer'),'config',
                                'velodyne_default.rviz')

    # start RviZ with the saved config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        velodyne_launch,
        rviz_node
    ])