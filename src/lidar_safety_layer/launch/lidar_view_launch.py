import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_safety_layer')

    #Velodyne
    velodyne_launch_dir = os.path.join(get_package_share_directory('velodyne'), 'launch')
    
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_launch_dir, 'velodyne-all-nodes-VLP16-launch.py')
        )
    )

    #URDF (truck model)
    urdf_file = os.path.join(pkg_share, 'config', 'truck.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )


    #RVIZ
    rviz_config_dir = os.path.join(pkg_share,'config', 'velodyne_default.rviz')

    # start RviZ with the saved config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        velodyne_launch,
       # filter_node,
        robot_state_publisher_node,
        rviz_node
    ])