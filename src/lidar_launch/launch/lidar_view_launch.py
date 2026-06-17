import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_launch')

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
        truck_desc = infp.read()

    # 3. truck model publisher (Robot State Publisher Node)
    truck_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='truck_description_publisher',
        parameters=[{'robot_description': truck_desc}],
    )

    #RVIZ
    rviz_config_dir = os.path.join(pkg_share,'config', 'velodyne_default_V2.rviz')

    # start RviZ with the saved config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        velodyne_launch,
        truck_description_publisher,
        rviz_node
    ])