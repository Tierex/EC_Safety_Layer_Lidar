import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    #Get package directorys
    pkg_dir = get_package_share_directory('lidar_launch')


  #Velodyne
    velodyne_launch_dir = os.path.join(get_package_share_directory('velodyne'), 'launch')
    
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velodyne_launch_dir, 'velodyne-all-nodes-VLP16-launch.py')
        )
    )

  #Object detection
    obj_det_pkg_dir = get_package_share_directory('lidar_object_detection')
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(obj_det_pkg_dir, 'launch', 'minimal_lidar_safety.launch.py')
        )
    )

  #RVIZ2
   #Truck model
    urdf_file = os.path.join(pkg_dir, 'config', 'truck.urdf')
    with open(urdf_file, 'r') as infp:
        truck_desc = infp.read()

    truck_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='truck_description_publisher',
        parameters=[{'robot_description': truck_desc}],
    )

   #Safety overlay node
    safety_overlay_node = Node(
        package='lidar_launch',
        executable='safety_overlay_node',
        name='safety_overlay_node'
    )

   #Rviz node
    rviz_config_dir = os.path.join(pkg_dir,'config', 'safety_layer_V1.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        velodyne_launch,
        object_detection_launch,
        truck_description_publisher,
        safety_overlay_node,
        rviz_node
    ])