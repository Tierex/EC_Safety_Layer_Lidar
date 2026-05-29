import os

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    launch_actions = []

    # =========================================================
    # Package directories
    # =========================================================
    pkg_dir = get_package_share_directory('lidar_launch')
    obj_det_pkg_dir = get_package_share_directory('lidar_object_detection')

    # =========================================================
    # Velodyne driver
    # =========================================================
    try:
        velodyne_launch_dir = os.path.join(
            get_package_share_directory('velodyne'),
            'launch'
        )

        velodyne_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    velodyne_launch_dir,
                    'velodyne-all-nodes-VLP16-launch.py'
                )
            )
        )

        launch_actions.append(velodyne_launch)

    except PackageNotFoundError:
        launch_actions.append(
            LogInfo(
                msg='velodyne package not found; skipping velodyne launch.'
            )
        )

    # =========================================================
    # Object detection + tracker + safety supervisor
    # =========================================================
    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                obj_det_pkg_dir,
                'launch',
                'minimal_lidar_safety.launch.py'
            )
        )
    )

    launch_actions.append(object_detection_launch)

    # =========================================================
    # Truck model / robot_state_publisher
    # =========================================================
    urdf_file = os.path.join(
        pkg_dir,
        'config',
        'truck.urdf'
    )

    with open(urdf_file, 'r') as infp:
        truck_desc = infp.read()

    truck_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='truck_description_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': truck_desc
            }
        ],
    )

    launch_actions.append(truck_description_publisher)

    # =========================================================
    # Safety overlay node
    #
    # BELANGRIJK:
    # Deze staat uit omdat safety_supervisor nu zelf
    # /safety_zones_array publiceert.
    #
    # Als safety_overlay_node ook markers publiceert op
    # /safety_zones, dan krijg je opnieuw topic-conflicten.
    # =========================================================

    # safety_overlay_node = Node(
    #     package='lidar_launch',
    #     executable='safety_overlay_node',
    #     name='safety_overlay_node',
    #     output='screen'
    # )
    #
    # launch_actions.append(safety_overlay_node)

    # =========================================================
    # RViz2
    # =========================================================
    rviz_config_file = os.path.join(
        pkg_dir,
        'config',
        'velodyne_default_V4.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config_file
        ]
    )

    launch_actions.append(rviz_node)

    # =========================================================
    # Return launch description
    # =========================================================
    return LaunchDescription(launch_actions)