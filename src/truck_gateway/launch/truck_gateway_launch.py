from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Joy Node (Standard driver)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0, #ros2 run joy joy_enumerate_devices
            'deadzone': 0.1,
            'autorepeat_rate': 0.0,
            ' coalesce_interval_ms' : 2
        }]
    )

    # Xbox Translator
    translator_node = Node(
        package='truck_gateway',
        executable='xbox_translator_exe',
        name='xbox_translator'
    )

    # CAN Gateway
    gateway_node = Node(
        package='truck_gateway',
        executable='truck_can_exe',
        name='truck_can_gateway'
    )

    return LaunchDescription([
        joy_node,
        translator_node,
        gateway_node
    ])