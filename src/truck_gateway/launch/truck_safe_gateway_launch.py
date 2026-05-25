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
            'autorepeat_rate': 20.0,
            ' coalesce_interval_ms' : 2
        }]
    )

    # Xbox Translator
    translator_node = Node(
        package='truck_gateway',
        executable='xbox_translator_exe',
        name='xbox_translator',
        remappings=[('truck_cmd', 'joy_truck_cmd')]
    )

    # Truck Cmd Mux
    mux_node = Node(
        package='truck_gateway',
        executable='truck_cmd_mux_exe',
        name='truck_cmd_mux'
    )

    # CAN Gateway
    gateway_node = Node(
        package='truck_gateway',
        executable='truck_can_exe',
        name='truck_can_gateway',
        parameters=[{"can_channel": "can0"}]
    )

    return LaunchDescription([
        joy_node,
        translator_node,
        mux_node,
        gateway_node
    ])