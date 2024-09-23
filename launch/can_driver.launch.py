from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    kvaser_bridge_info = Node(
        package='kvaser_interface',
        executable='kvaser_can_bridge',
        namespace='can_info',
        output='screen',
        parameters=[
            {
                'hardware_id': '11162',
                'circuit_id':  '0',
                'bit_rate':    '500000'
            }
        ]
    )

    kvaser_bridge_control = Node(
        package='kvaser_interface',
        executable='kvaser_can_bridge',
        namespace='can_ctrl',
        output='screen',
        parameters=[
            {
                'hardware_id': '11162',
                'circuit_id':  '1',
                'bit_rate':    '500000'
            }
        ]
    )

    nissan_vehicle_info = Node(
        package='nissan_can_driver',
        executable='nissan_vehicle_info',
        namespace='can_info',
        output='screen'
    )

    nissan_vehicle_control = Node(
        package='nissan_can_driver',
        executable='nissan_vehicle_control',
        namespace='can_ctrl',
        output='screen'
    )

    return LaunchDescription([
        kvaser_bridge_info,
        kvaser_bridge_control,
        nissan_vehicle_info,
        nissan_vehicle_control
    ])