from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    hardware_id = LaunchConfiguration('hardware_id', default='11162')
    circuit_id = LaunchConfiguration('circuit_id', default='0')
    bit_rate = LaunchConfiguration('bit_rate', default='500000')

    kvaser_bridge_node = Node(
        package='kvaser_interface',
        executable='kvaser_can_bridge',
        namespace='can',
        output='screen',
        parameters=[
            {
                'hardware_id': hardware_id,
                'circuit_id': circuit_id,
                'bit_rate': bit_rate
            }
        ]
    )

    nissan_can_driver = Node(
        package='nissan_can_driver',
        executable='nissan_vehicle_info',
        namespace='can',
        output='screen'
    )

    return LaunchDescription([
        kvaser_bridge_node,
        nissan_can_driver
    ])