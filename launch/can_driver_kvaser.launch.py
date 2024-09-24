from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    autoware_control_input_arg = DeclareLaunchArgument(
        'autoware_control_input',
        default_value='true',
        description='Enable this argument to use the Autoware control input topics')
    
    kvaser_hardware_id_arg = DeclareLaunchArgument(
        'kvaser_hardware_id',
        default_value='11162',
        description='Kvaser hardware ID')
    
    kvaser_bridge_info = Node(
        package='kvaser_interface',
        executable='kvaser_can_bridge',
        name='kvaser_can_bridge_info',
        namespace='can_info',
        output='screen',
        parameters=[
            {
                'hardware_id': LaunchConfiguration('kvaser_hardware_id'),
                'circuit_id':  0,
                'bit_rate':    500000
            }
        ]
    )

    kvaser_bridge_control = Node(
        package='kvaser_interface',
        executable='kvaser_can_bridge',
        name='kvaser_can_bridge_control',
        namespace='ctrl',
        output='screen',
        parameters=[
            {
                'hardware_id': LaunchConfiguration('kvaser_hardware_id'),
                'circuit_id':  1,
                'bit_rate':    500000
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
        namespace='ctrl',
        output='screen',
        parameters=[{
            'autoware_control_input': LaunchConfiguration('autoware_control_input')
        }]
    )

    return LaunchDescription([
        autoware_control_input_arg,
        kvaser_hardware_id_arg,

        kvaser_bridge_info,
        kvaser_bridge_control,
        nissan_vehicle_info,
        nissan_vehicle_control
    ])