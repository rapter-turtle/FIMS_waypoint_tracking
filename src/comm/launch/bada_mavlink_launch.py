from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    connection_type_arg = DeclareLaunchArgument(
        'connection_type',
        default_value='udp',
        description='Connection type: udp or serial'
    )
    
    udp_local_ip_arg = DeclareLaunchArgument(
        'udp_local_ip',
        default_value='127.0.0.1',
        description='Local IP for UDP connection'
    )
    
    udp_local_port_arg = DeclareLaunchArgument(
        'udp_local_port',
        default_value='15555',
        description='Local port for UDP connection'
    )
    
    udp_remote_ip_arg = DeclareLaunchArgument(
        'udp_remote_ip',
        default_value='192.168.0.1',
        description='Remote IP for UDP connection'
    )
    
    udp_remote_port_arg = DeclareLaunchArgument(
        'udp_remote_port',
        default_value='15550',
        description='Remote port for UDP connection'
    )
    
    serial_device_arg = DeclareLaunchArgument(
        'serial_device',
        default_value='/dev/ttyUSB0',
        description='Serial device for MAVLink connection'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baudrate for serial connection'
    )
    
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10',
        description='Update rate in Hz'
    )
    
    # 노드 정의
    mavlink_node = Node(
        package='comm',
        executable='bada_mavlink',
        name='bada_mavlink',
        output='screen',
        parameters=[{
            'connection_type': LaunchConfiguration('connection_type'),
            'udp_local_ip': LaunchConfiguration('udp_local_ip'),
            'udp_local_port': LaunchConfiguration('udp_local_port'),
            'udp_remote_ip': LaunchConfiguration('udp_remote_ip'),
            'udp_remote_port': LaunchConfiguration('udp_remote_port'),
            'serial_device': LaunchConfiguration('serial_device'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'update_rate': LaunchConfiguration('update_rate'),
        }],
    )

    return LaunchDescription([
        connection_type_arg,
        udp_local_ip_arg,
        udp_local_port_arg,
        udp_remote_ip_arg,
        udp_remote_port_arg,
        serial_device_arg,
        serial_baudrate_arg,
        update_rate_arg,
        mavlink_node,
    ])