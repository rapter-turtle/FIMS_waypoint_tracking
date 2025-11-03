from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Launch Arguments
    use_plccomm = LaunchConfiguration('use_plccomm')
    use_mavlink = LaunchConfiguration('use_mavlink')
    use_actuator_publisher = LaunchConfiguration('use_actuator_publisher')

    declare_use_plccomm = DeclareLaunchArgument(
        'use_plccomm',
        default_value='true',
        description='Whether to launch the plccomm node'
    )

    declare_use_mavlink = DeclareLaunchArgument(
        'use_mavlink',
        default_value='true',
        description='Whether to launch the bada_mavlink node'
    )
    
    declare_use_actuator_publisher = DeclareLaunchArgument(
        'use_actuator_publisher',
        default_value='true',
        description='Whether to launch the ActuatorPublisher node'
    )

    # Define the nodes
    plccomm_node = Node(
        package='comm',
        executable='plccomm',
        name='plccomm',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_plccomm')),
    )

    mavlink_node = Node(
        package='comm',
        executable='bada_mavlink',
        name='bada_mavlink',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_mavlink')),
    )
    
    actuator_publisher_node = Node(
        package='control',
        executable='aura_wpt_model_carrot_pid_thrust',
        name='actuator_publisher',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_actuator_publisher')),
    )

    # Return the LaunchDescription
    return LaunchDescription([
        declare_use_plccomm,
        declare_use_mavlink,
        declare_use_actuator_publisher,
        plccomm_node,
        actuator_publisher_node,
        mavlink_node,
    ])