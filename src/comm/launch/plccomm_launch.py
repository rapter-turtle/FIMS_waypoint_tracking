from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='comm',
            executable='plccomm',
            name='plccomm',
            output='screen',
            emulate_tty=True,
        ),
    ])