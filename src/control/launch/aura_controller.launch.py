from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='aura_wpt_model_carrot_pid_thrust',
            name='aura_wpt_controller',
            output='screen',
            emulate_tty=True,
        )
    ])