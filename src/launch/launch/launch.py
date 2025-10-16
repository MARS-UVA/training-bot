from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop',
            namespace='bot',
            executable='teleop',
            name='teleop'
        ),
        Node(
            package='teleop',
            namespace='bot',
            executable='auto',
            name='auto'
        ),
        Node(
            package='serial_comms',
            executable='serial',
            name='serial',
            namespace = 'bot'
        ),
        Node(
            package='teleop',
            executable='motor_command_reader',
            name='motor_command_reader',
            output='screen'
        )
    ])
