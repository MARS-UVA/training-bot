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
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam',
            namespace = 'bot'
        ),
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_drive',
            name='turtlebot3_drive',
            namespace = 'bot'
        ),
        Node(
            package='serial_comms',
            executable='serial',
            name='serial',
            namespace = 'bot'
        ),
        Node(
            package='teleop',
            executable='motor_command_reader.py',
            name='motor_command_reader',
            output='screen',
            namespace='bot'
        )
    ])
