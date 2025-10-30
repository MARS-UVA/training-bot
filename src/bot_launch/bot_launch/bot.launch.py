from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='teleop',
        #     namespace='bot',
        #     executable='teleop',
        #     name='teleop'
        # ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam',
            namespace = 'bot'
        ),
        # Node(
        #     package='serial_comms',
        #     executable='serial',
        #     name='serial',
        #     namespace = 'bot'
        # ),
        Node(
            package='teleop',
            executable='motor_command_reader',
            name='motor_command_reader',
            output='screen',
            namespace='bot'
        ),
        IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('turtlebot3_gazebo'),
                        'launch',
                        'turtlebot3_world.launch.py'
                    ]),
                    launch_arguments={
                        'use_sim_time': 'True'
                    }.items()
                ),
        IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
        )
    ])