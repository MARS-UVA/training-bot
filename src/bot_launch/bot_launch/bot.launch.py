from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os

rviz_config_file = os.path.join(
    get_package_share_directory('bot_launch'), 
    'rviz2',
    'nav2_default_view.rviz')

rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
)


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
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
                    'online_async_launch.py',
                ]),
                launch_arguments={
                    'resolution': '0.0005',
                }.items()
        ),
        IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
        ),
        TimerAction(
            period=5.0,      # Delay in seconds
            actions=[rviz_node]
        )
    ])