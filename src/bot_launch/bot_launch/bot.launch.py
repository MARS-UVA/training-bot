from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

#creats a file path that works on all operating systems
rviz_config_file = os.path.join(
    get_package_share_directory('bot_launch'), 
    'rviz2',
    'nav2_default_view.rviz')

#creates a node that launches rviz with our config file for later use
rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
)

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'), #needed for the turtlebot in gazebo
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
                        'use_sim_time': 'true',
                        'gui' : '0',
                        'publish_motors' : 'false',
                        'turtlebot3_model': 'waffle'
                    }.items()
                ),
        IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('zed_wrapper'),
                        'launch',
                        'zed_camera.launch.py'
                    ]),
                    launch_arguments={
                        'camera_model' : 'zed2i',
                        'publish_map_tf' : 'true',
                        'res' : 'HD720',
                        'depth_mode' : 'PERFORMANCE',
                        'camera_fps' : '30',
                        'colorize_depth' : 'false',
                        'color_enhancing' : 'false',
                        'odom_topic' : '',
                        'pos_tracking_enabled' : 'true',
                        'pos_tracking_mode' : 'GEN_1',
                        'area_memory' : 'false',
                        'sensors_image_sync' : 'true'
                    }.items()
                ),
        IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare('rtabmap_launch'),
                        'launch',
                        'rtabmap.launch.py'
                    ]),
                    launch_arguments={
                        'rgbd': 'true',
                        'camera_namespace': '/zed/zed_node',
                        'frame_id': 'zed_camera_link',
                        'visual_odometry': 'true',
                        'rgb_topic': '/zed/zed_node/rgb/color/rect/image',
                        'depth_topic': '/zed/zed_node/depth/depth_registered',
                        'camera_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
                        'approx_sync': 'false',
                        'rgbd_sync': 'true',
                        'approx_rgbd_sync': 'true',
                        'rviz': 'true',
                        'database_path': '/tmp/rtabmap.db',
                        'rtabmap_args': '--delete_db_on_startup true '
                                        '--RGBD/DepthAsIntensity true '
                                        '--Vis/MaxFeatures 1000 '
                                        '--Vis/FeatureType 6'
                    }.items()
                ),
        # IncludeLaunchDescription(
        #         PathJoinSubstitution([
        #             FindPackageShare('slam_toolbox'),
        #             'launch',
        #             'online_async_launch.py',
        #         ]),
        #         launch_arguments={
        #             'resolution': '0.0005',
        #         }.items()
        # ),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'slam': 'true'
            }.items()
        )
        TimerAction(
            period=5.0,      # Delay in seconds
            actions=[rviz_node]
        )
    ])
