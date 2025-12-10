# navigation2.launch.py (fixed & slam support)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam = LaunchConfiguration('slam', default='False')

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('navigation2'),
            'map',
            'map.yaml'))

    # 네 로봇용 파라미터 파일 이름
    param_file_name = 'my_robot.yaml'

    # Humble이면 param/humble/my_robot.yaml, 아니면 param/my_robot.yaml
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('navigation2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('navigation2'),
                'param',
                param_file_name))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('navigation2'),
        'rviz',
        'tb3_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'slam',
            default_value='False',
            description='Use SLAM (True) or map server (False)'),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam': slam,
                'map': map_dir,
                'params_file': param_dir,
            }.items(),
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])

