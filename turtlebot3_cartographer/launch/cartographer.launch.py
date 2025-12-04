# cartographer.launch.py (내 로봇용 수정 버전)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')

    # === Cartographer 설정 파일 경로 ===
    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(turtlebot3_cartographer_prefix, 'config')
    )

    # ★ 여기서 네가 만든 lua 이름으로 바꿔줌
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='turtlebot3_lds_2d.lua'   # 또는 네가 저장한 lua 파일 이름
    )

    # OccupancyGrid 설정
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_cartographer'),
        'rviz',
        'tb3_cartographer.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config directory for Cartographer'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for Cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # === Cartographer SLAM 노드 ===
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            # /scan, /odom 이름이 기본과 같으면 remapping 필요 없음
            # 필요하면 예시:
            # remappings=[('scan', '/scan'),
            #             ('odom', '/odom')],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
        ),

        # === OccupancyGrid( /map ) 퍼블리셔 ===
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/occupancy_grid.launch.py']
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items(),
        ),

        # === RViz ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
