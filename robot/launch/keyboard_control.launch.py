#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # 모터 제어 노드 (motor.py)
    motor_node = Node(
        package='robot',
        executable='motor',          # setup.py의 console_scripts 이름
        name='motor',
        output='screen',
        parameters=[
            {"control_hz": 50.0},
            {"debug": True},
        ]
    )

    # 키보드 teleop 노드 (teleop.py)
    teleop_node = Node(
        package='robot',
        executable='teleop',         # setup.py의 console_scripts 이름
        name='teleop',
        output='screen',
        parameters=[
            {"linear_speed": 0.20},
            {"angular_speed": 0.80},
        ]
    )

    # x-IMU3 퍼블리셔 노드 (ximu3_publisher.py)
    imu_node = Node(
        package='robot',
        executable='ximu3_publisher',
        name='ximu3_publisher',
        output='screen',
        parameters=[
            {"serial_port": "/dev/ttyACM0"},
            {"serial_baud": 115200},
            {"publish_tf": False},
            {"debug": False},
        ]
    )

    return LaunchDescription([
        motor_node,
        teleop_node,
        imu_node,
    ])

