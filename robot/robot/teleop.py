#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_key.py
- 키보드로 /cmd_vel 퍼블리시
- w : 전진
- s : 후진
- a : 좌회전  (왼쪽 느리게, 오른쪽 빠르게)
- d : 우회전  (왼쪽 빠르게, 오른쪽 느리게)
- space : 정지
- q : 프로그램 종료

★ 변경점
- 키를 누르고 있을 때만 해당 속도 명령을 계속 보냄
- 키 입력이 없으면 매 주기마다 '정지' 명령을 퍼블리시해서
  키보드를 떼면 바로 멈추도록 구현
"""

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


INSTRUCTIONS = """
---------------------------
키보드 조작:

    w : 전진
    s : 후진
    a : 좌회전  (왼쪽 느리게, 오른쪽 빠르게)
    d : 우회전  (왼쪽 빠르게, 오른쪽 느리게)
  space : 정지
    q : 종료

※ 키를 누르고 있는 동안만 동작합니다.
---------------------------
"""


def get_key(old_settings, timeout=0.1):
    """
    non-blocking 키 입력 함수
    - timeout 동안 입력이 없으면 None 반환
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = None
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 기본 선속도/각속도 파라미터
        self.declare_parameter('linear_speed', 0.20)   # m/s
        self.declare_parameter('angular_speed', 0.80)  # rad/s

        self.lin = float(self.get_parameter('linear_speed').value)
        self.ang = float(self.get_parameter('angular_speed').value)

        self.get_logger().info("TeleopKey node started.")
        self.get_logger().info(INSTRUCTIONS)

    def publish_stop(self):
        """정지 명령 퍼블리시"""
        twist = Twist()
        self.pub.publish(twist)
        # 너무 로그 많이 찍히면 지저분하면 여기 INFO는 주석 처리 가능
        # self.get_logger().info("publish STOP cmd_vel")

    def process_key(self, key: str):
        """
        입력된 key 에 따라 Twist 생성 후 퍼블리시
        True  -> 계속 루프
        False -> 종료
        """
        # 종료 키
        if key == 'q' or key == '\x03':  # q 또는 Ctrl+C
            self.get_logger().info("Teleop quit.")
            self.publish_stop()
            return False

        twist = Twist()

        if key == 'w':
            twist.linear.x = self.lin
            twist.angular.z = 0.0
        elif key == 's':
            twist.linear.x = -self.lin
            twist.angular.z = 0.0
        elif key == 'a':
            twist.linear.x = self.lin
            twist.angular.z = +self.ang
        elif key == 'd':
            twist.linear.x = self.lin
            twist.angular.z = -self.ang
        elif key == ' ':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # 정의되지 않은 키 → 그냥 무시 (STOP은 메인 루프에서 처리)
            return True

        self.pub.publish(twist)
        self.get_logger().info(
            f"cmd_vel: lin={twist.linear.x:.2f} m/s, "
            f"ang={twist.angular.z:.2f} rad/s (key='{key}')"
        )
        return True


def main(args=None):
    # 터미널 설정 저장
    old_settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopKey()

    try:
        while rclpy.ok():
            # ROS 콜백 처리
            rclpy.spin_once(node, timeout_sec=0.0)

            # 키 입력 읽기
            key = get_key(old_settings, timeout=0.1)

            if key is None:
                # ★ 키 입력이 전혀 없으면 STOP 명령 보냄
                node.publish_stop()
                continue

            # 키 처리 (q / Ctrl+C면 False 반환)
            if not node.process_key(key):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 정지 명령 한 번 더 전송
        try:
            node.publish_stop()
        except Exception:
            pass

        # 터미널 설정 복구
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
