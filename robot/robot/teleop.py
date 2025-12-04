#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teleop_keyboard.py

- ROS 2 Humble + rclpy
- /cmd_vel (geometry_msgs/Twist) 퍼블리시
- odom_pid_node.py 의 파라미터에 맞춘 속도 한계:
    max_speed      = 0.8 [m/s]
    max_ang_speed  = 2.0 [rad/s]

키 매핑
  w : 전진 (linear.x +)
  x : 후진 (linear.x -)
  a : 좌회전 (angular.z +)
  d : 우회전 (angular.z -)
  space / s : 정지

추가 기능
  [ : 전체 선속도 스케일 ↓ (0.1 ~ 1.0)
  ] : 전체 선속도 스케일 ↑

특징
  - 키를 누르고 있을 동안에만 속도 유지
  - 키 입력이 일정 시간(예: 0.3초) 없으면 자동으로 정지 명령 전송
"""

import os
import select
import sys
import time

import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# ==================== 로봇 속도 한계 (odom_pid_node와 맞춤) ====================
MAX_LIN_VEL = 0.8   # [m/s]  odom_pid_node의 max_speed와 동일하게
MAX_ANG_VEL = 2.0   # [rad/s] odom_pid_node의 max_ang_speed와 동일

LIN_VEL_STEP_SIZE = 0.05   # 선속도 증가/감소 스텝
ANG_VEL_STEP_SIZE = 0.1    # 각속도 증가/감소 스텝

# ==================== 안내 메시지 ====================
msg = f"""
Control Your Robot (keyboard)
-----------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (max: {MAX_LIN_VEL:.2f} m/s)
a/d : increase/decrease angular velocity (max: {MAX_ANG_VEL:.2f} rad/s)
[   : decrease speed scale (0.1 ~ 1.0)
]   : increase speed scale (0.1 ~ 1.0)

space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    """터틀봇3 teleop와 동일한 방식: non-blocking 키 입력"""
    if os.name == 'nt':
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8')
        else:
            time.sleep(0.1)
            return ''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def constrain(val, low, high):
    if val < low:
        return low
    if val > high:
        return high
    return val


def main():
    # 터미널 설정 저장
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('teleop_keyboard')
    qos = QoSProfile(depth=10)
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    target_linear = 0.0
    target_angular = 0.0

    # 실제로 퍼블리시하는 값 (부드러운 변화 만들고 싶으면 프로파일링도 가능하지만, 일단 단순)
    control_linear = 0.0
    control_angular = 0.0

    # 속도 스케일 (전체 배율) : 0.1 ~ 1.0
    speed_scale = 1.0
    SPEED_SCALE_STEP = 0.1
    SPEED_SCALE_MIN = 0.1
    SPEED_SCALE_MAX = 1.0

    # "키 누르고 있을 때만 동작"을 위해 최근 키 입력 시각 체크
    last_key_time = time.time()
    KEY_TIMEOUT = 0.3  # 0.3초 이상 키 입력이 없으면 정지

    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings)

            now = time.time()
            key_pressed = False

            if key:
                key_pressed = True
                last_key_time = now

                # ========== 방향/속도 제어 ==========
                if key == 'w':
                    target_linear += LIN_VEL_STEP_SIZE
                    target_linear = constrain(target_linear, -MAX_LIN_VEL, MAX_LIN_VEL)
                    print(f"[w] target_linear = {target_linear:.3f}, scale = {speed_scale:.2f}")

                elif key == 'x':
                    target_linear -= LIN_VEL_STEP_SIZE
                    target_linear = constrain(target_linear, -MAX_LIN_VEL, MAX_LIN_VEL)
                    print(f"[x] target_linear = {target_linear:.3f}, scale = {speed_scale:.2f}")

                elif key == 'a':
                    target_angular += ANG_VEL_STEP_SIZE
                    target_angular = constrain(target_angular, -MAX_ANG_VEL, MAX_ANG_VEL)
                    print(f"[a] target_angular = {target_angular:.3f}, scale = {speed_scale:.2f}")

                elif key == 'd':
                    target_angular -= ANG_VEL_STEP_SIZE
                    target_angular = constrain(target_angular, -MAX_ANG_VEL, MAX_ANG_VEL)
                    print(f"[d] target_angular = {target_angular:.3f}, scale = {speed_scale:.2f}")

                # ========== 속도 스케일 조절 ==========
                elif key == '[':
                    speed_scale -= SPEED_SCALE_STEP
                    if speed_scale < SPEED_SCALE_MIN:
                        speed_scale = SPEED_SCALE_MIN
                    print(f"[ [ ] speed_scale ↓ -> {speed_scale:.2f}")

                elif key == ']':
                    speed_scale += SPEED_SCALE_STEP
                    if speed_scale > SPEED_SCALE_MAX:
                        speed_scale = SPEED_SCALE_MAX
                    print(f"[ ] ] speed_scale ↑ -> {speed_scale:.2f}")

                # ========== 정지 ==========
                elif key == ' ' or key == 's':
                    target_linear = 0.0
                    target_angular = 0.0
                    control_linear = 0.0
                    control_angular = 0.0
                    print("[STOP]")

                # ========== 종료 ==========
                elif key == '\x03':  # Ctrl+C
                    break

            # ===== 키 입력이 일정 시간 없으면 정지 =====
            if (now - last_key_time) > KEY_TIMEOUT:
                target_linear = 0.0
                target_angular = 0.0

            # 이 예제에서는 프로파일링은 생략하고 바로 반영해도 됨
            control_linear = target_linear * speed_scale
            control_angular = target_angular

            # ===== Twist 메시지 퍼블리시 =====
            twist = Twist()
            twist.linear.x = control_linear
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = control_angular

            pub.publish(twist)

    except Exception as ex:
        print(e)
        print(ex)

    finally:
        # 종료 시 정지 명령 한 번 더 보냄
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.linear.z = 0.0
        stop_twist.angular.x = 0.0
        stop_twist.angular.y = 0.0
        stop_twist.angular.z = 0.0
        pub.publish(stop_twist)

        if os.name != 'nt' and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
