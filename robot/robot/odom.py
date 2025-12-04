#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom.py (OdometryPIDNode)

- RB-35GM 11TYPE (Encoder 26P/R) + 감속비 1/60
- 라즈베리파이4 + Ubuntu 22.04 + ROS 2 Humble
- RPi.GPIO + ROS2 Node

기능
1) /cmd_vel 구독 → 좌/우 바퀴 목표 속도 계산
2) PID 속도 제어
3) 오돔: 엔코더가 메인, IMU는 보조 yaw 보정
4) /odom 퍼블리시 + TF(odom→base_link)
"""

import math
import time
from threading import Lock

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros

# ==================== 하드웨어 핀 ====================
MOTOR1_DIR = 40
MOTOR1_PWM = 32
MOTOR1_EN  = 37
MOTOR1_HALL_A = 35
MOTOR1_HALL_B = 36

MOTOR2_DIR = 18
MOTOR2_PWM = 12
MOTOR2_EN  = 15
MOTOR2_HALL_A = 13
MOTOR2_HALL_B = 16

# ==================== 엔코더 / 기구 ====================
ENCODER_PPR    = 13.0
GEAR_RATIO     = 60.0
WHEEL_DIAMETER = 0.115     # 11.5cm
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER

MOTOR_TICKS_PER_REV = ENCODER_PPR * 4.0
TICKS_PER_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO


# ==================== PID ====================
class PID:
    def __init__(self, kp, ki, kd, out_min=0.0, out_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, target, meas, dt):
        error = target - meas
        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral

        if self.first:
            d = 0.0
            self.first = False
        else:
            d = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        out = p + i + d
        out = min(self.out_max, max(self.out_min, out))
        return out


# ==================== 쿼드러처 디코드 ====================
def decode_x4(prev_a, prev_b, curr_a, curr_b):
    delta = 0
    direction = 0

    # A
    if prev_a == 0 and curr_a == 1:
        delta = 1
        direction = 1 if curr_b == 0 else -1
    elif prev_a == 1 and curr_a == 0:
        delta = 1
        direction = 1 if curr_b == 1 else -1

    # B
    elif prev_b == 0 and curr_b == 1:
        delta = 1
        direction = 1 if curr_a == 1 else -1
    elif prev_b == 1 and curr_b == 0:
        delta = 1
        direction = 1 if curr_a == 0 else -1

    return delta, direction


# ==================== 메인 클래스 ====================
class OdometryPIDNode(Node):
    def __init__(self):
        super().__init__("odom")

        # Parameters
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("kp", 750.0)
        self.declare_parameter("ki", 6.0)
        self.declare_parameter("kd", 15.0)
        self.declare_parameter("min_pwm", 15.0)
        self.declare_parameter("max_pwm_delta", 5.0)
        self.declare_parameter("plot", False)
        self.declare_parameter("debug", False)

        self.declare_parameter("wheel_base", 0.33)  # 33 cm
        self.declare_parameter("max_speed", 0.8)
        self.declare_parameter("max_ang_speed", 2.0)

        self.declare_parameter("imu_yaw_correction_gain", 0.02)

        # 방향 보정
        self.declare_parameter("left_dir_sign", 1.0)
        self.declare_parameter("right_dir_sign", 1.0)

        # Read params
        control_hz = float(self.get_parameter("control_hz").value)
        self.dt = 1.0 / control_hz

        kp = float(self.get_parameter("kp").value)
        ki = float(self.get_parameter("ki").value)
        kd = float(self.get_parameter("kd").value)

        self.min_pwm = float(self.get_parameter("min_pwm").value)
        self.max_pwm_delta = float(self.get_parameter("max_pwm_delta").value)

        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_ang_speed = float(self.get_parameter("max_ang_speed").value)

        self.imu_corr_gain = float(self.get_parameter("imu_yaw_correction_gain").value)

        # 방향 보정
        self.left_dir_sign = 1.0 if float(self.get_parameter("left_dir_sign").value) >= 0 else -1.0
        self.right_dir_sign = 1.0 if float(self.get_parameter("right_dir_sign").value) >= 0 else -1.0

        self.debug = bool(self.get_parameter("debug").value)
        self.plot_flag = bool(self.get_parameter("plot").value)

        self.get_logger().info(
            f"[ODOM] wheel_base={self.wheel_base}, imu_gain={self.imu_corr_gain}"
        )

        # 상태 변수
        self.encoder_lock = Lock()
        self.m1_count = 0
        self.m2_count = 0
        self.m1_prev_count = 0
        self.m2_prev_count = 0
        self.m1_dir = 0
        self.m2_dir = 0

        # 속도
        self.meas_v_left = 0.0
        self.meas_v_right = 0.0

        # 오돔용 부호 포함 속도
        self.odom_v_left = 0.0
        self.odom_v_right = 0.0

        # PID
        self.pid_left = PID(kp, ki, kd)
        self.pid_right = PID(kp, ki, kd)

        # cmd_vel
        self.cmd_linear_x = 0.0
        self.cmd_ang_z = 0.0

        # 오돔 적분 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw_enc = 0.0
        self.yaw_imu = 0.0
        self.yaw = 0.0
        self.have_imu = False

        # 그래프
        if self.plot_flag:
            self.setup_plot()

        self.setup_gpio()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Imu, "/imu/data", self.imu_callback, 20)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.create_timer(self.dt, self.control_loop)

    # ---------- cmd_vel ----------
    def cmd_vel_callback(self, msg):
        self.cmd_linear_x = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.cmd_ang_z = max(-self.max_ang_speed, min(self.max_ang_speed, msg.angular.z))

    # ---------- IMU ----------
    def imu_callback(self, msg):
        if msg.orientation_covariance[0] == -1.0:
            return

        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny, cosy)

        self.yaw_imu = yaw
        self.have_imu = True

    # ---------- GPIO ----------
    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        GPIO.setup(MOTOR1_DIR, GPIO.OUT)
        GPIO.setup(MOTOR1_PWM, GPIO.OUT)
        GPIO.setup(MOTOR1_EN, GPIO.OUT)
        GPIO.setup(MOTOR1_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR1_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(MOTOR2_DIR, GPIO.OUT)
        GPIO.setup(MOTOR2_PWM, GPIO.OUT)
        GPIO.setup(MOTOR2_EN, GPIO.OUT)
        GPIO.setup(MOTOR2_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR2_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pwm1 = GPIO.PWM(MOTOR1_PWM, 1000)
        self.pwm2 = GPIO.PWM(MOTOR2_PWM, 1000)
        self.pwm1.start(0.0)
        self.pwm2.start(0.0)

        GPIO.add_event_detect(MOTOR1_HALL_A, GPIO.BOTH, self.motor1_callback)
        GPIO.add_event_detect(MOTOR1_HALL_B, GPIO.BOTH, self.motor1_callback)
        GPIO.add_event_detect(MOTOR2_HALL_A, GPIO.BOTH, self.motor2_callback)
        GPIO.add_event_detect(MOTOR2_HALL_B, GPIO.BOTH, self.motor2_callback)

    def motor1_callback(self, ch):
        curr_a = GPIO.input(MOTOR1_HALL_A)
        curr_b = GPIO.input(MOTOR1_HALL_B)
        delta, direction = decode_x4(0, 0, curr_a, curr_b)
        if delta:
            with self.encoder_lock:
                self.m1_count += 1
                self.m1_dir = direction

    def motor2_callback(self, ch):
        curr_a = GPIO.input(MOTOR2_HALL_A)
        curr_b = GPIO.input(MOTOR2_HALL_B)
        delta, direction = decode_x4(0, 0, curr_a, curr_b)
        if delta:
            with self.encoder_lock:
                self.m2_count += 1
                self.m2_dir = direction

    # ---------- 제어 루프 ----------
    def control_loop(self):
        v = self.cmd_linear_x
        w = self.cmd_ang_z

        tgt_left = v - w * self.wheel_base / 2.0
        tgt_right = v + w * self.wheel_base / 2.0

        # 엔코더 읽기
        with self.encoder_lock:
            m1, d1 = self.m1_count, self.m1_dir
            m2, d2 = self.m2_count, self.m2_dir

        delta1 = m1 - self.m1_prev_count
        delta2 = m2 - self.m2_prev_count
        self.m1_prev_count = m1
        self.m2_prev_count = m2

        # 실제 속도 크기만 계산
        rev1 = delta1 / TICKS_PER_REV / self.dt
        rev2 = delta2 / TICKS_PER_REV / self.dt

        speed_left_mag = rev1 * WHEEL_CIRCUMFERENCE
        speed_right_mag = rev2 * WHEEL_CIRCUMFERENCE

        # 오돔에서 사용할 부호는 "명령 방향"
        sign_left_cmd = 1.0 if tgt_left >= 0 else -1.0
        sign_right_cmd = 1.0 if tgt_right >= 0 else -1.0

        # 오돔용 속도 (전/후진 확실히 반영됨)
        self.odom_v_left = speed_left_mag * sign_left_cmd * self.left_dir_sign
        self.odom_v_right = speed_right_mag * sign_right_cmd * self.right_dir_sign

        # PID에서는 크기만 사용
        vL = abs(speed_left_mag)
        vR = abs(speed_right_mag)

        pwmL = self.pid_left.update(abs(tgt_left), vL, self.dt)
        pwmR = self.pid_right.update(abs(tgt_right), vR, self.dt)

        pwmL = max(self.min_pwm, pwmL) if abs(tgt_left) > 0 else 0
        pwmR = max(self.min_pwm, pwmR) if abs(tgt_right) > 0 else 0

        self.apply_pwm(pwmL, pwmR, tgt_left, tgt_right)

        self.update_and_publish_odom()

    # ---------- 오돔 ----------
    def update_and_publish_odom(self):
        v_left = self.odom_v_left
        v_right = self.odom_v_right

        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # 휠 오돔 yaw 적분
        self.yaw_enc += omega * self.dt
        self.yaw_enc = math.atan2(math.sin(self.yaw_enc), math.cos(self.yaw_enc))

        # IMU로 서서히 보정
        if self.have_imu:
            diff = math.atan2(
                math.sin(self.yaw_imu - self.yaw_enc),
                math.cos(self.yaw_imu - self.yaw_enc)
            )
            self.yaw_enc += self.imu_corr_gain * diff
            self.yaw_enc = math.atan2(math.sin(self.yaw_enc), math.cos(self.yaw_enc))

        self.yaw = self.yaw_enc

        # 위치 적분
        self.x += v * self.dt * math.cos(self.yaw)
        self.y += v * self.dt * math.sin(self.yaw)

        # quaternion(yaw only)
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        now = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"

        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw

        od.twist.twist.linear.x = v
        od.twist.twist.angular.z = omega

        self.odom_pub.publish(od)

        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    # ---------- PWM ----------
    def apply_pwm(self, pwmL, pwmR, tgtL, tgtR):
        GPIO.output(MOTOR1_DIR, GPIO.LOW if tgtL >= 0 else GPIO.HIGH)
        GPIO.output(MOTOR2_DIR, GPIO.LOW if tgtR >= 0 else GPIO.HIGH)

        self.pwm1.ChangeDutyCycle(pwmL)
        self.pwm2.ChangeDutyCycle(pwmR)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
