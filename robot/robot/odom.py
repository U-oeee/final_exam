#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom.py (OdometryPIDNode) - STANDARD TF TREE VERSION
- TF Tree: odom -> base_footprint -> base_link -> laser_frame
- Fixes: Local map spinning issue by adding proper base_footprint.
- Includes: Previous bug fixes (TypeError, Dynamic TF)
"""

import math
from threading import Lock
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros

# ==================== 하드웨어 핀 (BOARD) ====================
MOTOR1_DIR = 40; MOTOR1_PWM = 32; MOTOR1_EN  = 37
MOTOR1_HALL_A = 35; MOTOR1_HALL_B = 36
MOTOR2_DIR = 18; MOTOR2_PWM = 12; MOTOR2_EN  = 15
MOTOR2_HALL_A = 13; MOTOR2_HALL_B = 16

# ==================== 엔코더 / 기구 ====================
ENCODER_PPR    = 26.0
GEAR_RATIO     = 60.0
WHEEL_DIAMETER = 0.115 
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
MOTOR_TICKS_PER_REV = ENCODER_PPR * 4.0
TICKS_PER_REV = MOTOR_TICKS_PER_REV * GEAR_RATIO

def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

# ==================== PID ====================
class PID:
    def __init__(self, kp, ki, kd, out_min=0.0, out_max=100.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self.out_min = out_min; self.out_max = out_max
        self.integral = 0.0; self.prev_error = 0.0; self.first = True

    def update(self, target, meas, dt):
        if dt <= 0.0: return 0.0
        error = target - meas
        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral
        if self.first:
            d = 0.0; self.first = False
        else:
            d = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        out = p + i + d
        return min(self.out_max, max(self.out_min, out))

# ==================== 쿼드러처 디코드 ====================
_QUAD_TABLE = {
    (0, 1): +1, (1, 3): +1, (3, 2): +1, (2, 0): +1,
    (0, 2): -1, (2, 3): -1, (3, 1): -1, (1, 0): -1,
}
def decode_x4(prev_a, prev_b, curr_a, curr_b):
    prev = (int(prev_a) << 1) | int(prev_b)
    curr = (int(curr_a) << 1) | int(curr_b)
    val = _QUAD_TABLE.get((prev, curr), 0)
    return abs(val), val

# ==================== 메인 노드 ====================
class OdometryPIDNode(Node):
    def __init__(self):
        super().__init__("odom")

        # Parameters
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("kp", 750.0); self.declare_parameter("ki", 6.0); self.declare_parameter("kd", 15.0)
        self.declare_parameter("min_pwm", 15.0); self.declare_parameter("max_pwm_delta", 8.0)
        self.declare_parameter("wheel_base", 0.33); self.declare_parameter("max_speed", 0.8)
        self.declare_parameter("max_ang_speed", 2.0); self.declare_parameter("v_deadband", 0.01)
        self.declare_parameter("omega_deadband", 0.03); self.declare_parameter("dt_max", 0.2)
        self.declare_parameter("cmd_timeout", 0.5); self.declare_parameter("use_imu_yaw", False)
        self.declare_parameter("imu_yaw_correction_gain", 0.01); self.declare_parameter("imu_only_when_moving", True)
        self.declare_parameter("left_dir_sign", 1.0); self.declare_parameter("right_dir_sign", 1.0)
        self.declare_parameter("en_active_low", True); self.declare_parameter("lidar_yaw_deg", 30.0)
        self.declare_parameter("debug", False)
        
        # Read Params
        control_hz = float(self.get_parameter("control_hz").value)
        self.nominal_dt = 1.0 / max(1.0, control_hz)
        kp = float(self.get_parameter("kp").value); ki = float(self.get_parameter("ki").value); kd = float(self.get_parameter("kd").value)
        self.min_pwm = float(self.get_parameter("min_pwm").value)
        self.max_pwm_delta = float(self.get_parameter("max_pwm_delta").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_ang_speed = float(self.get_parameter("max_ang_speed").value)
        self.v_deadband = float(self.get_parameter("v_deadband").value)
        self.omega_deadband = float(self.get_parameter("omega_deadband").value)
        self.dt_max = float(self.get_parameter("dt_max").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.use_imu_yaw = bool(self.get_parameter("use_imu_yaw").value)
        self.imu_corr_gain = float(self.get_parameter("imu_yaw_correction_gain").value)
        self.imu_only_when_moving = bool(self.get_parameter("imu_only_when_moving").value)
        self.left_dir_sign = 1.0 if float(self.get_parameter("left_dir_sign").value) >= 0 else -1.0
        self.right_dir_sign = 1.0 if float(self.get_parameter("right_dir_sign").value) >= 0 else -1.0
        self.en_active_low = bool(self.get_parameter("en_active_low").value)
        
        self.lidar_yaw_deg = float(self.get_parameter("lidar_yaw_deg").value)
        yaw_rad = math.radians(self.lidar_yaw_deg)
        self.lidar_qz = math.sin(yaw_rad / 2.0)
        self.lidar_qw = math.cos(yaw_rad / 2.0)

        # State
        self.encoder_lock = Lock()
        self.m1_count = 0; self.m2_count = 0; self.m1_dir = 0; self.m2_dir = 0
        self.m1_prev_a = 0; self.m1_prev_b = 0; self.m2_prev_a = 0; self.m2_prev_b = 0
        self.m1_prev_count = 0; self.m2_prev_count = 0
        self.pid_left = PID(kp, ki, kd); self.pid_right = PID(kp, ki, kd)
        self.cmd_linear_x = 0.0; self.cmd_ang_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_pwmL = 0.0; self.last_pwmR = 0.0
        self.x = 0.0; self.y = 0.0; self.yaw_enc = 0.0
        self.yaw_imu = 0.0; self.have_imu = False
        self.last_time = self.get_clock().now()

        # Setup
        self.setup_gpio()
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 20)
        self.timer = self.create_timer(self.nominal_dt, self.control_loop)

    def set_enable(self, enable: bool):
        level = GPIO.LOW if (self.en_active_low and enable) else (GPIO.HIGH if not self.en_active_low and enable else (GPIO.HIGH if self.en_active_low else GPIO.LOW))
        # Logic fix for cleaner read:
        # Active LOW: Enable=True -> LOW
        # Active HIGH: Enable=True -> HIGH
        if self.en_active_low:
            level = GPIO.LOW if enable else GPIO.HIGH
        else:
            level = GPIO.HIGH if enable else GPIO.LOW
        GPIO.output(MOTOR1_EN, level); GPIO.output(MOTOR2_EN, level)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear_x = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.cmd_ang_z = max(-self.max_ang_speed, min(self.max_ang_speed, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        if not self.use_imu_yaw: return
        if msg.orientation_covariance[0] == -1.0: return
        qx, qy, qz, qw = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw_imu = math.atan2(siny, cosy)
        self.have_imu = True

    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD); GPIO.setwarnings(False)
        GPIO.setup(MOTOR1_DIR, GPIO.OUT); GPIO.setup(MOTOR1_PWM, GPIO.OUT); GPIO.setup(MOTOR1_EN, GPIO.OUT)
        GPIO.setup(MOTOR1_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP); GPIO.setup(MOTOR1_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR2_DIR, GPIO.OUT); GPIO.setup(MOTOR2_PWM, GPIO.OUT); GPIO.setup(MOTOR2_EN, GPIO.OUT)
        GPIO.setup(MOTOR2_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP); GPIO.setup(MOTOR2_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.pwm1 = GPIO.PWM(MOTOR1_PWM, 1000); self.pwm2 = GPIO.PWM(MOTOR2_PWM, 1000)
        self.pwm1.start(0.0); self.pwm2.start(0.0)
        GPIO.output(MOTOR1_DIR, GPIO.LOW); GPIO.output(MOTOR2_DIR, GPIO.LOW)
        self.set_enable(False)
        self.m1_prev_a = GPIO.input(MOTOR1_HALL_A); self.m1_prev_b = GPIO.input(MOTOR1_HALL_B)
        self.m2_prev_a = GPIO.input(MOTOR2_HALL_A); self.m2_prev_b = GPIO.input(MOTOR2_HALL_B)
        GPIO.add_event_detect(MOTOR1_HALL_A, GPIO.BOTH, callback=self.motor1_callback)
        GPIO.add_event_detect(MOTOR1_HALL_B, GPIO.BOTH, callback=self.motor1_callback)
        GPIO.add_event_detect(MOTOR2_HALL_A, GPIO.BOTH, callback=self.motor2_callback)
        GPIO.add_event_detect(MOTOR2_HALL_B, GPIO.BOTH, callback=self.motor2_callback)

    def motor1_callback(self, _ch):
        curr_a, curr_b = GPIO.input(MOTOR1_HALL_A), GPIO.input(MOTOR1_HALL_B)
        delta, direction = decode_x4(self.m1_prev_a, self.m1_prev_b, curr_a, curr_b)
        self.m1_prev_a, self.m1_prev_b = curr_a, curr_b
        if delta:
            with self.encoder_lock:
                self.m1_count += 1; self.m1_dir = direction

    def motor2_callback(self, _ch):
        curr_a, curr_b = GPIO.input(MOTOR2_HALL_A), GPIO.input(MOTOR2_HALL_B)
        delta, direction = decode_x4(self.m2_prev_a, self.m2_prev_b, curr_a, curr_b)
        self.m2_prev_a, self.m2_prev_b = curr_a, curr_b
        if delta:
            with self.encoder_lock:
                self.m2_count += 1; self.m2_dir = direction

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > self.dt_max: dt = self.nominal_dt

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            v_cmd = 0.0; w_cmd = 0.0
        else:
            v_cmd = self.cmd_linear_x; w_cmd = self.cmd_ang_z

        tgt_left = v_cmd - w_cmd * self.wheel_base / 2.0
        tgt_right = v_cmd + w_cmd * self.wheel_base / 2.0

        with self.encoder_lock:
            m1, d1 = self.m1_count, self.m1_dir
            m2, d2 = self.m2_count, self.m2_dir
        delta1 = m1 - self.m1_prev_count; self.m1_prev_count = m1
        delta2 = m2 - self.m2_prev_count; self.m2_prev_count = m2

        rev1 = (delta1 / TICKS_PER_REV) / dt
        rev2 = (delta2 / TICKS_PER_REV) / dt
        speed_left_mag = rev1 * WHEEL_CIRCUMFERENCE
        speed_right_mag = rev2 * WHEEL_CIRCUMFERENCE

        if delta1 != 0: signL = 1.0 if d1 >= 0 else -1.0
        else: signL = 0.0 if abs(tgt_left) < 1e-6 else (1.0 if tgt_left >= 0 else -1.0)
        if delta2 != 0: signR = 1.0 if d2 >= 0 else -1.0
        else: signR = 0.0 if abs(tgt_right) < 1e-6 else (1.0 if tgt_right >= 0 else -1.0)

        v_left = speed_left_mag * signL * self.left_dir_sign
        v_right = speed_right_mag * signR * self.right_dir_sign

        pwmL = self.pid_left.update(abs(tgt_left), abs(speed_left_mag), dt)
        pwmR = self.pid_right.update(abs(tgt_right), abs(speed_right_mag), dt)
        
        pwmL = max(self.min_pwm, pwmL) if abs(tgt_left) > 1e-3 else 0.0
        pwmR = max(self.min_pwm, pwmR) if abs(tgt_right) > 1e-3 else 0.0
        pwmL = self._slew(self.last_pwmL, pwmL, self.max_pwm_delta)
        pwmR = self._slew(self.last_pwmR, pwmR, self.max_pwm_delta)
        self.last_pwmL, self.last_pwmR = pwmL, pwmR

        self.apply_pwm(pwmL, pwmR, tgt_left, tgt_right)
        self.update_and_publish_odom(v_left, v_right, dt)

    def _slew(self, prev, target, max_delta):
        if max_delta <= 0.0: return target
        diff = target - prev
        if diff > max_delta: return prev + max_delta
        if diff < -max_delta: return prev - max_delta
        return target

    def update_and_publish_odom(self, v_left, v_right, dt):
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base
        moving = (abs(v) >= self.v_deadband) or (abs(omega) >= self.omega_deadband)

        if moving:
            self.yaw_enc = wrap_pi(self.yaw_enc + omega * dt)
            if self.use_imu_yaw and self.have_imu:
                if (not self.imu_only_when_moving) or moving:
                    diff = wrap_pi(self.yaw_imu - self.yaw_enc)
                    self.yaw_enc = wrap_pi(self.yaw_enc + self.imu_corr_gain * diff)
        else:
            v = 0.0; omega = 0.0

        self.x += v * dt * math.cos(self.yaw_enc)
        self.y += v * dt * math.sin(self.yaw_enc)
        qz = math.sin(self.yaw_enc / 2.0); qw = math.cos(self.yaw_enc / 2.0)
        stamp = self.get_clock().now().to_msg()

        # 1. Odometry Msg (odom -> base_footprint)
        od = Odometry()
        od.header.stamp = stamp; od.header.frame_id = "odom"
        # [변경] 이제 Odometry는 base_footprint를 기준으로 합니다.
        od.child_frame_id = "base_footprint"
        od.pose.pose.position.x = self.x; od.pose.pose.position.y = self.y; od.pose.pose.position.z = 0.0
        od.pose.pose.orientation.z = qz; od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = v; od.twist.twist.angular.z = omega
        self.odom_pub.publish(od)

        # 2. TF: odom -> base_footprint (Dynamic)
        t_odom = TransformStamped()
        t_odom.header.stamp = stamp; t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_footprint"
        t_odom.transform.translation.x = self.x; t_odom.transform.translation.y = self.y; t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation.z = qz; t_odom.transform.rotation.w = qw

        # 3. TF: base_footprint -> base_link (Static, Fixed)
        # 로봇 중심을 0.0으로 둡니다. (만약 바퀴 때문에 몸체가 떠 있다면 z를 0.05 등으로 주면 됨)
        t_base = TransformStamped()
        t_base.header.stamp = stamp; t_base.header.frame_id = "base_footprint"
        t_base.child_frame_id = "base_link"
        t_base.transform.translation.x = 0.0; t_base.transform.translation.y = 0.0; t_base.transform.translation.z = 0.0
        t_base.transform.rotation.z = 0.0; t_base.transform.rotation.w = 1.0

        # 4. TF: base_link -> laser_frame (Static, Fixed)
        t_laser = TransformStamped()
        t_laser.header.stamp = stamp; t_laser.header.frame_id = "base_link"
        t_laser.child_frame_id = "laser_frame"
        t_laser.transform.translation.x = 0.0; t_laser.transform.translation.y = 0.0; t_laser.transform.translation.z = 0.15
        t_laser.transform.rotation.x = 0.0; t_laser.transform.rotation.y = 0.0
        t_laser.transform.rotation.z = self.lidar_qz; t_laser.transform.rotation.w = self.lidar_qw

        # 3개의 TF를 동시에 방송
        self.tf_broadcaster.sendTransform([t_odom, t_base, t_laser])

    def apply_pwm(self, pwmL, pwmR, tgtL, tgtR):
        GPIO.output(MOTOR1_DIR, GPIO.LOW if tgtL >= 0 else GPIO.HIGH)
        GPIO.output(MOTOR2_DIR, GPIO.LOW if tgtR >= 0 else GPIO.HIGH)
        pwmL = max(0.0, min(100.0, float(pwmL)))
        pwmR = max(0.0, min(100.0, float(pwmR)))
        moving = (pwmL > 0.0) or (pwmR > 0.0)
        self.set_enable(moving)
        self.pwm1.ChangeDutyCycle(pwmL); self.pwm2.ChangeDutyCycle(pwmR)

    def destroy_node(self):
        try:
            self.pwm1.ChangeDutyCycle(0.0); self.pwm2.ChangeDutyCycle(0.0); self.set_enable(False)
            self.pwm1.stop(); self.pwm2.stop(); GPIO.cleanup()
        except Exception: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = OdometryPIDNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__": main()
