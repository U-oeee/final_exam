#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_pid_speed_manual_smooth.py

- RB-35GM 11TYPE (Encoder 26P/R) + 감속비 1/60
- 라즈베리파이4 + Ubuntu 22.04 + ROS 2 Humble
- RPi.GPIO + ROS2 Node

동작 개요
1) 사용자가 target_speed [m/s]를 직접 지정 (파라미터)
2) 왼쪽/오른쪽 모터 각각에 대해 PID 속도 제어 수행
3) 끊김 완화를 위한 처리:
   - 제어 주기 30 Hz
   - 엔코더 인터럽트: 소프트웨어 디바운스(bouncetime) 사용 안 함
   - PWM 변화율 제한 (smooth_pwm)
   - 최소 PWM (min_pwm) 적용: 아주 작은 듀티에서 덜덜거리는 현상 감소
4) 실시간 그래프:
   - 위 그래프: 속도 [m/s]
       초록 점선 : target_speed
       파란 실선 : left speed
       빨간 실선 : right speed
   - 아래 그래프: PWM [%]
       파란 실선 : left PWM
       빨간 실선 : right PWM
"""

import math
import time
from threading import Lock

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt

# ==================== 하드웨어 핀 정의 (BOARD 모드) ====================
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

# ==================== 엔코더/기구 파라미터 ====================
ENCODER_PPR    = 26.0      # 모터축 P/R
GEAR_RATIO     = 60.0      # 감속비
WHEEL_DIAMETER = 0.115     # [m] 바퀴 지름 11.5cm

MOTOR_TICKS_PER_REV = ENCODER_PPR * 4.0      # 쿼드러처 X4 디코딩
TICKS_PER_REV       = MOTOR_TICKS_PER_REV * GEAR_RATIO
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER


# ==================== PID 클래스 ====================
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

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.first = True

    def update(self, target, measurement, dt):
        """
        target, measurement : 속도 [m/s]
        out : PWM duty [%] (0~100)
        """
        error = target - measurement

        # P
        p = self.kp * error

        # I
        self.integral += error * dt
        i = self.ki * self.integral

        # D
        if self.first or dt <= 0.0:
            d = 0.0
            self.first = False
        else:
            d = self.kd * (error - self.prev_error) / dt

        self.prev_error = error

        out = p + i + d
        if out > self.out_max:
            out = self.out_max
        elif out < self.out_min:
            out = self.out_min
        return out


# ==================== 쿼드러처 X4 디코딩 ====================
def decode_x4(prev_a, prev_b, curr_a, curr_b):
    """
    필요하면 네 motor.py에서 쓰던 함수랑 1:1로 교체해도 됨.
    """
    delta = 0
    direction = 0

    # A 엣지
    if prev_a == 0 and curr_a == 1:      # rising
        delta = 1
        direction = 1 if curr_b == 0 else -1
    elif prev_a == 1 and curr_a == 0:    # falling
        delta = 1
        direction = 1 if curr_b == 1 else -1

    # B 엣지
    elif prev_b == 0 and curr_b == 1:
        delta = 1
        direction = 1 if curr_a == 1 else -1
    elif prev_b == 1 and curr_b == 0:
        delta = 1
        direction = 1 if curr_a == 0 else -1

    return delta, direction


# ==================== 메인 노드 ====================
class RB35SpeedPIDNode(Node):
    def __init__(self):
        super().__init__('motor_pid_speed_manual_smooth')

        # ===== 파라미터 =====
        self.declare_parameter('control_hz', 30.0)        # 제어 주기 [Hz]
        self.declare_parameter('target_speed', 0.65)      # 목표 속도 [m/s]
        self.declare_parameter('kp', 850.0)
        self.declare_parameter('ki', 5.5)
        self.declare_parameter('kd', 5.0)
        self.declare_parameter('debug', True)
        self.declare_parameter('plot', True)
        self.declare_parameter('min_pwm', 15.0)           # 최소 PWM [%]
        self.declare_parameter('max_pwm_delta', 5.0)      # 한 주기당 PWM 변화 허용량 [%]

        control_hz        = float(self.get_parameter('control_hz').value)
        self.dt           = 1.0 / control_hz
        self.target_speed = float(self.get_parameter('target_speed').value)
        kp                = float(self.get_parameter('kp').value)
        ki                = float(self.get_parameter('ki').value)
        kd                = float(self.get_parameter('kd').value)
        self.debug        = bool(self.get_parameter('debug').value)
        self.plot_flag    = bool(self.get_parameter('plot').value)
        self.min_pwm      = float(self.get_parameter('min_pwm').value)
        self.max_pwm_delta = float(self.get_parameter('max_pwm_delta').value)

        self.get_logger().info(
            f"Node started (control_hz={control_hz}, target_speed={self.target_speed:.4f} m/s)"
        )
        self.get_logger().info(
            f"TICKS_PER_REV={TICKS_PER_REV}, wheel_circ={WHEEL_CIRCUMFERENCE:.4f} m"
        )

        # ===== 상태 변수 =====
        self.encoder_lock = Lock()
        self.m1_count = 0
        self.m2_count = 0
        self.m1_prev_count = 0
        self.m2_prev_count = 0
        self.m1_dir = 0
        self.m2_dir = 0
        self.m1_prev_a = 0
        self.m1_prev_b = 0
        self.m2_prev_a = 0
        self.m2_prev_b = 0

        # 속도 [m/s]
        self.meas_v_left  = 0.0
        self.meas_v_right = 0.0

        # 현재 PWM [%] (각 모터별)
        self.current_pwm_left  = 0.0
        self.current_pwm_right = 0.0

        # PID (모터별)
        self.pid_left  = PID(kp, ki, kd, out_min=0.0, out_max=100.0)
        self.pid_right = PID(kp, ki, kd, out_min=0.0, out_max=100.0)

        # GPIO 초기화
        self.setup_gpio()

        # 그래프 설정
        if self.plot_flag:
            self.setup_plot()

        # 제어 루프 타이머
        self.control_timer = self.create_timer(self.dt, self.control_loop)

    # ---------- PWM 변화율 제한 ----------
    def smooth_pwm(self, prev, new):
        """
        한 주기마다 PWM이 너무 많이 변하지 않도록 제한.
        ex) max_delta=5 이면, 한 번에 ±5% 이상 변하지 않게 함.
        """
        max_delta = self.max_pwm_delta
        if new > prev + max_delta:
            return prev + max_delta
        if new < prev - max_delta:
            return prev - max_delta
        return new

    # ---------- GPIO / 엔코더 ----------
    def setup_gpio(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # 모터 핀
        GPIO.setup(MOTOR1_DIR, GPIO.OUT)
        GPIO.setup(MOTOR1_PWM, GPIO.OUT)
        GPIO.setup(MOTOR1_EN,  GPIO.OUT)
        GPIO.setup(MOTOR2_DIR, GPIO.OUT)
        GPIO.setup(MOTOR2_PWM, GPIO.OUT)
        GPIO.setup(MOTOR2_EN,  GPIO.OUT)

        # 엔코더 핀
        GPIO.setup(MOTOR1_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR1_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR2_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(MOTOR2_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.pwm1 = GPIO.PWM(MOTOR1_PWM, 1000)  # 1 kHz
        self.pwm2 = GPIO.PWM(MOTOR2_PWM, 1000)
        self.pwm1.start(0.0)
        self.pwm2.start(0.0)

        self.m1_prev_a = GPIO.input(MOTOR1_HALL_A)
        self.m1_prev_b = GPIO.input(MOTOR1_HALL_B)
        self.m2_prev_a = GPIO.input(MOTOR2_HALL_A)
        self.m2_prev_b = GPIO.input(MOTOR2_HALL_B)

        # ⚠ bouncetime 인자 제거 → 소프트웨어 디바운스 사용 안 함
        GPIO.add_event_detect(MOTOR1_HALL_A, GPIO.BOTH,
                              callback=self.motor1_callback)
        GPIO.add_event_detect(MOTOR1_HALL_B, GPIO.BOTH,
                              callback=self.motor1_callback)
        GPIO.add_event_detect(MOTOR2_HALL_A, GPIO.BOTH,
                              callback=self.motor2_callback)
        GPIO.add_event_detect(MOTOR2_HALL_B, GPIO.BOTH,
                              callback=self.motor2_callback)

        # 정지 상태로 초기화
        GPIO.output(MOTOR1_EN, GPIO.LOW)
        GPIO.output(MOTOR2_EN, GPIO.LOW)
        GPIO.output(MOTOR1_DIR, GPIO.LOW)
        GPIO.output(MOTOR2_DIR, GPIO.LOW)

        self.get_logger().info("GPIO initialized (BOARD mode)")

    def motor1_callback(self, channel):
        curr_a = GPIO.input(MOTOR1_HALL_A)
        curr_b = GPIO.input(MOTOR1_HALL_B)
        delta, direction = decode_x4(self.m1_prev_a, self.m1_prev_b,
                                     curr_a, curr_b)
        if delta:
            with self.encoder_lock:
                self.m1_count += 1
                self.m1_dir = direction
        self.m1_prev_a = curr_a
        self.m1_prev_b = curr_b

    def motor2_callback(self, channel):
        curr_a = GPIO.input(MOTOR2_HALL_A)
        curr_b = GPIO.input(MOTOR2_HALL_B)
        delta, direction = decode_x4(self.m2_prev_a, self.m2_prev_b,
                                     curr_a, curr_b)
        if delta:
            with self.encoder_lock:
                self.m2_count += 1
                self.m2_dir = direction
        self.m2_prev_a = curr_a
        self.m2_prev_b = curr_b

    # ---------- matplotlib ----------    
    def setup_plot(self):
        plt.ion()
        self.fig, (self.ax_speed, self.ax_pwm) = plt.subplots(2, 1, sharex=True, figsize=(6, 6))

        # 속도 그래프
        self.ax_speed.set_title("Speed control (target vs left/right)")
        self.ax_speed.set_ylabel("v [m/s]")

        # PWM 그래프
        self.ax_pwm.set_title("PWM output (left/right)")
        self.ax_pwm.set_xlabel("time [s]")
        self.ax_pwm.set_ylabel("PWM [%]")
        self.ax_pwm.set_ylim(0, 100)

        # 히스토리 버퍼
        self.time_hist = []
        self.target_speed_hist = []   # 목표 속도
        self.left_speed_hist = []     # 왼쪽 모터 속도
        self.right_speed_hist = []    # 오른쪽 모터 속도
        self.left_pwm_hist = []       # 왼쪽 PWM
        self.right_pwm_hist = []      # 오른쪽 PWM

        # 선 객체
        (self.line_target_speed,) = self.ax_speed.plot([], [], 'g--', label='target speed')
        (self.line_left_speed,)   = self.ax_speed.plot([], [], 'b-', label='left speed')
        (self.line_right_speed,)  = self.ax_speed.plot([], [], 'r-', label='right speed')
        self.ax_speed.legend()

        (self.line_left_pwm,)  = self.ax_pwm.plot([], [], 'b-', label='left PWM')
        (self.line_right_pwm,) = self.ax_pwm.plot([], [], 'r-', label='right PWM')
        self.ax_pwm.legend()

        self.plot_start_time = time.time()

    # ---------- 제어 루프 ----------
    def control_loop(self):
        # 1) 엔코더 값 읽어서 속도 계산
        with self.encoder_lock:
            m1 = self.m1_count
            m2 = self.m2_count
            d1 = self.m1_dir
            d2 = self.m2_dir

        delta1 = m1 - self.m1_prev_count
        delta2 = m2 - self.m2_prev_count
        self.m1_prev_count = m1
        self.m2_prev_count = m2

        # 회전수 [rev/s]
        rev1 = delta1 / TICKS_PER_REV / self.dt
        rev2 = delta2 / TICKS_PER_REV / self.dt

        # 속도 [m/s] (전진 기준 절대값 사용)
        self.meas_v_left  = rev1 * WHEEL_CIRCUMFERENCE * (1 if d1 >= 0 else -1)
        self.meas_v_right = rev2 * WHEEL_CIRCUMFERENCE * (1 if d2 >= 0 else -1)
        v_left  = abs(self.meas_v_left)
        v_right = abs(self.meas_v_right)

        # 2) PID 제어 (모터별)
        raw_pwm_left  = self.pid_left.update(self.target_speed, v_left, self.dt)
        raw_pwm_right = self.pid_right.update(self.target_speed, v_right, self.dt)

        # 최소 PWM 적용 (목표 속도가 0이 아닐 때만)
        if self.target_speed > 0.0:
            if raw_pwm_left > 0.0:
                raw_pwm_left = max(raw_pwm_left, self.min_pwm)
            if raw_pwm_right > 0.0:
                raw_pwm_right = max(raw_pwm_right, self.min_pwm)

        # PWM 변화율 제한 (부드럽게)
        self.current_pwm_left  = self.smooth_pwm(self.current_pwm_left,  raw_pwm_left)
        self.current_pwm_right = self.smooth_pwm(self.current_pwm_right, raw_pwm_right)

        # 모터 구동
        self.apply_pwm(self.current_pwm_left, self.current_pwm_right)

        if self.debug:
            self.get_logger().info(
                f"[PID] vL={v_left:.4f}  vR={v_right:.4f}  "
                f"target={self.target_speed:.4f}  "
                f"PWM_L={self.current_pwm_left:.1f}  PWM_R={self.current_pwm_right:.1f}"
            )

        # 3) 그래프 업데이트
        if self.plot_flag:
            self.update_plot(v_left, v_right)

    # ---------- 그래프 업데이트 ----------
    def update_plot(self, v_left, v_right):
        t_now = time.time() - self.plot_start_time
        self.time_hist.append(t_now)

        self.target_speed_hist.append(self.target_speed)
        self.left_speed_hist.append(v_left)
        self.right_speed_hist.append(v_right)
        self.left_pwm_hist.append(self.current_pwm_left)
        self.right_pwm_hist.append(self.current_pwm_right)

        # 최근 200포인트만 유지
        max_len = 200
        if len(self.time_hist) > max_len:
            self.time_hist         = self.time_hist[-max_len:]
            self.target_speed_hist = self.target_speed_hist[-max_len:]
            self.left_speed_hist   = self.left_speed_hist[-max_len:]
            self.right_speed_hist  = self.right_speed_hist[-max_len:]
            self.left_pwm_hist     = self.left_pwm_hist[-max_len:]
            self.right_pwm_hist    = self.right_pwm_hist[-max_len:]

        # 속도 그래프
        self.line_target_speed.set_data(self.time_hist, self.target_speed_hist)
        self.line_left_speed.set_data(self.time_hist, self.left_speed_hist)
        self.line_right_speed.set_data(self.time_hist, self.right_speed_hist)

        # y축 범위 자동 조정
        if len(self.left_speed_hist) > 5:
            v_min = min(min(self.left_speed_hist),
                        min(self.right_speed_hist),
                        min(self.target_speed_hist))
            v_max = max(max(self.left_speed_hist),
                        max(self.right_speed_hist),
                        max(self.target_speed_hist))
            margin = max(0.01, 0.2 * (v_max - v_min))
            self.ax_speed.set_ylim(v_min - margin, v_max + margin)

        # PWM 그래프
        self.line_left_pwm.set_data(self.time_hist, self.left_pwm_hist)
        self.line_right_pwm.set_data(self.time_hist, self.right_pwm_hist)
        self.ax_pwm.set_xlim(max(0, self.time_hist[-1] - 10), self.time_hist[-1] + 0.1)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    # ---------- 모터 출력 ----------
    def apply_pwm(self, pwm_left: float, pwm_right: float):
        """
        pwm_left, pwm_right : 0~100 [%]
        두 바퀴 모두 같은 방향(정회전)으로 구동
        EN 핀 active-low / active-high 여부는
        네 보드 특성에 따라 LOW/HIGH를 바꿔야 할 수 있음.
        """
        pwm_left  = max(0.0, min(100.0, pwm_left))
        pwm_right = max(0.0, min(100.0, pwm_right))

        if pwm_left <= 0.0 and pwm_right <= 0.0:
            # 둘 다 정지
            GPIO.output(MOTOR1_EN, GPIO.LOW)
            GPIO.output(MOTOR2_EN, GPIO.LOW)
            self.pwm1.ChangeDutyCycle(0.0)
            self.pwm2.ChangeDutyCycle(0.0)
        else:
            # 정회전 (필요하면 DIR HIGH/LOW 바꿔서 방향 맞추기)
            GPIO.output(MOTOR1_EN, GPIO.LOW)
            GPIO.output(MOTOR2_EN, GPIO.LOW)
            GPIO.output(MOTOR1_DIR, GPIO.LOW)
            GPIO.output(MOTOR2_DIR, GPIO.LOW)
            self.pwm1.ChangeDutyCycle(pwm_left)
            self.pwm2.ChangeDutyCycle(pwm_right)

    # ---------- 종료 처리 ----------
    def cleanup(self):
        try:
            GPIO.remove_event_detect(MOTOR1_HALL_A)
            GPIO.remove_event_detect(MOTOR1_HALL_B)
            GPIO.remove_event_detect(MOTOR2_HALL_A)
            GPIO.remove_event_detect(MOTOR2_HALL_B)
        except Exception:
            pass

        try:
            self.pwm1.stop()
            self.pwm2.stop()
        except Exception:
            pass

        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RB35SpeedPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

