#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
encoder_distance_calib.py

- RB-35GM 11TYPE (Encoder 26P/R) + 감속비 고려
- 라즈베리파이4 + Ubuntu 22.04
- RPi.GPIO 단독 테스트용 (ROS 사용 X)
- 좌우 바퀴(MOTOR1, MOTOR2)를 일정 시간 구동하고,
  엔코더 카운트로 계산한 이동거리와
  실제로 자로 잰 이동거리를 비교해서
  휠 지름 보정에 쓰는 데이터를 얻는다.
"""

import math
import time
from threading import Lock

import RPi.GPIO as GPIO

# ==================== 하드웨어 핀 정의 (BOARD 모드) ====================
# 왼쪽 모터
MOTOR1_DIR = 40
MOTOR1_PWM = 32
MOTOR1_EN  = 37
MOTOR1_HALL_A = 35
MOTOR1_HALL_B = 36

# 오른쪽 모터 (motor.py와 동일하게)
MOTOR2_DIR = 18
MOTOR2_PWM = 12
MOTOR2_EN  = 15
MOTOR2_HALL_A = 13
MOTOR2_HALL_B = 16

# ==================== 엔코더/기구 파라미터 ====================
ENCODER_PPR    = 26.0      # 모터축 P/R
GEAR_RATIO     = 60.0      # ★ motor.py와 동일하게
WHEEL_DIAMETER = 0.115     # [m] 바퀴 지름 (motor.py와 동일하게 넣기!)

MOTOR_TICKS_PER_REV = ENCODER_PPR * 4.0
TICKS_PER_REV       = MOTOR_TICKS_PER_REV * GEAR_RATIO
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER


# ==================== X4 디코딩 함수 ====================
def decode_x4(prev_a, prev_b, curr_a, curr_b):
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


# ==================== 전역 변수 ====================
enc_lock = Lock()

m1_count = 0
m1_dir = 0
m1_prev_a = 0
m1_prev_b = 0

m2_count = 0
m2_dir = 0
m2_prev_a = 0
m2_prev_b = 0


# ==================== 콜백 함수 ====================
def motor1_callback(channel):
    global m1_count, m1_dir, m1_prev_a, m1_prev_b

    curr_a = GPIO.input(MOTOR1_HALL_A)
    curr_b = GPIO.input(MOTOR1_HALL_B)
    delta, direction = decode_x4(m1_prev_a, m1_prev_b, curr_a, curr_b)

    if delta:
        with enc_lock:
            m1_count += 1
            m1_dir = direction

    m1_prev_a = curr_a
    m1_prev_b = curr_b


def motor2_callback(channel):
    global m2_count, m2_dir, m2_prev_a, m2_prev_b

    curr_a = GPIO.input(MOTOR2_HALL_A)
    curr_b = GPIO.input(MOTOR2_HALL_B)
    delta, direction = decode_x4(m2_prev_a, m2_prev_b, curr_a, curr_b)

    if delta:
        with enc_lock:
            m2_count += 1
            m2_dir = direction

    m2_prev_a = curr_a
    m2_prev_b = curr_b


# ==================== GPIO 초기화 ====================
def setup_gpio(pwm_duty):
    global m1_prev_a, m1_prev_b, m2_prev_a, m2_prev_b

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # 모터1
    GPIO.setup(MOTOR1_DIR, GPIO.OUT)
    GPIO.setup(MOTOR1_PWM, GPIO.OUT)
    GPIO.setup(MOTOR1_EN,  GPIO.OUT)
    GPIO.setup(MOTOR1_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(MOTOR1_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # 모터2
    GPIO.setup(MOTOR2_DIR, GPIO.OUT)
    GPIO.setup(MOTOR2_PWM, GPIO.OUT)
    GPIO.setup(MOTOR2_EN,  GPIO.OUT)
    GPIO.setup(MOTOR2_HALL_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(MOTOR2_HALL_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # 초기 상태
    GPIO.output(MOTOR1_EN, GPIO.LOW)
    GPIO.output(MOTOR1_DIR, GPIO.LOW)  # LOW = 전진 방향이라고 가정
    GPIO.output(MOTOR2_EN, GPIO.LOW)
    GPIO.output(MOTOR2_DIR, GPIO.LOW)

    # 엔코더 이전 상태
    m1_prev_a = GPIO.input(MOTOR1_HALL_A)
    m1_prev_b = GPIO.input(MOTOR1_HALL_B)
    m2_prev_a = GPIO.input(MOTOR2_HALL_A)
    m2_prev_b = GPIO.input(MOTOR2_HALL_B)

    # 인터럽트 등록
    GPIO.add_event_detect(MOTOR1_HALL_A, GPIO.BOTH,
                          callback=motor1_callback, bouncetime=1)
    GPIO.add_event_detect(MOTOR1_HALL_B, GPIO.BOTH,
                          callback=motor1_callback, bouncetime=1)
    GPIO.add_event_detect(MOTOR2_HALL_A, GPIO.BOTH,
                          callback=motor2_callback, bouncetime=1)
    GPIO.add_event_detect(MOTOR2_HALL_B, GPIO.BOTH,
                          callback=motor2_callback, bouncetime=1)

    # PWM 설정
    pwm1 = GPIO.PWM(MOTOR1_PWM, 1000)  # 1kHz
    pwm2 = GPIO.PWM(MOTOR2_PWM, 1000)
    pwm1.start(0.0)
    pwm2.start(0.0)

    print("[INFO] GPIO initialized. PWM duty = {:.1f}%".format(pwm_duty))
    return pwm1, pwm2


def cleanup(pwm1, pwm2):
    try:
        pwm1.stop()
        pwm2.stop()
    except Exception:
        pass

    try:
        GPIO.remove_event_detect(MOTOR1_HALL_A)
        GPIO.remove_event_detect(MOTOR1_HALL_B)
        GPIO.remove_event_detect(MOTOR2_HALL_A)
        GPIO.remove_event_detect(MOTOR2_HALL_B)
    except Exception:
        pass

    GPIO.cleanup()
    print("[INFO] GPIO cleaned up.")


# ==================== 메인 테스트 루틴 ====================
def main():
    global m1_count, m1_dir, m2_count, m2_dir

    # ===== 설정값 =====
    pwm_duty = 30.0       # 모터를 돌릴 PWM [%] (평지 base_pwm 근처)
    run_time = 3.0        # [s] 구동 시간 (2~5초 정도)

    print("=== 엔코더 거리 스케일 테스트 (양쪽 바퀴) ===")
    print(f"ENCODER_PPR={ENCODER_PPR}, GEAR_RATIO={GEAR_RATIO}, "
          f"WHEEL_DIAMETER={WHEEL_DIAMETER} m")
    print(f"TICKS_PER_REV={TICKS_PER_REV}, WHEEL_CIRCUMFERENCE={WHEEL_CIRCUMFERENCE:.4f} m")

    pwm1, pwm2 = setup_gpio(pwm_duty)

    try:
        input("엔터를 누르면 양쪽 모터가 전진합니다. 시작점에 표시해두고 준비하세요...")

        # 카운트 초기화
        with enc_lock:
            m1_count = 0
            m1_dir = 0
            m2_count = 0
            m2_dir = 0

        # 방향 설정 (전진)
        GPIO.output(MOTOR1_DIR, GPIO.LOW)  # 필요시 HIGH로 전/후진 방향 확인
        GPIO.output(MOTOR2_DIR, GPIO.LOW)

        # ★ motor.py에서 EN 핀을 어떻게 쓰는지에 따라 HIGH/LOW 조정 필요
        # 지금 motor.py에서 EN을 LOW로 두고도 모터가 돈다면 아래 그대로 두고,
        # "EN HIGH일 때만 도는" 보드라면 아래를 HIGH로 바꿔줘야 함.
        GPIO.output(MOTOR1_EN, GPIO.LOW)
        GPIO.output(MOTOR2_EN, GPIO.LOW)

        pwm1.ChangeDutyCycle(pwm_duty)
        pwm2.ChangeDutyCycle(pwm_duty)

        print(f"[RUN] {run_time}초 동안 모터 구동 중...")
        t_start = time.time()

        while time.time() - t_start < run_time:
            time.sleep(0.01)

        # 정지
        pwm1.ChangeDutyCycle(0.0)
        pwm2.ChangeDutyCycle(0.0)
        GPIO.output(MOTOR1_EN, GPIO.LOW)
        GPIO.output(MOTOR2_EN, GPIO.LOW)
        print("[RUN] 정지 완료. 엔코더 카운트 읽는 중...")

        # 결과 계산
        with enc_lock:
            total_ticks_1 = m1_count
            last_dir_1 = m1_dir
            total_ticks_2 = m2_count
            last_dir_2 = m2_dir

        print(f"[RESULT] 모터1: {total_ticks_1} ticks (마지막 방향: {last_dir_1})")
        print(f"[RESULT] 모터2: {total_ticks_2} ticks (마지막 방향: {last_dir_2})")

        # tick -> 회전수 -> 거리
        rev1 = total_ticks_1 / TICKS_PER_REV
        rev2 = total_ticks_2 / TICKS_PER_REV

        dist1 = rev1 * WHEEL_CIRCUMFERENCE
        dist2 = rev2 * WHEEL_CIRCUMFERENCE
        dist_avg = (dist1 + dist2) / 2.0

        print(f"[RESULT] 모터1 회전수: {rev1:.4f} rev, 거리: {dist1:.4f} m")
        print(f"[RESULT] 모터2 회전수: {rev2:.4f} rev, 거리: {dist2:.4f} m")
        print(f"[RESULT] 코드 기준 평균 이동거리(로봇 전진 거리): {dist_avg:.4f} m")

        # ===== 사용자에게 실제 거리 입력받기 =====
        try:
            real_dist = float(input("실제로 자로 잰 로봇 이동거리[m]를 입력하세요 (예: 0.95): "))
            if real_dist > 0.0:
                distance = dist_avg
                scale = real_dist / distance if distance > 1e-6 else 1.0
                new_wheel_diameter = WHEEL_DIAMETER * scale

                print("\n=== 보정 제안 ===")
                print(f"- 실제 / 코드 평균거리 비율(scale) = {scale:.4f}")
                print(f"- 현재 WHEEL_DIAMETER = {WHEEL_DIAMETER:.6f} m")
                print(f"- 추천 새로운 WHEEL_DIAMETER = {new_wheel_diameter:.6f} m")
                print("\n이 값을 motor.py의 WHEEL_DIAMETER에 반영한 뒤,")
                print("다시 이 테스트를 실행해서 1m에 더 가깝게 맞추면 됩니다.")
            else:
                print("[WARN] 실제 거리가 0 이하로 입력됨. 스케일 보정은 건너뜁니다.")
        except ValueError:
            print("[WARN] 숫자가 아닌 값이 입력됨. 스케일 보정은 건너뜁니다.")

    finally:
        cleanup(pwm1, pwm2)


if __name__ == "__main__":
    main()
