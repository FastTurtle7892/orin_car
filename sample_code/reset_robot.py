#!/usr/bin/env python3
import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

# ===============================================================
# [핵심] PWMThrottleHat 클래스를 내부에 포함 (Import 에러 방지)
# 출처: motor_test.py
# ===============================================================
class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60 # 주파수 설정

    def set_throttle(self, throttle):
        pulse = int(0xFFFF * abs(throttle)) # 16비트 듀티 사이클 계산

        if throttle < 0: # 전진
            self.pwm.channels[self.channel + 5].duty_cycle = pulse
            self.pwm.channels[self.channel + 4].duty_cycle = 0
            self.pwm.channels[self.channel + 3].duty_cycle = 0xFFFF
        elif throttle > 0: # 후진
            self.pwm.channels[self.channel + 5].duty_cycle = pulse
            self.pwm.channels[self.channel + 4].duty_cycle = 0xFFFF
            self.pwm.channels[self.channel + 3].duty_cycle = 0
        else: # 정지
            self.pwm.channels[self.channel + 5].duty_cycle = 0
            self.pwm.channels[self.channel + 4].duty_cycle = 0
            self.pwm.channels[self.channel + 3].duty_cycle = 0

def reset_robot():
    print("🔄 [RESET] 로봇 모터 초기화 시작...")
    
    # I2C 버스 생성 (하나의 버스를 공유해서 사용)
    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"❌ I2C 버스 초기화 실패: {e}")
        return

    # ================= [1. DC 모터 정지] =================
    try:
        # PCA9685 설정 (DC 모터 제어용)
        pca = PCA9685(i2c)
        pca.frequency = 60
        
        # motor_test.py 기준: channel=0 사용
        motor_hat = PWMThrottleHat(pca, channel=0)
        motor_hat.set_throttle(0) # 정지
        
        print("✅ DC 모터: 정지 완료 (Throttle 0)")
        
    except Exception as e:
        print(f"❌ DC 모터 제어 실패: {e}")

    # ================= [2. 서보 모터 초기화] =================
    try:
        # ServoKit 설정 (서보 제어용, Address 0x60)
        # 이미 생성한 i2c 객체를 재사용하여 충돌 방지
        kit = ServoKit(channels=16, i2c=i2c, address=0x60)

        # [서보 0번] 조향 (Steering) -> 100도 (중앙)
        kit.servo[0].angle = 100
        print("✅ 서보 0 (조향): 100° (중앙)")

        # [서보 1번] 리프트 (Lift) -> 160도 (올림)
        kit.servo[1].angle = 160
        print("✅ 서보 1 (리프트): 160° (UP)")

        # [서보 2번] 그리퍼 (Gripper) -> 70도 (열림)
        kit.servo[2].angle = 70
        print("✅ 서보 2 (그리퍼): 70° (OPEN)")

    except Exception as e:
        print(f"❌ 서보 모터 제어 실패: {e}")

    # 프로그램 종료 전 정리 (필요시)
    try:
        if 'pca' in locals():
            pca.deinit()
    except:
        pass

    print("✨ 모든 모터 초기화 완료!")

if __name__ == "__main__":
    reset_robot()