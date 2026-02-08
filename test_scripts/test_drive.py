#!/usr/bin/env python3
import time
import math
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit

class DirectComplexTest:
    def __init__(self):
        print("🔌 하드웨어 연결 중...")
        
        # 1. I2C 및 PCA9685 초기화
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 60
        self.kit = ServoKit(channels=16, i2c=self.i2c, address=0x60)
        
        # 2. 채널 설정
        self.servo_channel = 0
        self.motor_channel = 0
        
        # 3. 조향 설정
        self.center_angle = 100.0
        self.max_steering_deg = 50.0

        # 초기화
        self.kit.servo[self.servo_channel].angle = self.center_angle
        self.set_throttle(0.0)
        print("✅ 하드웨어 준비 완료")

        # ==========================================
        # 🚀 [핵심] 복합 궤적 시퀀스 정의
        # 형식: (조향각, 시간, 방향, 시작속도, 끝속도)
        # 방향: 1(전진), -1(후진)
        # ==========================================
        self.sequences = [
            # 1. P4 구간: 완만한 우회전
            [
                (0.0, 4, 1, 2.0, 1.5),
                (-10.0, 4.5, 1, 1.5, 0.0)
                
            ],
            
            # 2. P5 구간: 후진 (회전 -> 직진 -> 회전)
            [
                (40.0, 3.5, -1, 2.0, 1.5),  # 진입 회전
                (0.0,  2.0, -1, 1.5, 1.0)  # 중간 직진
            ],
            
            #3. P6 구간: 전진 (직진 -> 급우회전)
            [

                (-11.0, 14.0, 1, 1.2, 0.0)   # 꺾어서 진입
            ],
            
            # 4. P7 구간: 단순 직진
            [
                (0.0, 4.0, 1, 2.0, 0.0)
            ]
        ]

    def set_steering(self, deg):
        """ 각도(degree)를 서보 모터 값으로 변환 """
        deg = max(-self.max_steering_deg, min(self.max_steering_deg, deg))
        if deg > 0:
            target = self.center_angle - (deg * 1.4)
        else:
            target = self.center_angle - (deg * 1.2)
        target = max(0, min(180, target))
        self.kit.servo[self.servo_channel].angle = target

    def set_throttle(self, throttle):
        """ DC 모터 PWM 제어 """
        throttle = max(-0.4, min(0.4, throttle)) # 안전 제한
        pulse = int(0xFFFF * abs(throttle))
        
        in1 = self.motor_channel + 5
        in2 = self.motor_channel + 4
        in3 = self.motor_channel + 3

        if abs(throttle) < 0.05:
            self.pca.channels[in1].duty_cycle = 0
            self.pca.channels[in2].duty_cycle = 0
            self.pca.channels[in3].duty_cycle = 0
        elif throttle > 0:
            self.pca.channels[in1].duty_cycle = pulse
            self.pca.channels[in2].duty_cycle = 0
            self.pca.channels[in3].duty_cycle = 0xFFFF
        else:
            self.pca.channels[in1].duty_cycle = pulse
            self.pca.channels[in2].duty_cycle = 0xFFFF
            self.pca.channels[in3].duty_cycle = 0

    def run_full_test(self):
        print("🚀 복합 궤적 테스트 시작 (Ctrl+C로 중단)")
        
        try:
            for idx, steps in enumerate(self.sequences):
                print(f"\n▶ [구간 {idx+1}] 시작 ({len(steps)} 단계)")
                
                for step_i, (deg, dur, direct, v_start, v_end) in enumerate(steps):
                    print(f"   ㄴ Step {step_i+1}: 각도 {deg}°, {dur}초, 속도 {v_start}->{v_end}")
                    
                    self.set_steering(deg)
                    self.run_ramp_motion(dur, direct, v_start, v_end)
                
                # 구간 사이 잠시 정지
                self.set_throttle(0.0)
                time.sleep(1.0)
                
            print("\n🏁 모든 테스트 완료")

        except KeyboardInterrupt:
            print("\n⚠️ 사용자 중단!")
        except Exception as e:
            print(f"\n❌ 오류 발생: {e}")
        finally:
            self.set_throttle(0.0)
            self.pca.deinit()

    def run_ramp_motion(self, duration, direction, start_speed, end_speed):
        """ 속도를 부드럽게 변화시키며 주행 """
        rate_hz = 50 # 부드러운 제어를 위해 Hz 상향
        steps = int(duration * rate_hz)
        dt = 1.0 / rate_hz
        
        for i in range(steps):
            # 선형 보간 (Linear Interpolation)
            alpha = i / float(steps)
            current_speed_mag = start_speed * (1.0 - alpha) + end_speed * alpha
            
            # ROS Driver 로직 역산: set_throttle(-v)
            # 전진(dir=1) -> v양수 -> throttle 음수
            # 후진(dir=-1) -> v음수 -> throttle 양수
            v = current_speed_mag * direction
            hw_input = -v 
            
            self.set_throttle(hw_input)
            time.sleep(dt)

if __name__ == '__main__':
    tester = DirectComplexTest()
    tester.run_full_test()