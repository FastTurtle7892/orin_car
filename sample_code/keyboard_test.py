import sys
import tty
import termios
from adafruit_motor import motor
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import time

# --- 기존 클래스 유지 ---
class PWMThrottleHat:
    def __init__(self, pwm, channel):
        self.pwm = pwm
        self.channel = channel
        self.pwm.frequency = 60

    def set_throttle(self, throttle):
        pulse = int(0xFFFF * abs(throttle))
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

# --- 하드웨어 설정 ---
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 60

motor_hat = PWMThrottleHat(pca, channel=0)
kit = ServoKit(channels=16, i2c=i2c, address=0x60)

# 초기값 설정
pan = 100
kit.servo[0].angle = pan
current_throttle = 0

# --- 핵심: SSH에서 키 입력 한 글자씩 받아오는 함수 ---
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("========================================")
print("   SSH 원격 로봇 제어기 (엔터 필요 없음)   ")
print("========================================")
print(" [ W ] : 전진 (속도 증가)")
print(" [ S ] : 후진 (속도 증가)")
print(" [Space]: 정지 (즉시 멈춤)")
print(" [ A ] : 좌회전 (10도)")
print(" [ D ] : 우회전 (10도)")
print(" [ Q ] : 종료")
print("========================================")

try:
    while True:
        # 키보드 입력을 기다림 (누르는 순간 반응)
        key = getch()

        if key == 'w':
            current_throttle = -0.5 # 전진 속도 고정 (필요시 조절)
            print(f"\rForward ({current_throttle})", end="")
            motor_hat.set_throttle(current_throttle)
            
        elif key == 's':
            current_throttle = 0.5 # 후진 속도 고정
            print(f"\rBackward ({current_throttle})", end="")
            motor_hat.set_throttle(current_throttle)

        elif key == ' ' or key == 'x': # 스페이스바 또는 x로 정지
            current_throttle = 0
            print(f"\rStop                    ", end="")
            motor_hat.set_throttle(0)

        elif key == 'a':
            pan -= 10
            if pan < 0: pan = 0
            kit.servo[0].angle = pan
            print(f"\rLeft: {pan}             ", end="")
        
        elif key == 'd':
            pan += 10
            if pan > 180: pan = 180
            kit.servo[0].angle = pan
            print(f"\rRight: {pan}            ", end="")

        elif key == 'q':
            print("\nQuitting...")
            break
            
except KeyboardInterrupt:
    pass

finally:
    # 종료 시 안전하게 정지
    motor_hat.set_throttle(0)
    kit.servo[0].angle = 100
    pca.deinit()
    print("\nProgram Cleaned up.")
