#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')
        
        # 1. 하드웨어 초기화
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = 60
            self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            self.get_logger().info("PCA9685 Initialized at 0x60")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Hardware: {e}")
            return

        # 2. 서보 및 모터 채널 설정
        self.servo_channel = 0
        self.motor_channel = 0
        
        # === [중요] 조향 각도 설정 ===
        self.center_angle = 100.0   # 정면 (바퀴 정렬)
        self.max_turn_angle = 25.0  # 최대 꺾임 각도 (키보드 꽉 눌렀을 때)
        # 결과: 좌 65도 ~ 우 135도 사이에서 움직임
        
        # 초기화
        self.kit.servo[self.servo_channel].angle = self.center_angle
        self.set_throttle(0.0)

        # 3. 구독 설정
        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.get_logger().info("Ackermann Driver Ready (Proportional Mode)")

    def listener_callback(self, msg):
        # === [1] 조향 제어 (Steering) ===
        # Nav2와 호환성을 위해 '비례 제어' 방식을 사용합니다.
        # teleop_twist_keyboard는 angular.z 값을 1.0으로 보냅니다.
        # Nav2는 0.1, 0.5 등 다양한 값을 보냅니다.
        
        # 입력값(-1.0 ~ 1.0) * 최대각도(35도)
        steering_offset = msg.angular.z * self.max_turn_angle
        
        # ROS 좌표계: 좌회전(+) -> 각도 감소 / 우회전(-) -> 각도 증가
        target_angle = self.center_angle - steering_offset

        # 서보 보호 (물리적 한계 제한)
        # 서보가 0도나 180도를 넘어가서 끼기기긱 소리 나는 것을 방지
        limit_min = self.center_angle - self.max_turn_angle
        limit_max = self.center_angle + self.max_turn_angle
        target_angle = max(limit_min, min(limit_max, target_angle))
        
        self.kit.servo[self.servo_channel].angle = target_angle
        
        # === [2] 속도 제어 (Throttle) ===
        self.set_throttle(-msg.linear.x)

    def set_throttle(self, throttle):
        # 속도 제한 (SLAM 품질을 위해 너무 빠르면 안 됨)
        max_speed = 0.6
        throttle = max(-max_speed, min(max_speed, throttle))
        
        pulse = int(0xFFFF * abs(throttle))
        
        # 핀 매핑 (사용자 하드웨어 기준)
        in1 = self.motor_channel + 5  # PWM/ENA
        in2 = self.motor_channel + 4  # IN1
        in3 = self.motor_channel + 3  # IN2

        if abs(throttle) < 0.05: # 정지 (노이즈 방지용 데드존)
            self.pca.channels[in1].duty_cycle = 0
            self.pca.channels[in2].duty_cycle = 0
            self.pca.channels[in3].duty_cycle = 0
            
        elif throttle < 0: # 전진 (Forward)
            self.pca.channels[in1].duty_cycle = pulse
            self.pca.channels[in2].duty_cycle = 0
            self.pca.channels[in3].duty_cycle = 0xFFFF
            
        else: # 후진 (Backward) - 양수일 때 후진
            self.pca.channels[in1].duty_cycle = pulse
            self.pca.channels[in2].duty_cycle = 0xFFFF
            self.pca.channels[in3].duty_cycle = 0

    def stop_robot(self):
        self.set_throttle(0)
        self.kit.servo[self.servo_channel].angle = self.center_angle
        self.pca.deinit()

def main(args=None):
    rclpy.init(args=args)
    driver = AckermannDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.stop_robot()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
