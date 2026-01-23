#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState  # [추가] 관절 상태 메시지
from adafruit_pca9685 import PCA9685
from adafruit_servokit import ServoKit
import board
import busio
import math
import time

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
        self.max_turn_angle = 35.0  # 최대 꺾임 각도 (도 단위)
        
        # 현재 상태 저장 변수 (JointState 발행용)
        self.current_steering_angle_rad = 0.0
        self.current_speed = 0.0
        self.wheel_position = 0.0  # 바퀴 회전 누적값 (가상)

        # 초기화
        self.kit.servo[self.servo_channel].angle = self.center_angle
        self.set_throttle(0.0)

        # 3. 구독 설정
        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

        # 4. [추가] JointState 퍼블리셔 설정 (30Hz)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.033, self.publish_joint_states)
        
        self.get_logger().info("Ackermann Driver Ready with JointStates")

    def listener_callback(self, msg):
        # === [1] 조향 제어 (Steering) ===
        # 입력값(-1.0 ~ 1.0) * 최대각도(35도)
        steering_offset_deg = msg.angular.z * self.max_turn_angle

        # ROS 좌표계: 좌회전(+) -> 각도 감소 / 우회전(-) -> 각도 증가 (하드웨어 서보 기준)
        target_angle = self.center_angle - steering_offset_deg

        # 서보 보호
        limit_min = self.center_angle - self.max_turn_angle
        limit_max = self.center_angle + self.max_turn_angle
        target_angle = max(limit_min, min(limit_max, target_angle))

        self.kit.servo[self.servo_channel].angle = target_angle

        # [추가] JointState용 라디안 각도 저장
        # URDF 상에서는 좌회전이 (+), 우회전이 (-)인 경우가 많으므로 부호 확인 필요
        # 보통 msg.angular.z가 (+)면 좌회전이므로 그대로 라디안 변환
        self.current_steering_angle_rad = math.radians(steering_offset_deg)

        # === [2] 속도 제어 (Throttle) ===
        self.set_throttle(-msg.linear.x)
        
        # [추가] 현재 속도 저장 (바퀴 회전 시각화용)
        self.current_speed = msg.linear.x

    def set_throttle(self, throttle):
        # 속도 제한
        max_speed = 0.6
        throttle = max(-max_speed, min(max_speed, throttle))

        pulse = int(0xFFFF * abs(throttle))

        # 핀 매핑
        in1 = self.motor_channel + 5  # PWM/ENA
        in2 = self.motor_channel + 4  # IN1
        in3 = self.motor_channel + 3  # IN2

        if abs(throttle) < 0.05: # 정지
            self.pca.channels[in1].duty_cycle = 0
            self.pca.channels[in2].duty_cycle = 0
            self.pca.channels[in3].duty_cycle = 0
        elif throttle > 0: # 전진
            self.pca.channels[in1].duty_cycle = pulse
            self.pca.channels[in2].duty_cycle = 0
            self.pca.channels[in3].duty_cycle = 0xFFFF
        else: # 후진
            self.pca.channels[in1].duty_cycle = pulse
            self.pca.channels[in2].duty_cycle = 0xFFFF
            self.pca.channels[in3].duty_cycle = 0

    def publish_joint_states(self):
        # JointState 메시지 생성
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # URDF(robot.xacro)에 정의된 관절 이름과 정확히 일치해야 합니다.
        msg.name = [
            'front_left_steering_joint',  # 앞바퀴 조향 (좌)
            'front_right_steering_joint', # 앞바퀴 조향 (우)
            'front_left_wheel_joint',  # 앞바퀴 회전
            'front_right_wheel_joint', # 앞바퀴 회전
            'rear_left_wheel_joint',   # 뒷바퀴 회전
            'rear_right_wheel_joint'   # 뒷바퀴 회전
        ]

        # 바퀴 회전 시각화 (가상 적분)
        # dt = 0.033 (30Hz), 바퀴 반지름 약 0.05m 가정 시 각속도 계산 등은 생략하고 단순 비례
        self.wheel_position += self.current_speed * 0.1 

        msg.position = [
            self.current_steering_angle_rad,  # fl_steer
            self.current_steering_angle_rad,  # fr_steer (아커만 기하학 무시하고 단순 동일 각도 적용)
            self.wheel_position,              # fl_wheel
            self.wheel_position,              # fr_wheel
            self.wheel_position,              # rl_wheel
            self.wheel_position               # rr_wheel
        ]
        
        self.joint_pub.publish(msg)

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
