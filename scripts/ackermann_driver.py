#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import threading
import time

# 하드웨어 라이브러리 안전 임포트
try:
    from adafruit_pca9685 import PCA9685
    from adafruit_servokit import ServoKit
    import board
    import busio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False
except Exception:
    HARDWARE_AVAILABLE = False

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        # 1. 하드웨어 설정 변수
        self.servo_channel = 0
        self.motor_channel = 0
        self.center_angle = 100.0
        self.max_turn_angle = 50.0  # 최대 조향각 50도
        
        self.hardware_connected = False
        self.pca = None
        self.kit = None

        # 2. cmd_vel 구독 (이동 명령 수신)
        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.get_logger().info("✅ Motor Control System Online (Visual Disabled)")

        # 3. 하드웨어 연결 (별도 쓰레드)
        if HARDWARE_AVAILABLE:
            self.hw_thread = threading.Thread(target=self.connect_hardware)
            self.hw_thread.daemon = True
            self.hw_thread.start()
        else:
            self.get_logger().warn("⚠️ Simulation Mode (No Hardware Libs)")

    def connect_hardware(self):
        time.sleep(1.0)
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = 60
            self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            
            # 초기화: 정면 정렬 및 정지
            self.kit.servo[self.servo_channel].angle = self.center_angle
            self.set_throttle_hardware(0.0)
            
            self.hardware_connected = True
            self.get_logger().info("🔌 Hardware Connected!")
        except Exception as e:
            self.get_logger().error(f"❌ Hardware Error: {e}")

    def listener_callback(self, msg):
        # 목표 조향각 및 속도 계산
        steering_offset_deg = msg.angular.z * self.max_turn_angle
        
        if self.hardware_connected:
            try:
                # 1. 조향 제어 (Servo)
                target = self.center_angle - steering_offset_deg
                # 서보 보호를 위한 각도 제한 (중심 기준 +-50도)
                target = max(self.center_angle - 50, min(self.center_angle + 50, target))
                self.kit.servo[self.servo_channel].angle = target
                
                # 2. 속도 제어 (DC Motor)
                self.set_throttle_hardware(-msg.linear.x)
            except Exception as e:
                self.get_logger().warn(f"Control Error: {e}")

    def set_throttle_hardware(self, throttle):
        if not self.pca: return
        
        # 속도 제한 (-0.6 ~ 0.6)
        throttle = max(-0.8, min(0.8, throttle))
        pulse = int(0xFFFF * abs(throttle))
        
        in1 = self.motor_channel + 5
        in2 = self.motor_channel + 4
        in3 = self.motor_channel + 3

        if abs(throttle) < 0.05: # 정지 (Deadzone)
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

    def stop_robot(self):
        if self.hardware_connected:
            try:
                self.set_throttle_hardware(0)
                self.kit.servo[self.servo_channel].angle = self.center_angle
                self.pca.deinit()
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
