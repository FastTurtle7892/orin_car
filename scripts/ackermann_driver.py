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

        # 1. 차량 하드웨어 스펙 (URDF와 일치해야 함)
        self.wheelbase = 0.27        # 축간거리 (m)
        self.max_steering_deg = 40.0 # 최대 조향각 (도)
        
        # 2. 서보/모터 설정
        self.servo_channel = 0
        self.motor_channel = 0
        self.center_angle = 100.0    # 서보 중심값
        
        self.hardware_connected = False
        self.pca = None
        self.kit = None

        # 3. cmd_vel 구독
        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.get_logger().info("✅ Physics-based Ackermann Driver Online")

        # 4. 하드웨어 연결
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
            
            # 초기화
            self.kit.servo[self.servo_channel].angle = self.center_angle
            self.set_throttle_hardware(0.0)
            
            self.hardware_connected = True
            self.get_logger().info("🔌 Hardware Connected!")
        except Exception as e:
            self.get_logger().error(f"❌ Hardware Error: {e}")

    def listener_callback(self, msg):
        if not self.hardware_connected: return

        v = msg.linear.x  # 선속도 (m/s)
        w = msg.angular.z # 회전속도 (rad/s)
        
        # [핵심] 아커만 조향 공식 (Bicycle Model)
        # delta = arctan( (w * L) / v )
        
        if abs(v) < 0.01: 
            # 정지 상태에서는 조향 유지 (또는 0으로)
            steering_angle_rad = 0.0 
        else:
            steering_angle_rad = math.atan((w * self.wheelbase) / v)
        
        # 라디안 -> 도 변환
        steering_angle_deg = math.degrees(steering_angle_rad)
        
        # [중요] 최대 조향각 제한 (50도)
        steering_angle_deg = max(-self.max_steering_deg, min(self.max_steering_deg, steering_angle_deg))
        
        current_center = 100.0  # 사용자 설정값
        
        # 비율 계산 (Nav2 최대각 50도 기준)
        # 왼쪽으로 갈 때: 100 -> 30 (변화량 70) => 비율 1.4
        # 오른쪽으로 갈 때: 100 -> 160 (변화량 60) => 비율 1.2
        left_gain = 1.4  
        right_gain = 1.2 

        if steering_angle_deg > 0: # 왼쪽 회전 (Positive)
            # 100 - (각도 * 1.4) -> 50도일 때 100 - 70 = 30
            target_servo_angle = current_center - (steering_angle_deg * left_gain)
        else: # 오른쪽 회전 (Negative)
            # 100 - (각도 * 1.2) -> -50도일 때 100 - (-60) = 160
            target_servo_angle = current_center - (steering_angle_deg * right_gain)
            
        # 서보 안전 범위 (0~180)
        target_servo_angle = max(0, min(180, target_servo_angle))
        
        try:
            self.kit.servo[self.servo_channel].angle = target_servo_angle
            # 모터 제어 (- 붙여야 전진이면 유지, 아니면 제거)
            self.set_throttle_hardware(-v) 
        except Exception as e:
            self.get_logger().warn(f"Ctrl Error: {e}")

    def set_throttle_hardware(self, throttle):
        if not self.pca: return
        
        # 속도 제한 (-0.8 ~ 0.8)
        throttle = max(-0.8, min(0.8, throttle))
        pulse = int(0xFFFF * abs(throttle))
        
        in1 = self.motor_channel + 5
        in2 = self.motor_channel + 4
        in3 = self.motor_channel + 3

        if abs(throttle) < 0.05: # 정지 Deadzone
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
