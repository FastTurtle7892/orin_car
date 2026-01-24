#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time  # [추가] 시간 조작용
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
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

        # 설정
        self.servo_channel = 0
        self.motor_channel = 0
        self.center_angle = 100.0
        self.max_turn_angle = 50.0 
        
        self.current_steering_angle_rad = 0.0
        self.current_speed = 0.0
        self.wheel_position = 0.0
        
        self.hardware_connected = False
        self.pca = None
        self.kit = None

        # [중요] JointState 퍼블리셔
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states) # 20Hz
        self.get_logger().info("✅ Visual System Online")

        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)

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
            
            self.kit.servo[self.servo_channel].angle = self.center_angle
            self.set_throttle_hardware(0.0)
            
            self.hardware_connected = True
            self.get_logger().info("🔌 Hardware Connected!")
        except Exception as e:
            self.get_logger().error(f"❌ Hardware Error: {e}")

    def listener_callback(self, msg):
        steering_offset_deg = msg.angular.z * self.max_turn_angle
        self.current_steering_angle_rad = math.radians(steering_offset_deg)
        self.current_speed = msg.linear.x

        if self.hardware_connected:
            try:
                target = self.center_angle - steering_offset_deg
                target = max(self.center_angle - 50, min(self.center_angle + 50, target))
                self.kit.servo[self.servo_channel].angle = target
                self.set_throttle_hardware(-msg.linear.x)
            except: pass

    def set_throttle_hardware(self, throttle):
        if not self.pca: return
        throttle = max(-0.6, min(0.6, throttle))
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

    def publish_joint_states(self):
        msg = JointState()
        
        # [핵심 수정] 현재 시간보다 0.2초 과거로 시간을 찍어서 보냄 (PC가 무조건 받아줌)
        now_nanos = self.get_clock().now().nanoseconds
        past_time_nanos = now_nanos - 200000000  # 0.2초 뺌
        if past_time_nanos < 0: past_time_nanos = 0
        
        msg.header.stamp = Time(nanoseconds=past_time_nanos).to_msg()
        
        msg.name = [
            'front_left_steering_joint', 'front_right_steering_joint',
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint'
        ]
        self.wheel_position += self.current_speed * 0.1 
        msg.position = [
            self.current_steering_angle_rad, self.current_steering_angle_rad,
            self.wheel_position, self.wheel_position,
            self.wheel_position, self.wheel_position
        ]
        self.joint_pub.publish(msg)

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
