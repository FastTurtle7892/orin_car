#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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

        # 1. 차량 하드웨어 스펙
        self.wheelbase = 0.145        
        self.max_steering_deg = 40.0 
        
        # 2. 서보/모터 채널 설정
        self.servo_channel = 0      # 조향
        self.motor_channel = 0      # 모터
        self.center_angle = 100.0   # 조향 중심값
        
        self.lift_channel = 1       # 리프트
        self.gripper_channel = 2    # 그리퍼
        
        # ==========================================
        # [수정 완료] 테스트로 확인된 각도 값 적용
        # ==========================================
        self.LIFT_UP = 180.0        # 초기/복귀 (올림)
        self.LIFT_DOWN = 170.0      # 잡으러 갈 때 (내림)
        self.GRIP_CLOSE = 100.0     # 잡기
        self.GRIP_OPEN = 50.0       # 풀기 (초기)
        
        self.hardware_connected = False
        self.pca = None
        self.kit = None

        # 3. 토픽 구독
        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.create_subscription(String, '/gripper_cmd', self.gripper_callback, 10)
        
        self.get_logger().info("✅ Physics-based Ackermann Driver + Gripper Online")

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
            
            # [중요] 주소 0x60 적용
            self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            
            # [초기화] 
            # 1. 조향
            self.kit.servo[self.servo_channel].angle = self.center_angle
            # 2. 모터
            self.set_throttle_hardware(0.0)
            
            # 3. 리프트 & 그리퍼 초기 상태 (UP & OPEN)
            # 테스트 코드의 초기 상태: Lift=180, Grip=50
            self.kit.servo[self.lift_channel].angle = self.LIFT_UP
            self.kit.servo[self.gripper_channel].angle = self.GRIP_OPEN
            
            self.hardware_connected = True
            self.get_logger().info(f"🔌 Connected (Addr:0x60) | Lift:{self.LIFT_UP}, Grip:{self.GRIP_OPEN}")
        except Exception as e:
            self.get_logger().error(f"❌ Hardware Error: {e}")

    def gripper_callback(self, msg):
        if not self.hardware_connected: return

        cmd = msg.data.upper()
        self.get_logger().info(f"🦾 Gripper Cmd: {cmd}")

        try:
            if cmd == "UP":
                self.kit.servo[self.lift_channel].angle = self.LIFT_UP
            elif cmd == "DOWN":
                self.kit.servo[self.lift_channel].angle = self.LIFT_DOWN
            elif cmd == "GRIP":
                self.kit.servo[self.gripper_channel].angle = self.GRIP_CLOSE
            elif cmd == "RELEASE":
                self.kit.servo[self.gripper_channel].angle = self.GRIP_OPEN
        except Exception as e:
            self.get_logger().error(f"Gripper Servo Error: {e}")

    def listener_callback(self, msg):
        if not self.hardware_connected: return

        v = msg.linear.x 
        w = msg.angular.z 
        
        if abs(v) < 0.01: 
            steering_angle_rad = 0.0 
        else:
            steering_angle_rad = math.atan((w * self.wheelbase) / v)
        
        steering_angle_deg = math.degrees(steering_angle_rad)
        steering_angle_deg = max(-self.max_steering_deg, min(self.max_steering_deg, steering_angle_deg))
        
        current_center = 100.0
        left_gain = 1.4  
        right_gain = 1.2 

        if steering_angle_deg > 0: 
            target_servo_angle = current_center - (steering_angle_deg * left_gain)
        else: 
            target_servo_angle = current_center - (steering_angle_deg * right_gain)
            
        target_servo_angle = max(0, min(180, target_servo_angle))
        
        try:
            self.kit.servo[self.servo_channel].angle = target_servo_angle
            self.set_throttle_hardware(-v) 
        except Exception as e:
            self.get_logger().warn(f"Ctrl Error: {e}")

    def set_throttle_hardware(self, throttle):
        if not self.pca: return
        
        throttle = max(-0.8, min(0.8, throttle))
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

    def stop_robot(self):
        if self.hardware_connected:
            try:
                self.set_throttle_hardware(0)
                self.kit.servo[self.servo_channel].angle = self.center_angle
                # 종료 시 안전하게 초기 위치(UP & OPEN)로 복귀
                self.kit.servo[self.lift_channel].angle = self.LIFT_UP
                self.kit.servo[self.gripper_channel].angle = self.GRIP_OPEN
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
