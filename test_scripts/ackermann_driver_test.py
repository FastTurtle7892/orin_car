#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import threading
import time

try:
    from adafruit_pca9685 import PCA9685
    from adafruit_servokit import ServoKit
    import board
    import busio
    HARDWARE_AVAILABLE = True
except ImportError:
    HARDWARE_AVAILABLE = False

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        self.wheelbase = 0.145        
        self.max_steering_deg = 50.0 
        
        self.servo_channel = 0
        self.motor_channel = 0
        self.center_angle = 100.0
        
        # [수정됨] test_gripper.py 기준 채널 설정
        self.lift_channel = 1     # 팔
        self.gripper_channel = 2  # 집게
        
        # [수정됨] test_gripper.py 기준 각도 설정
        # 초기 상태: Lift=160(위), Gripper=70(열림)
        # 잡기 상태: Lift=140(아래), Gripper=120(닫힘)
        self.LIFT_UP = 160.0      
        self.LIFT_DOWN = 140.0    
        
        self.GRIP_OPEN = 70.0     
        self.GRIP_CLOSE = 120.0   

        # 현재 각도 저장 (초기값은 UP/OPEN으로 가정)
        self.current_lift = self.LIFT_UP
        self.current_grip = self.GRIP_OPEN
        
        self.hardware_connected = False
        self.pca = None
        self.kit = None

        self.create_subscription(Twist, 'cmd_vel', self.listener_callback, 10)
        self.create_subscription(String, '/gripper_cmd', self.gripper_callback, 10)
        
        self.get_logger().info("✅ Ackermann Driver (Updated Angles) Started")

        if HARDWARE_AVAILABLE:
            self.hw_thread = threading.Thread(target=self.connect_hardware)
            self.hw_thread.daemon = True
            self.hw_thread.start()

    def connect_hardware(self):
        time.sleep(1.0)
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = 60
            self.kit = ServoKit(channels=16, i2c=i2c, address=0x60)
            
            # 조향 서보 초기화
            self.kit.servo[self.servo_channel].angle = self.center_angle
            self.set_throttle_hardware(0.0)
            
            # [초기화 동작] 시작 시 천천히 초기 위치(UP, OPEN)로 이동
            self.get_logger().info("🏁 Initializing Servo Positions...")
            self.move_servo_smooth(self.lift_channel, self.current_lift, self.LIFT_UP)
            self.move_servo_smooth(self.gripper_channel, self.current_grip, self.GRIP_OPEN)
            
            self.hardware_connected = True
            self.get_logger().info("🔌 Hardware Connected & Ready")
        except Exception as e:
            self.get_logger().error(f"❌ Hardware Error: {e}")

    def move_servo_smooth(self, channel, start_angle, end_angle, step_delay=0.03):
        """
        step_delay: 0.01(빠름) ~ 0.05(느림). 
        test_gripper.py 처럼 천천히 움직이도록 0.03으로 설정했습니다.
        """
        if not self.hardware_connected: return
        if abs(start_angle - end_angle) < 1.0: return

        step = 1.0 if end_angle > start_angle else -1.0
        current = float(start_angle)
        
        while True:
            if (step > 0 and current >= end_angle) or (step < 0 and current <= end_angle):
                break
            current += step
            try:
                self.kit.servo[channel].angle = current
            except: pass
            time.sleep(step_delay)
            
        try:
            self.kit.servo[channel].angle = end_angle
        except: pass

    def gripper_callback(self, msg):
        if not self.hardware_connected: return
        cmd = msg.data.upper()
        
        # 동작 중복 방지를 위해 쓰레드로 실행
        threading.Thread(target=self.execute_gripper, args=(cmd,)).start()

    def execute_gripper(self, cmd):
        """
        그리퍼 동작 시퀀스 실행
        cmd: "PICK" (잡기), "PLACE" (놓기), "INIT" (초기화)
        """
        self.get_logger().info(f"🦾 Gripper Sequence: {cmd}")
        
        # [시퀀스 1] 물체 잡기 (내리기 -> 잡기 -> 올리기)
        if cmd == "PICK":
            # 1. 리프트 내리기
            self.get_logger().info("  -> Lift DOWN")
            self.move_servo_smooth(self.lift_channel, self.current_lift, self.LIFT_DOWN)
            self.current_lift = self.LIFT_DOWN
            time.sleep(1.0) # 기구적 안정화 대기
            
            # 2. 그리퍼 잡기
            self.get_logger().info("  -> Grip CLOSE")
            self.move_servo_smooth(self.gripper_channel, self.current_grip, self.GRIP_CLOSE)
            self.current_grip = self.GRIP_CLOSE
            time.sleep(1.0) # 꽉 잡을 시간 대기
            
            # 3. 리프트 올리기
            self.get_logger().info("  -> Lift UP")
            self.move_servo_smooth(self.lift_channel, self.current_lift, self.LIFT_UP)
            self.current_lift = self.LIFT_UP
            time.sleep(1.0)
            
            self.get_logger().info("✅ PICK Sequence Complete")

        # [시퀀스 2] 물체 놓기 (내리기 -> 풀기 -> 올리기)
        elif cmd == "PLACE":
            # 1. 리프트 내리기
            self.get_logger().info("  -> Lift DOWN")
            self.move_servo_smooth(self.lift_channel, self.current_lift, self.LIFT_DOWN)
            self.current_lift = self.LIFT_DOWN
            time.sleep(1.0)
            
            # 2. 그리퍼 풀기
            self.get_logger().info("  -> Grip OPEN")
            self.move_servo_smooth(self.gripper_channel, self.current_grip, self.GRIP_OPEN)
            self.current_grip = self.GRIP_OPEN
            time.sleep(1.0) # 물체가 떨어질 시간 대기
            
            # 3. 리프트 올리기
            self.get_logger().info("  -> Lift UP")
            self.move_servo_smooth(self.lift_channel, self.current_lift, self.LIFT_UP)
            self.current_lift = self.LIFT_UP
            time.sleep(1.0)
            
            self.get_logger().info("✅ PLACE Sequence Complete")

        # [시퀀스 3] 초기화 (안전하게 들고 벌리기)
        elif cmd == "INIT":
            self.get_logger().info("  -> Initializing...")
            # 먼저 들어올려서 바닥 충돌 방지
            self.move_servo_smooth(self.lift_channel, self.current_lift, self.LIFT_UP)
            self.current_lift = self.LIFT_UP
            time.sleep(0.5)
            
            # 벌리기
            self.move_servo_smooth(self.gripper_channel, self.current_grip, self.GRIP_OPEN)
            self.current_grip = self.GRIP_OPEN
            self.get_logger().info("✅ Initialized")
            
        else:
            self.get_logger().warn(f"⚠️ Unknown command: {cmd}")

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
        
        # 조향 제어
        if steering_angle_deg > 0: 
            target = 100.0 - (steering_angle_deg * 1.4)
        else: 
            target = 100.0 - (steering_angle_deg * 1.2)
        
        try:
            self.kit.servo[self.servo_channel].angle = max(0, min(180, target))
            self.set_throttle_hardware(-v) 
        except: pass

    def set_throttle_hardware(self, throttle):
        if not self.pca: return
        throttle = max(-0.8, min(0.8, throttle))
        pulse = int(0xFFFF * abs(throttle))
        in1, in2, in3 = self.motor_channel + 5, self.motor_channel + 4, self.motor_channel + 3

        if abs(throttle) < 0.05: 
            self.pca.channels[in1].duty_cycle = 0; self.pca.channels[in2].duty_cycle = 0; self.pca.channels[in3].duty_cycle = 0
        elif throttle > 0: 
            self.pca.channels[in1].duty_cycle = pulse; self.pca.channels[in2].duty_cycle = 0; self.pca.channels[in3].duty_cycle = 0xFFFF
        else: 
            self.pca.channels[in1].duty_cycle = pulse; self.pca.channels[in2].duty_cycle = 0xFFFF; self.pca.channels[in3].duty_cycle = 0

    def stop_robot(self):
        if self.hardware_connected:
            self.set_throttle_hardware(0)
            self.pca.deinit()

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriver()
    rclpy.spin(node)
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
