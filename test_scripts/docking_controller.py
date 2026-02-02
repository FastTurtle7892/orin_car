#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import time
import sys
import os

# [중요] 같은 폴더에 docking_ai.py가 있어야 함
try:
    from docking_ai import DockingAI
except ImportError:
    # 경로 문제 시 현재 폴더 추가
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from docking_ai import DockingAI

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # 1. 통신 설정
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        # 2. 변수 초기화
        self.docking_ai = DockingAI()
        self.cap = None
        self.is_active = False
        self.step = 0
        
        # 3. 제어 루프 (0.1초 간격)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Docking Controller Ready (Waiting for Mode)")

    def mode_callback(self, msg):
        if msg.data == "DOCKING":
            if not self.is_active:
                self.start_docking()
        else:
            if self.is_active:
                self.stop_docking()

    def start_docking(self):
        self.get_logger().info("🚢 Mode: DOCKING -> Opening Camera 2...")
        self.is_active = True
        self.step = 0
        
        # 카메라 2번 열기 (후방)
        try:
            self.cap = cv2.VideoCapture(2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            if not self.cap.isOpened():
                self.get_logger().error("❌ Camera 2 Open Failed!")
                self.stop_docking()
        except Exception as e:
            self.get_logger().error(f"❌ Camera Error: {e}")

    def stop_docking(self):
        self.get_logger().info("🛑 Docking Stopped. (Closing Camera)")
        self.is_active = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.stop_robot()

    def control_loop(self):
        # 활성화 상태이고 카메라가 켜져있을 때만 동작
        if not self.is_active or not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret: return

        # AI 분석
        data, _ = self.docking_ai.process(frame)
        
        if not data['found']:
            self.stop_robot()
            return

        dist = data['dist_cm']
        x_cm = data['x_cm']
        
        # 제어 로직 (목표 거리 16.4cm)
        if dist <= 16.4:
            self.stop_robot()
            if self.step == 0:
                self.run_gripper_sequence()
        else:
            twist = Twist()
            twist.linear.x = -0.15  # 후진
            twist.angular.z = x_cm * 0.02 # 조향 P제어
            self.cmd_vel_pub.publish(twist)

    def run_gripper_sequence(self):
        if self.step != 0: return
        self.step = 1
        self.get_logger().info("✊ Arrived! Starting Gripper Sequence...")
        
        # 1. 내리기
        self.gripper_pub.publish(String(data="DOWN"))
        time.sleep(1.5)
        # 2. 잡기
        self.gripper_pub.publish(String(data="GRIP"))
        time.sleep(1.5)
        # 3. 올리기
        self.gripper_pub.publish(String(data="UP"))
        time.sleep(1.5)
        
        self.get_logger().info("✅ Grabbing Complete!")
        # (옵션) 완료 후 IDLE로 돌아가거나 유지

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()