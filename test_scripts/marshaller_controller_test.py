#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import sys
import os

# [중요] gesture_ai.py 필요
try:
    from gesture_ai_test import MarshallerAI
except ImportError:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from gesture_ai_test import MarshallerAI

class MarshallerController(Node):
    def __init__(self):
        super().__init__('marshaller_controller')
        
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # YOLO 모델 경로 (본인 환경에 맞게 수정 필요!)
        model_path = os.path.expanduser('~/ros_ws/src/orin_car/config/yolov8n-pose.pt')
        self.ai = MarshallerAI(model_path)
        
        self.cap = None
        self.is_active = False
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Marshaller Controller Ready")

    def mode_callback(self, msg):
        if msg.data == "MARSHALLER":
            if not self.is_active:
                self.start_marshaller()
        else:
            if self.is_active:
                self.stop_marshaller()

    def start_marshaller(self):
        self.get_logger().info("👮 Mode: MARSHALLER -> Opening Camera 0...")
        self.is_active = True
        
        try:
            self.cap = cv2.VideoCapture(0) # 전방 카메라
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            if not self.cap.isOpened():
                self.get_logger().error("❌ Camera 0 Open Failed!")
                self.stop_marshaller()
        except Exception as e:
            self.get_logger().error(f"❌ Camera Error: {e}")

    def stop_marshaller(self):
        self.get_logger().info("🛑 Marshaller Stopped.")
        self.is_active = False
        if self.cap:
            self.cap.release()
            self.cap = None
        self.stop_robot()

    def control_loop(self):
        if not self.is_active or not self.cap or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret: return

        # AI 인식
        action, _ = self.ai.detect_gesture(frame)
        self.get_logger().info(f"Gesture: {action}")

        # 제어 로직
        twist = Twist()
        if action == "go":
            twist.linear.x = 0.2
        elif action == "back":
            twist.linear.x = -0.2
        elif action == "left":
            twist.linear.x = 0.15
            twist.angular.z = 0.5
        elif action == "right":
            twist.linear.x = 0.15
            twist.angular.z = -0.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = MarshallerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()