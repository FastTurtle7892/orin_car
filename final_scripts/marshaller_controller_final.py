#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
from gesture_ai import MarshallerAI

class MarshallerControllerFinal(Node):
    def __init__(self):
        super().__init__('marshaller_controller_final')
        
        self.declare_parameter('speed_fast', 0.25)
        self.declare_parameter('speed_slow', 0.12)
        self.declare_parameter('turn_angle', 0.5)
        
        # 구독: 통합 비전 스트림
        self.create_subscription(Image, '/camera/integrated_stream', self.image_callback, 10)
        # 구독: 시스템 모드 (제어권 관리용)
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        model_path = os.path.expanduser('~/ros_ws/src/orin_car/config/yolov8n-pose.pt')
        self.ai = MarshallerAI(model_path)
        
        self.active_mode = False # ★ 기본 비활성화
        self.get_logger().info("✅ Marshaller Controller Final Started")

    def mode_callback(self, msg):
        if "MARSHALLER" in msg.data:
            if not self.active_mode:
                self.get_logger().info("🔔 Marshaller Mode ACTIVATED")
            self.active_mode = True
        else:
            if self.active_mode:
                self.stop_robot()
            self.active_mode = False

    def image_callback(self, msg):
        if not self.active_mode: return # 내 차례 아니면 리턴

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        action, _ = self.ai.detect_gesture(cv_image)
        self.control_robot(action)

    def control_robot(self, action):
        twist = Twist()
        speed_fast = self.get_parameter('speed_fast').value
        speed_slow = self.get_parameter('speed_slow').value
        turn_angle = self.get_parameter('turn_angle').value

        if action == "COME":
            twist.linear.x = speed_slow
        elif action == "LEFT":
            twist.angular.z = turn_angle
        elif action == "RIGHT":
            twist.angular.z = -turn_angle
        elif action == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = MarshallerControllerFinal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()