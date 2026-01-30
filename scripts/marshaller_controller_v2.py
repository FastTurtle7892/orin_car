#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import os
import sys
from ament_index_python.packages import get_package_share_directory

# Gesture AI import path fix
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from gesture_ai import MarshallerAI
except ImportError:
    pass

class MarshallerControllerV2(Node):
    def __init__(self):
        super().__init__('marshaller_controller')

        self.declare_parameter('speed_fast', 0.25)
        self.declare_parameter('speed_slow', 0.12)
        self.declare_parameter('turn_angle', 0.5)

        # [모드 제어]
        self.is_active = False
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        try:
            pkg_share = get_package_share_directory('orin_car')
            model_path = os.path.join(pkg_share, 'config', 'yolov8n-pose.pt')
            self.marshal_ai = MarshallerAI(model_path=model_path)
            self.get_logger().info("✅ Marshaller AI Loaded")
        except:
            self.marshal_ai = None

        self.get_logger().info("✅ Marshaller Controller V2 Ready (Waiting for 'MARSHALLING' mode)")

    def mode_callback(self, msg):
        if msg.data == "MARSHALLING":
            if not self.is_active:
                self.get_logger().info("🔔 Marshalling Mode ACTIVATED")
                self.is_active = True
        else:
            if self.is_active:
                self.get_logger().info("🔕 Marshalling Mode DEACTIVATED")
                self.process_command("STOP")
                self.is_active = False

    def image_callback(self, msg):
        if not self.is_active or self.marshal_ai is None: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except: return

        action, _ = self.marshal_ai.detect_gesture(frame)
        self.process_command(action)

    def process_command(self, action):
        twist = Twist()
        msg_grip = String()
        
        speed_fast = self.get_parameter('speed_fast').value
        speed_slow = self.get_parameter('speed_slow').value
        turn_ang = self.get_parameter('turn_angle').value

        if action == "FORWARD":
            twist.linear.x = float(speed_fast)
        elif action == "APPROACHING":
            twist.linear.x = float(speed_slow)
        elif action == "TURN_LEFT":
            twist.linear.x = float(speed_slow)
            twist.angular.z = float(turn_ang)
        elif action == "TURN_RIGHT":
            twist.linear.x = float(speed_slow)
            twist.angular.z = -float(turn_ang)
        elif action == "STOP" or action == "SET_BRAKES":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif action == "GRIPPER_HOLD":
            msg_grip.data = "GRIP"
            self.gripper_pub.publish(msg_grip)
        elif action == "GRIPPER_RELEASE":
            msg_grip.data = "OPEN"
            self.gripper_pub.publish(msg_grip)
        elif action == "RESET":
            msg_grip.data = "INIT"
            self.gripper_pub.publish(msg_grip)

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MarshallerControllerV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
