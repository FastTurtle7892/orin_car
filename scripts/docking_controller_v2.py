#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
from docking_ai import DockingAI

class DockingControllerV2(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        # [모드 제어]
        self.is_active = False
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)

        # 설정
        self.TARGET_DIST_CM = 16.4
        self.BASE_SPEED = -0.15
        self.KP_STEER = 0.02
        self.STEP_WAIT_TIME = 1.5

        self.create_subscription(Image, '/rear_camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        self.bridge = CvBridge()
        self.docking_ai = DockingAI() 
        
        self.state_mode = 0  
        self.docking_step = 0           
        self.docking_start_time = 0.0

        self.get_logger().info("✅ Docking Controller V2 Ready (Waiting for 'DOCKING' mode)")

    def mode_callback(self, msg):
        if msg.data == "DOCKING":
            if not self.is_active:
                self.get_logger().info("🔔 Docking Mode ACTIVATED")
                self.is_active = True
                self.initialize_pose()
        else:
            if self.is_active:
                self.get_logger().info("🔕 Docking Mode DEACTIVATED")
                self.stop_robot()
                self.is_active = False

    def initialize_pose(self):
        self.publish_gripper("INIT") 
        self.state_mode = 1 
        time.sleep(2.0)

    def image_callback(self, msg):
        if not self.is_active: return # 모드 비활성 시 무시

        if self.state_mode == 2:
            self.run_gripper_sequence()
            return
        if self.state_mode == 99 or self.state_mode == 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        result = self.docking_ai.process(cv_image)
        if not isinstance(result, (list, tuple)) or len(result) < 2:
            self.stop_robot()
            return

        data, frame = result
        found = data.get("found", False)

        if not found:
            self.stop_robot()
            return

        dist_cm = data.get("dist_cm", 999.9)
        x_cm = data.get("x_cm", 0.0)
        if dist_cm is None: dist_cm = 999.9

        if dist_cm <= self.TARGET_DIST_CM:
            self.get_logger().info("🚀 Starting Grip Sequence!")
            self.stop_robot()
            self.state_mode = 2
            self.docking_step = 1  
            self.docking_start_time = time.time() 
        else:
            twist = Twist()
            twist.linear.x = self.BASE_SPEED
            twist.angular.z = self.KP_STEER * x_cm 
            self.cmd_vel_pub.publish(twist)

    def run_gripper_sequence(self):
        elapsed = time.time() - self.docking_start_time
        if self.docking_step == 1:
            if elapsed > 0.5:
                self.publish_gripper("DOWN")
                self.next_step(2)
        elif self.docking_step == 2:
            if elapsed > self.STEP_WAIT_TIME:
                self.publish_gripper("GRIP")
                self.next_step(3)
        elif self.docking_step == 3:
            if elapsed > self.STEP_WAIT_TIME:
                self.publish_gripper("UP")
                self.next_step(4)
        elif self.docking_step == 4:
            if elapsed > self.STEP_WAIT_TIME:
                self.get_logger().info("✅ Docking Mission Complete")
                self.state_mode = 99

    def next_step(self, next_step_num):
        self.docking_step = next_step_num
        self.docking_start_time = time.time()

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_gripper(self, cmd):
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerV2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
