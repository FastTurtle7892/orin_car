#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
import math
from docking_ai import DockingAI

class DockingControllerV2(Node):
    def __init__(self):
        super().__init__('docking_controller_v2')

        # ================= [튜닝 파라미터] =================
        self.TARGET_DIST_CM = 16.4
        
        # [속도 약간 증가] 바퀴가 꺾여있을 때 마찰을 이기기 위해 속도를 올림
        self.BASE_SPEED = -0.18      
        
        # [조향 방향] 
        # 1.0 : 반대 방향으로 움직이게 수정됨
        self.STEER_DIR = 1.0 
        
        # [게인 값 안정화] 너무 높으면 바퀴가 떨면서 안 움직일 수 있어 낮춤
        self.KP_LAT = 0.030   # 위치 보정
        self.KP_YAW = 0.015   # 각도 보정
        
        self.MAX_YAW_TOLERANCE = 3.0
        # ===================================================

        self.create_subscription(Image, '/rear_camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        self.bridge = CvBridge()
        self.docking_ai = DockingAI() 
        
        self.state_mode = 0  
        self.docking_step = 0           
        self.docking_start_time = 0.0
        self.last_log_time = 0

        self.create_timer(1.0, self.initialize_pose_once)
        self.get_logger().info("✅ Docking Controller V2 (Stabilized) Started")

    def initialize_pose_once(self):
        if self.state_mode == 0:
            self.get_logger().info("🏁 [INIT] Pose Setup")
            self.publish_gripper("INIT") 
            self.state_mode = 1 
            time.sleep(2.0)

    def image_callback(self, msg):
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

        # 마커를 못 찾으면 정지합니다. (로그 확인 필요)
        if not found:
            if time.time() - self.last_log_time > 2.0:
                self.get_logger().info("👀 Searching Marker... (No Target Found)")
                self.last_log_time = time.time()
            self.stop_robot()
            return

        dist_cm = data.get("dist_cm", 999.9)
        x_cm = data.get("x_cm", 0.0)
        yaw_deg = data.get("yaw", 0.0)

        # 1. 도착 확인
        if dist_cm <= self.TARGET_DIST_CM:
            if abs(yaw_deg) > self.MAX_YAW_TOLERANCE:
                self.get_logger().warn(f"⚠️ Dist OK but Angle Bad! ({yaw_deg:.1f}°)")
            
            self.get_logger().info(f"🛑 Arrived! Dist:{dist_cm:.1f}cm")
            self.stop_robot()
            self.state_mode = 2
            self.docking_step = 1  
            self.docking_start_time = time.time() 
        
        else:
            # 2. 주행 제어
            raw_steering = (self.KP_LAT * x_cm) - (self.KP_YAW * yaw_deg)
            target_angular_z = raw_steering * self.STEER_DIR

            # [디버깅] 값이 정상적으로 나오는지 확인
            if time.time() - self.last_log_time > 0.5:
                print(f"[Run] X:{x_cm:.1f} | Yaw:{yaw_deg:.1f} | Speed:{self.BASE_SPEED} | Steer:{target_angular_z:.3f}")
                self.last_log_time = time.time()

            twist = Twist()
            twist.linear.x = self.BASE_SPEED
            twist.angular.z = float(target_angular_z)
            self.cmd_vel_pub.publish(twist)

    def run_gripper_sequence(self):
        elapsed = time.time() - self.docking_start_time
        STEP_WAIT = 1.5

        if self.docking_step == 1:
            if elapsed > 0.5:
                self.get_logger().info("🔽 Lift DOWN")
                self.publish_gripper("DOWN")
                self.next_step(2)

        elif self.docking_step == 2:
            if elapsed > STEP_WAIT:
                self.get_logger().info("✊ Gripper CLOSE")
                self.publish_gripper("GRIP")
                self.next_step(3)

        elif self.docking_step == 3:
            if elapsed > STEP_WAIT:
                self.get_logger().info("🔼 Lift UP")
                self.publish_gripper("UP")
                self.next_step(4)

        elif self.docking_step == 4:
            if elapsed > STEP_WAIT:
                self.get_logger().info("✅ Complete!")
                self.state_mode = 99

    def next_step(self, next_step_num):
        self.docking_step = next_step_num
        self.docking_start_time = time.time()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def publish_gripper(self, cmd):
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
