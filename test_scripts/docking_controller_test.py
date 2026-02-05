#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image  # 이미지 수신용
from cv_bridge import CvBridge     # 변환용
import cv2
import sys
import time
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, QoSDurabilityPolicy
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from docking_ai_test import DockingAI

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🔒 도킹 컨트롤러 (수신기 모드) 시작 🔒") 
        self.get_logger().info("====================================")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.completion_pub = self.create_publisher(String, '/task_completion', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(String, '/gripper_cmd', 10)
        
        # 시스템 모드 구독
        self.mode_sub = self.create_subscription(
            String, 
            '/system_mode', 
            self.mode_callback, 
            qos_profile
        )
        
        # ✅ [핵심 변경] 카메라 직접 여는 대신 video_stack이 주는 이미지 구독
        self.img_sub = self.create_subscription(
            Image,
            '/camera/rear/raw',
            self.image_callback,
            qos_profile_sensor
        )
        self.bridge = CvBridge()
        self.latest_frame = None

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.ai = DockingAI()
        
        self.is_docking_process_started = False
        self.create_timer(1.0, self.send_init_gripper)
        self.is_init_sent = False

        self.TARGET_DIST = 16.5
        self.STOP_TOLERANCE = 1.0 
        self.FIXED_SPEED = -0.25
        
        self.system_mode = "IDLE"
        self.is_docked = False

    def image_callback(self, msg):
        """ ROS Image 메시지를 OpenCV 포맷으로 변환해서 저장 """
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {e}")

    def mode_callback(self, msg):
        if self.system_mode != msg.data:
            self.get_logger().info(f"📨 모드 변경: {self.system_mode} -> {msg.data}")
            self.system_mode = msg.data
            
            if self.system_mode == "DOCKING":
                self.is_docked = False
                self.is_docking_process_started = False
                self.latest_frame = None # 이전 잔상 제거
            else:
                self.stop_robot()

    def send_init_gripper(self):
        if not self.is_init_sent:
            self.publish_gripper("INIT")
            self.is_init_sent = True

    def publish_gripper(self, command):
        msg = String()
        msg.data = command
        self.gripper_publisher.publish(msg)

    def execute_grip_sequence(self):
        self.get_logger().info("🚀 잡기 시퀀스 시작")
        self.publish_gripper("DOWN"); time.sleep(2.0) 
        self.publish_gripper("GRIP"); time.sleep(2.0)
        self.publish_gripper("UP"); time.sleep(2.0)
        self.get_logger().info("✅ 잡기 완료")

    def timer_callback(self):
        # 도킹 중이 아니면 스킵
        if self.system_mode != "DOCKING": return
        if self.is_docked: return
        if self.is_docking_process_started: return

        # ✅ [변경] 카메라 read() 대신 구독한 최신 프레임 사용
        frame = self.latest_frame
        if frame is None:
            # video_stack.py가 아직 이미지를 안 보내주거나 로딩중인 상태
            return

        try:
            data, processed_frame = self.ai.process(frame)
        except Exception:
            return

        cmd_msg = Twist()

        if data["found"]:
            dist = data["dist_cm"]
            x_err = data["x_cm"]
            
            self.get_logger().info(f"🟢 감지! 거리:{dist:.1f}cm", throttle_duration_sec=0.5)

            error_dist = dist - self.TARGET_DIST
            
            if abs(error_dist) <= self.STOP_TOLERANCE:
                self.is_docking_process_started = True 
                self.perform_docking(dist)
                return
            else:
                speed = self.FIXED_SPEED 
                if error_dist < -self.STOP_TOLERANCE: speed = -self.FIXED_SPEED 
                k_steer = 0.05
                steer = max(min(x_err * k_steer, 0.5), -0.5)
                
                cmd_msg.linear.x = speed
                cmd_msg.angular.z = steer
        else:
            self.get_logger().info(f"🔎 마커 찾는 중...", throttle_duration_sec=2.0)
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_publisher.publish(cmd_msg)

    def perform_docking(self, dist):
        self.stop_robot()
        self.get_logger().info(f"🎯 도착 완료! ({dist:.1f}cm)")
        
        self.execute_grip_sequence()
        
        self.is_docked = True
        
        # 완료 신호 전송 -> 이후 video_stack이 IDLE/DRIVING 모드로 바꾸면 카메라 복귀됨
        done_msg = String()
        done_msg.data = "DOCKING_COMPLETE"
        self.completion_pub.publish(done_msg)
        self.get_logger().info("📢 도킹 완료! (DOCKING_COMPLETE)")
        
        self.is_docking_process_started = False
        # 모드를 IDLE로 변경하여 루프 종료
        self.system_mode = "IDLE"

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()