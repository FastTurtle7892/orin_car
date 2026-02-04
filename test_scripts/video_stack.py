#!/usr/bin/env python3
"""
video_stack.py: 카메라 리소스 관리(Front/Rear), WebRTC 송출, 마샬러 AI, 도킹 영상 중계
"""

import asyncio
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ✅ 패키지 모듈 import
from orin_car.videos.camera_manager import CameraManager
from orin_car.videos.webrtc_sharedcam import webrtc_main

try:
    from orin_car.gesture_ai_test import MarshallerAI
except Exception:
    try:
        from gesture_ai_test import MarshallerAI
    except Exception:
        MarshallerAI = None
        print("⚠️ MarshallerAI module not found. AI features disabled.")
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy

class DualCameraSwitcher:
    """WebRTC용 카메라 선택 스위처"""
    def __init__(self, cam_front, cam_rear):
        self.cam_front = cam_front
        self.cam_rear = cam_rear
        self.active_cam = cam_front
        self.lock = threading.Lock()

    def set_front(self):
        with self.lock:
            self.active_cam = self.cam_front

    def set_rear(self):
        with self.lock:
            self.active_cam = self.cam_rear

    def get_latest_frame(self, copy: bool = True):
        with self.lock:
            return self.active_cam.get_latest_frame(copy=copy)


class MarshallerControllerSharedCam(Node):
    def __init__(self, cam_front, cam_switcher):
        super().__init__("marshaller_controller_sharedcam")

        self.cam_ai_source = cam_front  # 마샬러는 항상 전방
        self.cam_switcher = cam_switcher # WebRTC용 스위처

        # ✅ [수정] 이미지를 보낼 때 "SensorDataQoS" (Best Effort) 적용
        # 이렇게 하면 밀린 데이터는 버리고 최신 데이터 위주로 보냅니다.
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.rear_img_pub = self.create_publisher(Image, "/camera/rear/raw", qos_profile_sensor)
        self.bridge = CvBridge()

        # 기존 설정
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.mode_sub = self.create_subscription(String, "/system_mode", self.mode_callback, 10)
        self.mode_pub = self.create_publisher(String, "/system_mode", qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/marshaller/debug_image", 10)

        self.current_mode = "WAITING"
        self.drive_state = "STOP"
        self.marshaller_ai = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Video Stack & Camera Hub Started")

    def mode_callback(self, msg):
        prev = self.current_mode
        self.current_mode = msg.data

        if prev != self.current_mode:
            self.get_logger().info(f"🔄 Mode: {prev} -> {self.current_mode}")
            
            # 모드에 따라 WebRTC 송출 화면 변경
            if self.current_mode == "DOCKING":
                self.cam_switcher.set_rear()
            else:
                self.cam_switcher.set_front()
            
            self.drive_state = "STOP"

    def control_loop(self):
        # 1️⃣ [영상 중계] 도킹 모드라면 후방 카메라 이미지를 ROS 토픽으로 송신
        if self.current_mode == "DOCKING":
            frame_rear = self.cam_switcher.cam_rear.get_latest_frame(copy=True)
            if frame_rear is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame_rear, encoding="bgr8")
                self.rear_img_pub.publish(img_msg)

        # 2️⃣ 마샬러 모드가 아니면 AI 로직 종료
        if self.current_mode != "MARSHAL":
            return

        # 3️⃣ 마샬러 로직 (전방 카메라 사용)
        frame_front = self.cam_ai_source.get_latest_frame(copy=True)
        if frame_front is None: return

        if self.marshaller_ai is None and MarshallerAI:
            self.marshaller_ai = MarshallerAI()

        if self.marshaller_ai:
            action, out_frame = self.marshaller_ai.detect_gesture(frame_front)
            
            # 도킹 제스처 인식 시 모드 전환
            if action == "DOCKING":
                self.cmd_vel_pub.publish(Twist()) # 정지
                self.mode_pub.publish(String(data="DOCKING")) 
                return

            # 주행 로직
            valid = ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"]
            if action in valid and self.drive_state != action:
                self.drive_state = action
            
            twist = Twist()
            should_pub = True
            if self.drive_state == "FORWARD": twist.linear.x = 0.35
            elif self.drive_state == "BACKWARD": twist.linear.x = -0.35
            elif self.drive_state == "LEFT": twist.angular.z = 0.5
            elif self.drive_state == "RIGHT": twist.angular.z = -0.5
            elif self.drive_state == "STOP": 
                twist.linear.x = 0.0; twist.angular.z = 0.0
            elif self.drive_state == "MANUAL_MODE": should_pub = False
            
            if should_pub:
                self.cmd_vel_pub.publish(twist)

            # 디버그 이미지
            if self.debug_pub.get_subscription_count() > 0:
                cv2.putText(out_frame, f"CMD: {self.drive_state}", (10,30), 1, 2, (0,255,0), 2)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(out_frame, "bgr8"))


def _run_webrtc_thread(switcher):
    try: asyncio.run(webrtc_main(switcher))
    except Exception as e: print(f"WebRTC Error: {e}")


def main(args=None):
    # ⚠️ [필수 확인] 카메라 포트 번호
    IDX_FRONT = 0
    IDX_REAR = 2 

    print("📷 Opening Cameras...")
    cam_front = CameraManager(IDX_FRONT, 640, 480, 20)
    cam_rear = CameraManager(IDX_REAR, 640, 480, 20)
    
    if not cam_front.start():
        print(f"❌ Front Cam({IDX_FRONT}) Failed!")
        return
    if not cam_rear.start(): 
        print(f"⚠️ Rear Cam({IDX_REAR}) Failed! (WebRTC will use Front only)")

    # WebRTC & ROS 실행
    switcher = DualCameraSwitcher(cam_front, cam_rear)
    t_webrtc = threading.Thread(target=_run_webrtc_thread, args=(switcher,), daemon=True)
    t_webrtc.start()

    rclpy.init(args=args)
    node = MarshallerControllerSharedCam(cam_front, switcher)
    
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cam_front.stop()
        cam_rear.stop()
        print("✅ video_stack shutdown complete")

if __name__ == "__main__":
    main()