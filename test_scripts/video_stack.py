#!/usr/bin/env python3
"""
video_stack.py: 카메라 리소스 관리, WebRTC, 그리고 "초심플" 마샬러(YOLO 직접 통합)
"""

import asyncio
import threading
import time
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ✅ YOLO 라이브러리 추가
from ultralytics import YOLO

# ✅ 패키지 모듈 import
from orin_car.videos.camera_manager import CameraManager
from orin_car.videos.webrtc_sharedcam import webrtc_main

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

        # ROS2 설정
        qos_profile_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.rear_img_pub = self.create_publisher(Image, "/camera/rear/raw", qos_profile_sensor)
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.mode_sub = self.create_subscription(String, "/system_mode", self.mode_callback, 10)
        self.mode_pub = self.create_publisher(String, "/system_mode", qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/marshaller/debug_image", 10)

        self.bridge = CvBridge()
        self.current_mode = "WAITING"
        self.drive_state = "STOP"

        # =========================================================
        # [NEW] YOLOv8 Pose 모델 로드 (simple_motion.py 로직 통합)
        # =========================================================
        self.abs_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.engine'
        self.pt_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.pt'
        
        try:
            self.model = YOLO(self.abs_path, task='pose')
            self.get_logger().info("✅ TensorRT Engine 로드됨")
        except:
            self.model = YOLO(self.pt_path)
            self.get_logger().info("⚠️ .pt 모델 로드됨")

        self.frame_count = 0
        self.SKIP_FRAMES = 2  # 3프레임당 1번 추론

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Video Stack Started (Simple Marshaller Mode)")

    def mode_callback(self, msg):
        prev = self.current_mode
        self.current_mode = msg.data

        if prev != self.current_mode:
            self.get_logger().info(f"🔄 Mode: {prev} -> {self.current_mode}")
            
            # 도킹 모드일 때만 후방 카메라 송출
            if self.current_mode == "DOCKING":
                self.cam_switcher.set_rear()
            else:
                self.cam_switcher.set_front()
            
            self.drive_state = "STOP"

    def control_loop(self):
        # 1️⃣ [영상 중계] 도킹 모드 시 후방 카메라 토픽 발행
        if self.current_mode == "DOCKING":
            frame_rear = self.cam_switcher.cam_rear.get_latest_frame(copy=True)
            if frame_rear is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame_rear, encoding="bgr8")
                self.rear_img_pub.publish(img_msg)

        # 2️⃣ [마샬러 AI] 항상 동작 (전방 카메라)
        frame_front = self.cam_ai_source.get_latest_frame(copy=True)
        if frame_front is None: return

        # 프레임 스킵 (부하 감소)
        self.frame_count += 1
        if self.frame_count % (self.SKIP_FRAMES + 1) != 0:
            return 

        # YOLO 추론
        results = self.model(frame_front, verbose=False, conf=0.5, device=0, half=True)
        
        action = "STOP" # 기본값

        # 사람이 감지되었을 때 로직 수행
        if results[0].keypoints is not None and len(results[0].keypoints.data) > 0:
            kpts = results[0].keypoints.data[0].cpu().numpy()
            
            # 좌표 추출
            l_sh_y, r_sh_y = kpts[5][1], kpts[6][1]   # 어깨 Y
            l_wr_y, r_wr_y = kpts[9][1], kpts[10][1]  # 손목 Y
            l_wr_x, r_wr_x = kpts[9][0], kpts[10][0]  # 손목 X
            
            # [조건 1] 가슴 모으기 (강력 정지)
            body_center = (kpts[5][0] + kpts[6][0]) / 2
            shoulder_width = abs(kpts[5][0] - kpts[6][0])
            
            is_gathered = abs(l_wr_x - body_center) < shoulder_width * 0.8 and \
                          abs(r_wr_x - body_center) < shoulder_width * 0.8
            is_down = l_wr_y > l_sh_y and r_wr_y > r_sh_y

            if is_gathered and is_down:
                action = "STOP_CHEST"
            
            # [조건 2] 전진 (양손 만세) - 손목이 어깨보다 위
            elif l_wr_y < l_sh_y and r_wr_y < r_sh_y:
                action = "FORWARD"
            
            # 그 외는 STOP 유지

        # 3️⃣ 차량 제어 명령 발행
        twist = Twist()
        
        if action == "FORWARD":
            self.drive_state = "FORWARD"
            twist.linear.x = 0.35
            
        elif action == "STOP_CHEST":
            self.drive_state = "STOP (CHEST)"
            twist.linear.x = 0.0
            # 화면에 경고 표시
            cv2.putText(frame_front, "CHEST STOP!", (200, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
            
        else: # STOP
            self.drive_state = "STOP"
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

        # 4️⃣ 디버그 이미지 발행 (WebRTC 화면에 오버레이 됨)
        color = (0, 255, 0) if "FORWARD" in self.drive_state else (0, 0, 255)
        cv2.putText(frame_front, f"CMD: {self.drive_state}", (20, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame_front, "bgr8"))


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