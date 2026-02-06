#!/usr/bin/env python3
"""
video_stack.py: 모드 기반 마샬러 동작 제어 (안전장치 포함)
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
from ultralytics import YOLO
from orin_car.videos.camera_manager import CameraManager
from orin_car.videos.webrtc_sharedcam import webrtc_main

class DualCameraSwitcher:
    def __init__(self, cam_front, cam_rear):
        self.cam_front = cam_front
        self.cam_rear = cam_rear
        self.active_cam = cam_front
        self.lock = threading.Lock()
    def set_front(self):
        with self.lock: self.active_cam = self.cam_front
    def set_rear(self):
        with self.lock: self.active_cam = self.cam_rear
    def get_latest_frame(self, copy: bool = True):
        with self.lock: return self.active_cam.get_latest_frame(copy=copy)

class MarshallerControllerSharedCam(Node):
    def __init__(self, cam_front, cam_switcher):
        super().__init__("marshaller_controller_sharedcam")

        self.cam_ai_source = cam_front
        self.cam_switcher = cam_switcher

        qos_profile_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.rear_img_pub = self.create_publisher(Image, "/camera/rear/raw", qos_profile_sensor)
        
        qos_profile = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_sub = self.create_subscription(String, "/system_mode", self.mode_callback, 10)
        self.mode_pub = self.create_publisher(String, "/system_mode", qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/marshaller/debug_image", 10)

        self.bridge = CvBridge()
        self.current_mode = "WAITING"
        self.drive_state = "STOP"

        # YOLO 모델 로드
        self.abs_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.engine'
        self.pt_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.pt'
        try:
            self.model = YOLO(self.abs_path, task='pose')
            self.get_logger().info("✅ TensorRT Engine 로드됨")
        except:
            self.model = YOLO(self.pt_path)
            self.get_logger().info("⚠️ .pt 모델 로드됨")

        self.frame_count = 0
        self.SKIP_FRAMES = 2
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Video Stack Started")

    def mode_callback(self, msg):
        prev = self.current_mode
        self.current_mode = msg.data
        if prev != self.current_mode:
            self.get_logger().info(f"🔄 Mode: {prev} -> {self.current_mode}")
            if self.current_mode == "DOCKING": self.cam_switcher.set_rear()
            else: self.cam_switcher.set_front()
            self.drive_state = "STOP"

    def control_loop(self):
        # 1. 후방 카메라 송출 (도킹 시)
        if self.current_mode == "DOCKING":
            frame_rear = self.cam_switcher.cam_rear.get_latest_frame(copy=True)
            if frame_rear is not None:
                img_msg = self.bridge.cv2_to_imgmsg(frame_rear, encoding="bgr8")
                self.rear_img_pub.publish(img_msg)

        # 2. 전방 카메라 AI (항상 프레임은 가져옴)
        frame_front = self.cam_ai_source.get_latest_frame(copy=True)
        if frame_front is None: return

        # ✅ [핵심 안전장치] 마샬러 모드가 아니면 AI 추론 및 제어 금지!
        # 이렇게 해야 일반 주행 중에 사람이 손 흔들어도 차가 안 멈춥니다.
        if self.current_mode != "MARSHAL":
            return

        self.frame_count += 1
        if self.frame_count % (self.SKIP_FRAMES + 1) != 0: return 

        results = self.model(frame_front, verbose=False, conf=0.5, device=0, half=True)
        action = "STOP"

        if results[0].keypoints is not None and len(results[0].keypoints.data) > 0:
            kpts = results[0].keypoints.data[0].cpu().numpy()
            l_sh_y, r_sh_y = kpts[5][1], kpts[6][1]
            l_sh_x, r_sh_x = kpts[5][0], kpts[6][0]
            l_wr_x, l_wr_y = kpts[9][0], kpts[9][1]
            r_wr_x, r_wr_y = kpts[10][0], kpts[10][1]
            
            shoulder_width = abs(l_sh_x - r_sh_x)
            l_up, r_up = l_wr_y < l_sh_y, r_wr_y < r_sh_y
            l_spread = l_wr_x < (l_sh_x - shoulder_width * 0.3)
            r_spread = r_wr_x > (r_sh_x + shoulder_width * 0.3)

            if l_up and r_up: action = "FORWARD"
            elif l_up and not r_up: action = "LEFT_FWD"
            elif not l_up and r_up: action = "RIGHT_FWD"
            elif (not l_up and not r_up) and (l_spread and r_spread): action = "BACKWARD"
            else: action = "STOP"

        twist = Twist()
        LINEAR_VEL = 0.35
        ANGULAR_VEL = 0.5
        
        if action == "FORWARD":
            self.drive_state = "FORWARD"
            twist.linear.x = LINEAR_VEL
        elif action == "LEFT_FWD":
            self.drive_state = "LEFT >>"
            twist.linear.x = LINEAR_VEL
            twist.angular.z = ANGULAR_VEL
        elif action == "RIGHT_FWD":
            self.drive_state = "<< RIGHT"
            twist.linear.x = LINEAR_VEL
            twist.angular.z = -ANGULAR_VEL
        elif action == "BACKWARD":
            self.drive_state = "BACKWARD"
            twist.linear.x = -LINEAR_VEL
        else:
            self.drive_state = "STOP"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            cv2.putText(frame_front, "STOP", (250, 240), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,0,255), 4)

        self.cmd_vel_pub.publish(twist)

        # 디버그 화면
        color = (0, 255, 0) if action != "STOP" else (0, 0, 255)
        cv2.putText(frame_front, f"CMD: {self.drive_state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame_front, "bgr8"))

def _run_webrtc_thread(switcher):
    try: asyncio.run(webrtc_main(switcher))
    except Exception as e: print(f"WebRTC Error: {e}")

def main(args=None):
    IDX_FRONT, IDX_REAR = 0, 2
    cam_front = CameraManager(IDX_FRONT, 640, 480, 20)
    cam_rear = CameraManager(IDX_REAR, 640, 480, 20)
    
    if not cam_front.start(): return
    if not cam_rear.start(): print("⚠️ Rear Cam Failed")

    switcher = DualCameraSwitcher(cam_front, cam_rear)
    threading.Thread(target=_run_webrtc_thread, args=(switcher,), daemon=True).start()

    rclpy.init(args=args)
    node = MarshallerControllerSharedCam(cam_front, switcher)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cam_front.stop()
        cam_rear.stop()

if __name__ == "__main__":
    main()