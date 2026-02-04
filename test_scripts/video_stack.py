#!/usr/bin/env python3

"""video_stack.py

✅ 목표
- 카메라를 딱 1번만 open (CameraManager)
- ROS2 노드(마샬러)와 WebRTC 송출이 동일 카메라 프레임을 공유
- /system_mode == "MARSHAL"일 때만 마샬러 추론 수행

⚠️ 중요
- 이 파일은 ROS2 패키지 orin_car의 "설치된" 실행파일로 동작하는 것을 전제로 한다.
"""

import asyncio
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# ✅ videos 폴더를 orin_car 패키지 내부로 흡수한 뒤의 정식 import
from orin_car.videos.camera_manager import CameraManager
from orin_car.videos.webrtc_sharedcam import webrtc_main

# ✅ MarshallerAI도 패키지 import가 정석 (소스에서 단독 실행하는 경우만 fallback)
try:
    from orin_car.gesture_ai_test import MarshallerAI
except Exception:  # pragma: no cover
    # 개발 중 "python3 video_stack.py"로 돌릴 때를 위한 최소 fallback
    from gesture_ai_test import MarshallerAI

class MarshallerControllerSharedCam(Node):
    """
    기존 marshaller_controller_test.py의 구조를 최대한 유지하되,
    VideoCapture를 열지 않고 CameraManager에서 프레임만 받는다.
    """

    def __init__(self, cam_manager: CameraManager):
        super().__init__("marshaller_controller_sharedcam")

        self.get_logger().info("====================================")
        self.get_logger().info("🚀 Marshaller + Shared Camera 시작 🚀")
        self.get_logger().info("====================================")

        self.mode_sub = self.create_subscription(String, "/system_mode", self.mode_callback, 10)
        self.mode_pub = self.create_publisher(String, "/system_mode", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/marshaller/debug_image", 10)

        self.current_mode = "WAITING"
        self.drive_state = "STOP"
        self.wait_tick = 0

        self.bridge = CvBridge()
        self.cam = cam_manager

        self.marshaller_ai = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ MarshallerControllerSharedCam Initialized")

    def mode_callback(self, msg):
        prev = self.current_mode
        self.current_mode = msg.data
        if prev != self.current_mode:
            self.get_logger().info(f"🔄 모드 변경: {prev} -> {self.current_mode}")
            self.drive_state = "STOP"

    def control_loop(self):
        if self.current_mode != "MARSHAL":
            self.wait_tick += 1
            return

        self.wait_tick = 0

        frame = self.cam.get_latest_frame(copy=True)
        if frame is None:
            self.get_logger().warn("⚠️ No frame yet from CameraManager", throttle_duration_sec=2.0)
            return

        # AI lazy init (기존 흐름 유지)
        if self.marshaller_ai is None:
            try:
                self.marshaller_ai = MarshallerAI()
                self.get_logger().info("🧠 MarshallerAI Loaded.")
            except Exception as e:
                self.get_logger().error(f"❌ AI Init Error: {e}")
                return

        action, out_frame = self.marshaller_ai.detect_gesture(frame)

        valid_commands = ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "MANUAL_MODE"]

        if action == "DOCKING":
            self.get_logger().info("🚀 [EVENT] 도킹 명령 수신! DOCKING 모드로 전환")

            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)

            mode_msg = String()
            mode_msg.data = "DOCKING"
            self.mode_pub.publish(mode_msg)
            return

        if action in valid_commands and self.drive_state != action:
            self.get_logger().info(f"🔄 상태 변경: [{self.drive_state}] ➔ [{action}]")
            self.drive_state = action

        twist = Twist()
        should_publish = True

        if self.drive_state == "FORWARD":
            twist.linear.x = 0.35
        elif self.drive_state == "BACKWARD":
            twist.linear.x = -0.35
        elif self.drive_state == "LEFT":
            twist.angular.z = 0.5
        elif self.drive_state == "RIGHT":
            twist.angular.z = -0.5
        elif self.drive_state == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif self.drive_state == "MANUAL_MODE":
            should_publish = False

        if should_publish:
            self.cmd_vel_pub.publish(twist)

        # Debug image publish (기존 유지)
        if self.debug_pub.get_subscription_count() > 0:
            cv2.putText(
                out_frame,
                f"STATE: {self.drive_state}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 255),
                2,
            )
            msg = self.bridge.cv2_to_imgmsg(out_frame, encoding="bgr8")
            self.debug_pub.publish(msg)


def _run_webrtc_thread(cam: CameraManager):
    """
    WebRTC는 asyncio라서 별도 스레드에서 asyncio.run()으로 돌린다.
    카메라는 절대 열지 않고 cam.get_latest_frame만 사용.
    """
    try:
        asyncio.run(webrtc_main(cam))
    except Exception as e:
        print(f"[WebRTC Thread] exception: {e}")


def main(args=None):
    # 1) CameraManager 시작 (카메라 open은 여기 딱 1번)
    cam = CameraManager(device_index=0, width=640, height=480, fps=20)
    if not cam.start():
        print("❌ CameraManager failed to open camera.")
        return

    # 2) WebRTC 백그라운드 스레드 시작
    t_webrtc = threading.Thread(target=_run_webrtc_thread, args=(cam,), daemon=True)
    t_webrtc.start()

    # 3) ROS2 노드(마샬러) 실행
    rclpy.init(args=args)
    node = MarshallerControllerSharedCam(cam)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cam.stop()
        print("✅ video_stack shutdown complete")


if __name__ == "__main__":
    main()
