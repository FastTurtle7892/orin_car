#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class VisionManagerRaw(Node):
    def __init__(self):
        super().__init__('vision_manager_raw')
        
        # 1. 구독 및 퍼블리셔
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        
        # [핵심] 이미지를 송출할 퍼블리셔
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # 2. 카메라 설정 (전면 0, 후면 2)
        self.FRONT_CAM_IDX = 0
        self.REAR_CAM_IDX = 2
        
        self.cap_front = None
        self.cap_rear = None
        self.current_mode = "IDLE"

        # 3. 카메라 초기화 (MJPEG 모드 - 딜레이 0초)
        self.init_cameras()
        
        # 4. 루프 실행 (30Hz - 최대한 빠르게 쏘기)
        self.timer = self.create_timer(0.033, self.vision_loop)
        
        self.get_logger().info(f"🎥 Vision Manager RAW Started (No AI, Pure Stream)")

    def init_cameras(self):
        self.get_logger().info("📷 카메라 2대 초기화 중 (Raw Mode)...")
        self.cap_front = self.open_camera(self.FRONT_CAM_IDX)
        self.cap_rear = self.open_camera(self.REAR_CAM_IDX)
        
        if self.cap_front and self.cap_rear:
            self.get_logger().info("✅ 카메라 2대 준비 완료!")

    def open_camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            return cap
        return None

    def mode_callback(self, msg):
        if self.current_mode != msg.data:
            self.get_logger().info(f"⚡ 모드 전환: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data

    def vision_loop(self):
        status_msg = String()
        target_frame = None
        
        # 1. 버퍼 비우기 (항상 읽음)
        ret_f, frame_front = False, None
        ret_r, frame_rear = False, None
        
        if self.cap_front: ret_f, frame_front = self.cap_front.read()
        if self.cap_rear:  ret_r, frame_rear = self.cap_rear.read()

        # 2. 모드에 따라 원본 이미지 선택 (AI 처리 없음!)
        if self.current_mode == "DOCKING":
            if ret_r:
                target_frame = frame_rear
                status_msg.data = "DOCKING (RAW)"
            else:
                status_msg.data = "DOCKING (No Signal)"

        elif self.current_mode == "MARSHALLER":
            if ret_f:
                target_frame = frame_front
                status_msg.data = "MARSHALLER (RAW)"
            else:
                status_msg.data = "MARSHALLER (No Signal)"
        
        elif self.current_mode == "IDLE":
            status_msg.data = "IDLE"
            # IDLE일 때는 아무것도 안 보냄 (대역폭 절약)
        
        # 3. 이미지 즉시 발행
        if target_frame is not None:
            try:
                # [핵심] 그리기 과정 없이 바로 ROS 메시지로 변환해서 쏨
                ros_image = self.bridge.cv2_to_imgmsg(target_frame, "bgr8")
                self.image_pub.publish(ros_image)
            except Exception: pass

        self.vision_pub.publish(status_msg)

    def destroy_node(self):
        if self.cap_front: self.cap_front.release()
        if self.cap_rear: self.cap_rear.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionManagerRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
