#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class VisionFinal(Node):
    def __init__(self):
        super().__init__('vision_final')
        
        self.FRONT_CAM = 0
        self.REAR_CAM = 2
        
        # 시스템 모드 구독
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        # 통합된 이미지 토픽 발행
        self.image_pub = self.create_publisher(Image, '/camera/integrated_stream', 10)
        
        self.bridge = CvBridge()
        self.cap = None
        self.current_mode = "IDLE"
        
        # 약 15 FPS
        self.timer = self.create_timer(0.06, self.vision_loop) 
        self.get_logger().info("✅ Vision Final Started: Waiting for command...")

    def mode_callback(self, msg):
        if self.current_mode != msg.data:
            self.get_logger().info(f"🔄 Mode Switch: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            self.switch_camera(self.current_mode)

    def switch_camera(self, mode):
        # 1. 기존 카메라 자원 해제 (USB 대역폭 확보)
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(0.5) 

        # 2. 모드에 맞는 카메라 선택
        target_cam = -1
        if "DOCKING" in mode: 
            target_cam = self.REAR_CAM
        elif "MARSHALLER" in mode: 
            target_cam = self.FRONT_CAM
        elif "DRIVING" in mode:
            target_cam = self.FRONT_CAM # 주행 시 전방 카메라 사용 시
        
        # 3. 새 카메라 연결
        if target_cam != -1:
            self.cap = cv2.VideoCapture(target_cam)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if self.cap.isOpened():
                self.get_logger().info(f"📸 Camera {target_cam} ON")
            else:
                self.get_logger().error(f"❌ Failed to open Camera {target_cam}")

    def vision_loop(self):
        if self.cap is None or not self.cap.isOpened(): return

        ret, frame = self.cap.read()
        if not ret: return

        # 원본 이미지 송출
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionFinal()
    rclpy.spin(node)
    if node.cap: node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()