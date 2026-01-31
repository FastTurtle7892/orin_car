#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image  # [추가] 이미지 메시지
from cv_bridge import CvBridge     # [추가] OpenCV -> ROS 변환
import cv2
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

try:
    from docking_ai import DockingAI
    from gesture_ai import MarshallerAI
except ImportError as e:
    print(f"Import Error: {e}")

class VisionManager(Node):
    def __init__(self):
        super().__init__('vision_manager')
        
        # 1. 구독 및 퍼블리셔
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        
        # [NEW] 영상을 송출할 이미지 퍼블리셔 생성
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # 2. 모델 경로
        self.model_path = os.path.expanduser('~/ros_ws/src/orin_car/config/yolov8n-pose.pt')
        if not os.path.exists(self.model_path):
            self.model_path = None 
        else:
            self.get_logger().info(f"✅ 모델 파일 확인됨: {self.model_path}")

        # 3. AI 객체
        self.docking_ai = DockingAI()
        self.marshaller_ai = MarshallerAI(self.model_path)
        
        # 4. 카메라 설정
        self.FRONT_CAM = 0
        self.REAR_CAM = 2
        self.cap = None
        self.current_mode = "IDLE"
        
        self.timer = self.create_timer(0.1, self.vision_loop)
        self.log_timer = 0
        
        self.get_logger().info(f"👀 Vision Manager Started. Listening to '/system_mode'...")

    def mode_callback(self, msg):
        self.get_logger().info(f"📨 메시지 수신됨! 내용: {msg.data}")
        if self.current_mode != msg.data:
            self.get_logger().info(f"🔄 모드 변경 실행: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            self.switch_camera(self.current_mode)

    def switch_camera(self, mode):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(0.5)

        target_cam = -1
        if mode == "DOCKING": target_cam = self.REAR_CAM
        elif mode == "MARSHALLER": target_cam = self.FRONT_CAM
        
        if target_cam != -1:
            self.cap = cv2.VideoCapture(target_cam)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if self.cap.isOpened():
                self.get_logger().info(f"✅ Camera {target_cam} ON ({mode})")
            else:
                self.get_logger().error(f"❌ Camera {target_cam} Open Failed!")

    def vision_loop(self):
        status_msg = String()
        
        # IDLE 상태
        self.log_timer += 1
        if self.current_mode == "IDLE":
            if self.log_timer % 30 == 0: 
                self.get_logger().info("💤 IDLE 상태 대기 중... (명령을 주세요)")
            status_msg.data = "IDLE"
            self.vision_pub.publish(status_msg)
            if self.cap is not None: self.switch_camera("IDLE")
            return

        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret: return

        # AI 처리 및 결과 이미지 생성
        final_frame = frame.copy()

        if self.current_mode == "DOCKING":
            result, processed_frame = self.docking_ai.process(frame)
            status_msg.data = f"DOCKING: {result['found']}"
            final_frame = processed_frame # 처리된(박스 그려진) 이미지 사용
            
        elif self.current_mode == "MARSHALLER":
            action, processed_frame = self.marshaller_ai.detect_gesture(frame)
            status_msg.data = f"MARSHALLER: {action}"
            final_frame = processed_frame

        # [핵심] cv2.imshow 대신 ROS 토픽으로 발행
        try:
            ros_image = self.bridge.cv2_to_imgmsg(final_frame, "bgr8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"이미지 발행 실패: {e}")

        self.vision_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if node.cap: node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
