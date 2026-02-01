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
        
        # 1. 구독: 시스템 모드 수신
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        
        # 2. 발행: 상태 및 이미지
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # 3. 카메라 설정
        self.FRONT_CAM_IDX = 0
        self.REAR_CAM_IDX = 2
        
        # [조건 1] 초기 상태에서는 self.cap이 None이므로 카메라는 OFF 상태입니다.
        self.cap = None
        self.current_mode = "IDLE"

        # 4. 루프 실행 (30Hz)
        self.timer = self.create_timer(0.033, self.vision_loop)
        
        self.get_logger().info(f"🎥 Vision Final Ready (Smart Switching Mode)")

    def mode_callback(self, msg):
        if self.current_mode != msg.data:
            self.get_logger().info(f"⚡ 모드 변경 감지: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            self.switch_camera(self.current_mode)

    def switch_camera(self, mode):
        # [조건 2] 기존 카메라가 켜져 있다면 무조건 끄고 시작 -> 절대 2대가 동시에 켜지지 않음
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(0.5) # USB 자원 반환 대기

        target_cam = -1

        # 모드별 카메라 선택 로직
        if mode == "DOCKING":
            # [조건 3] 도킹 -> 후면 카메라
            target_cam = self.REAR_CAM_IDX
            self.get_logger().info(f"🔄 후방 카메라(Index {target_cam}) 연결 시도...")
        
        elif mode == "MARSHALLER" or mode == "DRIVING": 
            # [조건 4, 5] 마샬러 OR 주행 -> 전면 카메라
            target_cam = self.FRONT_CAM_IDX
            self.get_logger().info(f"🔄 전방 카메라(Index {target_cam}) 연결 시도...")
        
        elif mode == "IDLE":
            # IDLE -> 카메라 끔
            self.get_logger().info("💤 IDLE 모드: 카메라를 끕니다.")
            return

        # 선택된 카메라 연결
        if target_cam != -1:
            self.cap = self.open_camera(target_cam)
            if self.cap and self.cap.isOpened():
                self.get_logger().info(f"✅ 카메라 연결 성공 (Mode: {mode})")
            else:
                self.get_logger().error(f"❌ 카메라 연결 실패 (Mode: {mode})")

    def open_camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        # 대역폭 절약을 위한 MJPEG 설정
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap

    def vision_loop(self):
        status_msg = String()
        
        # 카메라가 꺼져 있으면 이미지 발행 안 함
        if self.cap is None or not self.cap.isOpened():
            status_msg.data = f"Mode: {self.current_mode} (Camera Off)"
            self.vision_pub.publish(status_msg)
            return

        ret, frame = self.cap.read()
        
        if ret:
            # 상태 텍스트 오버레이
            if self.current_mode == "DOCKING":
                status_msg.data = "Mode: DOCKING (Rear Cam)"
                cv2.putText(frame, "DOCKING MODE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            elif self.current_mode == "MARSHALLER":
                status_msg.data = "Mode: MARSHALLER (Front Cam)"
                cv2.putText(frame, "MARSHALLER MODE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            elif self.current_mode == "DRIVING":
                status_msg.data = "Mode: DRIVING (Front Cam)"
                cv2.putText(frame, "DRIVING MODE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

            # 이미지 ROS 토픽 발행
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
            except Exception:
                pass
        else:
            status_msg.data = f"Error: {self.current_mode} (No Frame)"

        self.vision_pub.publish(status_msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()