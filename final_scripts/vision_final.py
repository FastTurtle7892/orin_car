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
        
        # 1. 토픽 설정
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        
        self.bridge = CvBridge()

        # 2. 카메라 인덱스 (환경에 맞게 확인 필요)
        self.FRONT_CAM_IDX = 0  # 전방
        self.REAR_CAM_IDX = 2   # 후방
        
        # [핵심] 단 하나의 캡처 객체만 사용 (동시 접속 원천 차단)
        self.cap = None
        self.current_mode = "IDLE"

        # 3. 타이머 실행 (30Hz)
        self.timer = self.create_timer(0.033, self.vision_loop)
        
        self.get_logger().info(f"🎥 Vision Final Started (Single Camera Mode)")

    def mode_callback(self, msg):
        if self.current_mode != msg.data:
            self.get_logger().info(f"⚡ 모드 변경 감지: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            # 모드가 바뀔 때만 카메라 스위칭 실행
            self.switch_camera()

    def switch_camera(self):
        """ 기존 카메라를 끄고, 모드에 맞는 새 카메라를 켭니다. """
        
        # 1. 안전하게 기존 카메라 해제 (Release)
        if self.cap is not None:
            self.get_logger().info("🔄 기존 카메라 해제 중...")
            self.cap.release()
            self.cap = None
            # USB 버스 안정화를 위한 짧은 대기 (필수)
            time.sleep(0.5)

        target_cam = -1
        
        # 2. 모드별 타겟 카메라 설정
        if self.current_mode == "DOCKING":
            target_cam = self.REAR_CAM_IDX   # 도킹 -> 후방
            self.get_logger().info(f"🎯 Target: 후방 카메라 (Index {target_cam})")
            
        elif self.current_mode == "MARSHALLER":
            target_cam = self.FRONT_CAM_IDX  # 마샬러 -> 전방
            self.get_logger().info(f"🎯 Target: 전방 카메라 (Index {target_cam})")
            
        elif self.current_mode == "DRIVING":
            target_cam = self.FRONT_CAM_IDX  # [요청사항] 주행 -> 전방 카메라 ON
            self.get_logger().info(f"🎯 Target: 전방 카메라 (Index {target_cam})")
            
        elif self.current_mode == "IDLE":
            self.get_logger().info("💤 IDLE 모드: 카메라를 끕니다.")
            return # 연결 시도 안 함

        # 3. 카메라 열기 (Open)
        if target_cam != -1:
            try:
                self.cap = self.open_camera(target_cam)
                if self.cap and self.cap.isOpened():
                    self.get_logger().info(f"✅ 카메라 연결 성공! (Mode: {self.current_mode})")
                else:
                    self.get_logger().error(f"❌ 카메라 연결 실패 (Index: {target_cam})")
            except Exception as e:
                self.get_logger().error(f"❌ 카메라 에러 발생: {e}")

    def open_camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        # 대역폭 절약을 위한 MJPEG 포맷 설정
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap

    def vision_loop(self):
        status_msg = String()

        # 카메라가 없거나 닫혀있으면 IDLE 상태 전송
        if self.cap is None or not self.cap.isOpened():
            status_msg.data = f"Mode: {self.current_mode} (Cam Off)"
            self.vision_pub.publish(status_msg)
            return

        # 프레임 읽기
        ret, frame = self.cap.read()
        
        if ret:
            # 화면에 현재 모드 텍스트 표시
            color = (0, 255, 0) # 기본 초록색
            if self.current_mode == "DOCKING": color = (0, 0, 255) # 빨간색
            elif self.current_mode == "DRIVING": color = (0, 255, 255) # 노란색

            cv2.putText(frame, f"MODE: {self.current_mode}", (30, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            # ROS 이미지 메시지로 변환하여 발행
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
                status_msg.data = f"Mode: {self.current_mode} (Active)"
            except Exception: pass
        else:
            # 읽기 실패 시 자원 해제 (다음 명령 대기)
            self.get_logger().error("❌ 프레임 읽기 끊김! 카메라 닫음.")
            self.cap.release()
            self.cap = None
            status_msg.data = f"Error: {self.current_mode} (Lost Frame)"

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
