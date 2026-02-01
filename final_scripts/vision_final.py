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
        self.image_pub = self.create_publisher(Image, '/camera/integrated_stream', 10)
        
        self.bridge = CvBridge()

        # 2. 카메라 인덱스
        self.FRONT_CAM_IDX = 0  
        self.REAR_CAM_IDX = 2   
        
        self.cap = None
        self.current_mode = "IDLE"
        
        # [추가] 재연결 쿨타임 관리용
        self.last_retry_time = 0.0

        # 3. 타이머 실행 (30Hz)
        self.timer = self.create_timer(0.033, self.vision_loop)
        
        self.get_logger().info(f"🎥 Vision Final Started (Auto-Retry Enabled)")

    def mode_callback(self, msg):
        # 모드가 변경되면 즉시 카메라 스위칭 시도
        if self.current_mode != msg.data:
            self.get_logger().info(f"⚡ 모드 변경 감지: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            self.switch_camera()

    def switch_camera(self):
        """ 기존 카메라를 끄고, 모드에 맞는 새 카메라를 켭니다. """
        
        # 1. 기존 카메라 해제
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(0.5) # 안정화 대기

        target_cam = -1
        
        # 2. 모드별 타겟 설정
        if self.current_mode == "DOCKING":
            target_cam = self.REAR_CAM_IDX   
        elif self.current_mode == "MARSHALLER":
            target_cam = self.FRONT_CAM_IDX  
        elif self.current_mode == "DRIVING":
            target_cam = self.FRONT_CAM_IDX  
        elif self.current_mode == "IDLE":
            return 

        # 3. 카메라 열기 시도
        if target_cam != -1:
            try:
                self.cap = self.open_camera(target_cam)
                if self.cap and self.cap.isOpened():
                    self.get_logger().info(f"✅ 카메라 연결 성공! (Index: {target_cam})")
                else:
                    self.get_logger().error(f"❌ 카메라 연결 실패 (Index: {target_cam})")
            except Exception as e:
                self.get_logger().error(f"❌ 카메라 에러: {e}")

    def open_camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap

    def vision_loop(self):
        status_msg = String()

        # [핵심 수정] 자동 복구 로직 (Self-Healing)
        # 현재 모드가 IDLE이 아닌데(즉, 카메라가 켜져야 하는데) 카메라가 꺼져 있다면?
        if self.cap is None or not self.cap.isOpened():
            if self.current_mode in ["DRIVING", "DOCKING", "MARSHALLER"]:
                now = time.time()
                # 2초마다 재연결 시도
                if now - self.last_retry_time > 2.0:
                    self.get_logger().warn(f"⚠️ {self.current_mode} 모드인데 카메라 꺼짐. 재연결 시도...")
                    self.switch_camera()
                    self.last_retry_time = now
            
            status_msg.data = f"Mode: {self.current_mode} (Cam Off)"
            self.vision_pub.publish(status_msg)
            return

        # 프레임 읽기 및 발행
        ret, frame = self.cap.read()
        if ret:
            color = (0, 255, 0)
            if self.current_mode == "DOCKING": color = (0, 0, 255)
            elif self.current_mode == "DRIVING": color = (0, 255, 255)

            cv2.putText(frame, f"MODE: {self.current_mode}", (30, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
                status_msg.data = f"Mode: {self.current_mode} (Active)"
            except Exception: pass
        else:
            self.get_logger().error("❌ 프레임 끊김! 카메라 닫음.")
            self.cap.release()
            self.cap = None

        self.vision_pub.publish(status_msg)

    def destroy_node(self):
        if self.cap: self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionFinal()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
