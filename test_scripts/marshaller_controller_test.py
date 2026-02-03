#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

# [중요] 업데이트된 gesture_ai_test 파일에서 클래스 임포트
from gesture_ai_test import MarshallerAI 

class MarshallerController(Node):
    def __init__(self):
        super().__init__('marshaller_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🚀 마샬러 차량 제어 (State Machine Ver) 시작 🚀") 
        self.get_logger().info("====================================")
        
        # 1. Pub/Sub 설정
        self.mode_sub = self.create_subscription(
            String, '/system_mode', self.mode_callback, 10
        )
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/marshaller/debug_image', 10)

        # 2. 상태 및 객체 초기화
        self.current_mode = "WAITING"   # 시스템 모드
        self.drive_state = "STOP"       # [NEW] 주행 상태 기억 변수 (초기값 정지)
        
        self.bridge = CvBridge()
        self.cap = None
        self.marshaller_ai = None
        self.wait_tick = 0 
        
        # 전방 카메라 인덱스
        self.CAMERA_INDEX = 0 
        
        # 주기적 실행 (0.1초 단위)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("✅ Marshaller Controller Initialized")

    def mode_callback(self, msg):
        previous_mode = self.current_mode
        self.current_mode = msg.data
        if previous_mode != self.current_mode:
            self.get_logger().info(f"🔄 모드 변경 감지: {previous_mode} -> {self.current_mode}")
            # 모드가 바뀌면 주행 상태도 안전하게 정지로 초기화
            self.drive_state = "STOP"

    def open_camera(self):
        """카메라가 닫혀있으면 열기"""
        if self.cap is None:
            self.get_logger().info(f"📷 Opening Front Camera ({self.CAMERA_INDEX})...")
            self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 20)
            
            if not self.cap.isOpened():
                self.get_logger().error("❌ Failed to open Front Camera!")
                self.cap = None
            else:
                self.get_logger().info("✅ Camera Open Success")
                if self.marshaller_ai is None:
                    try:
                        self.marshaller_ai = MarshallerAI()
                        self.get_logger().info("🧠 MarshallerAI Loaded.")
                    except Exception as e:
                        self.get_logger().error(f"❌ AI Init Error: {e}")

    def close_camera(self):
        """카메라가 열려있으면 닫기"""
        if self.cap is not None:
            self.get_logger().info("zzz Closing Front Camera...")
            self.cap.release()
            self.cap = None

    def control_loop(self):
        # 1. MARSHAL 모드가 아니면 카메라 끄고 대기
        if self.current_mode != "MARSHAL":
            self.close_camera()
            self.wait_tick += 1
            return

        self.wait_tick = 0
        
        # 2. MARSHAL 모드면 카메라 켜기
        self.open_camera()
        if self.cap is None: return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Camera Read Error", throttle_duration_sec=2.0)
            return

        # 3. AI 추론
        action, out_frame = self.marshaller_ai.detect_gesture(frame)

        # =========================================================
        # [핵심 변경] 상태 기반 로직 (State Machine)
        # =========================================================
        
        # 상태를 변경시킬 수 있는 유효한 명령어 목록
        valid_commands = ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "MANUAL_MODE"]
        
        # A. 도킹 명령은 최우선 처리 (상태와 무관하게 즉시 종료 로직 수행)
        if action == "DOCKING":
            self.get_logger().info("🚀 [EVENT] 도킹 명령 수신! 도킹 모드로 전환합니다.")
            
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg) # 즉시 정지
            
            mode_msg = String()
            mode_msg.data = "DOCKING"
            self.mode_pub.publish(mode_msg)
            
            self.close_camera()
            return

        # B. 유효한 제스처가 감지되면 상태 업데이트 (Latching)
        #    (IDLE이나 SKIP인 경우, 이 블록을 건너뛰어 기존 self.drive_state 유지)
        if action in valid_commands:
            if self.drive_state != action:
                self.get_logger().info(f"🔄 상태 변경: [{self.drive_state}] ➔ [{action}]")
                self.drive_state = action
        
        # C. 현재 상태(self.drive_state)에 맞춰 차량 제어
        twist = Twist()
        should_publish = True
        
        if self.drive_state == "FORWARD":
            twist.linear.x = 0.35
            self.get_logger().info(f"🚗 [GO] 전진 중... (Action: {action})", throttle_duration_sec=2.0)
            
        elif self.drive_state == "BACKWARD":
            twist.linear.x = -0.35
            self.get_logger().info(f"🚗 [BACK] 후진 중... (Action: {action})", throttle_duration_sec=2.0)
            
        elif self.drive_state == "LEFT":
            twist.angular.z = 0.5
            self.get_logger().info(f"🔄 [LEFT] 좌회전 중... (Action: {action})", throttle_duration_sec=2.0)
            
        elif self.drive_state == "RIGHT":
            twist.angular.z = -0.5
            self.get_logger().info(f"🔄 [RIGHT] 우회전 중... (Action: {action})", throttle_duration_sec=2.0)
            
        elif self.drive_state == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # 정지 상태 로그는 가끔 출력
            self.get_logger().info(f"🛑 [STOP] 대기 중...", throttle_duration_sec=5.0)
            
        elif self.drive_state == "MANUAL_MODE":
            # 수동 모드 상태에서는 제어 명령을 보내지 않음 (혹은 0을 보냄)
            should_publish = False 
            self.get_logger().info("🙌 수동 모드 유지 중...", throttle_duration_sec=5.0)
            
        # 제어 명령 발행
        if should_publish:
            self.cmd_vel_pub.publish(twist)

        # 5. 디버그 이미지 발행 (화면에 현재 상태 표시)
        if self.debug_pub.get_subscription_count() > 0:
            # 이미지에 현재 상태 텍스트 추가
            cv2.putText(out_frame, f"STATE: {self.drive_state}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            msg = self.bridge.cv2_to_imgmsg(out_frame, encoding="bgr8")
            self.debug_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MarshallerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopping Marshaller Controller...")
    finally:
        node.close_camera()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()