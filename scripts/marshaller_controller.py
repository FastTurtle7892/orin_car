#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import sys
import os

# ROS 2 패키지 경로를 찾기 위한 라이브러리
from ament_index_python.packages import get_package_share_directory

# 현재 폴더(scripts)를 import 경로에 추가하여 gesture_ai를 찾을 수 있게 함
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from gesture_ai import MarshallerAI
except ImportError:
    print("❌ gesture_ai.py를 찾을 수 없습니다. 같은 폴더에 있는지 확인하세요.")

class MarshallerController(Node):
    def __init__(self):
        super().__init__('marshaller_controller')

        # 1. 튜닝 파라미터 (속도 및 회전각)
        self.declare_parameter('speed_fast', 0.25)
        self.declare_parameter('speed_slow', 0.12)
        self.declare_parameter('turn_angle', 0.5)

        # 2. 퍼블리셔 설정
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        # 3. 서브스크라이버 설정 (카메라 이미지 구독)
        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # 4. AI 모델 로드 (Config 폴더에서 모델 찾기)
        try:
            pkg_share = get_package_share_directory('orin_car')
            # install/share/orin_car/config/yolov8n-pose.pt 경로
            model_path = os.path.join(pkg_share, 'config', 'yolov8n-pose.pt')
            
            self.get_logger().info(f"📂 모델 경로 확인: {model_path}")
            
            # 모델 경로를 AI 모듈에 전달
            self.marshal_ai = MarshallerAI(model_path=model_path)
            self.get_logger().info("✅ Marshaller AI 로드 완료! 준비되었습니다.")
            
        except Exception as e:
            self.get_logger().error(f"❌ 모델 로드 실패: {e}")
            self.marshal_ai = None

    def image_callback(self, msg):
        if self.marshal_ai is None: return

        try:
            # ROS Image -> OpenCV Image 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return

        # --- AI 인식 실행 ---
        # action: 동작 이름 (문자열)
        # debug_frame: 화면 송출은 안 하지만, AI 내부 처리를 위해 변수는 받아둡니다.
        action, _ = self.marshal_ai.detect_gesture(frame)
        
        # --- 로봇 제어 및 로그 출력 ---
        self.process_command(action)

    def process_command(self, action):
        twist = Twist()
        msg_grip = String()
        
        # 파라미터 값 읽기
        speed_fast = self.get_parameter('speed_fast').value
        speed_slow = self.get_parameter('speed_slow').value
        turn_ang = self.get_parameter('turn_angle').value

        # --- 동작 매핑 ---
        if action == "FORWARD":
            twist.linear.x = float(speed_fast)
            self.get_logger().info(f"🚗 전진 (FAST)")
            
        elif action == "APPROACHING":
            twist.linear.x = float(speed_slow)
            self.get_logger().info(f"🚗 접근 (SLOW)")
            
        elif action == "TURN_LEFT":
            twist.linear.x = float(speed_slow)
            twist.angular.z = float(turn_ang)
            self.get_logger().info(f"↩️ 좌회전")
            
        elif action == "TURN_RIGHT":
            twist.linear.x = float(speed_slow)
            twist.angular.z = -float(turn_ang)
            self.get_logger().info(f"↪️ 우회전")
            
        elif action == "STOP" or action == "SET_BRAKES":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # 정지 상태 로그는 반복되므로 생략하거나 필요 시 추가하세요
            
        elif action == "GRIPPER_HOLD":
            msg_grip.data = "GRIP"
            self.gripper_pub.publish(msg_grip)
            self.get_logger().info("✊ 그리퍼 잡기")
            
        elif action == "GRIPPER_RELEASE":
            msg_grip.data = "OPEN"
            self.gripper_pub.publish(msg_grip)
            self.get_logger().info("🖐 그리퍼 놓기")
            
        elif action == "RESET":
            msg_grip.data = "INIT"
            self.gripper_pub.publish(msg_grip)
            self.get_logger().info("🔄 리셋 (초기화)")
            
        else:
            # IDLE, READY 등 주행 상태가 아니면 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MarshallerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
