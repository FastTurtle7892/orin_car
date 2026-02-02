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

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
# 현재 폴더(scripts) 경로 추가
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from gesture_ai import MarshallerAI
except ImportError:
    print("❌ gesture_ai.py를 찾을 수 없습니다.")

class MarshallerController(Node):
    def __init__(self):
        super().__init__('marshaller_controller')

        # 1. 파라미터
        self.declare_parameter('speed_fast', 0.25)
        self.declare_parameter('speed_slow', 0.12)
        self.declare_parameter('turn_angle', 0.5)

        # [통합 모드 관리]
        self.current_mode = "IDLE"
        self.create_subscription(String, '/robot_mode', self.mode_callback, 10)

        # 2. 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        # 3. 서브스크라이버
        self.create_subscription(Image, '/front_camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        # 4. AI 모델 로드
        try:
            pkg_share = get_package_share_directory('orin_car')
            model_path = os.path.join(pkg_share, 'config', 'yolov8n-pose.pt')
            
            self.get_logger().info(f"📂 모델 경로: {model_path}")
            self.marshal_ai = MarshallerAI(model_path=model_path)
            self.get_logger().info("✅ Marshaller AI 로드 완료 (Waiting for 'MARSHALLER' mode)")
            
        except Exception as e:
            self.get_logger().error(f"❌ 모델 로드 실패: {e}")
            self.marshal_ai = None

    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode == 'MARSHALLER':
            self.get_logger().info("🚩 마샬러 모드 활성화!")

    def image_callback(self, msg):
        # [중요] 모드가 마샬러가 아니면 리턴 (AI 연산도 하지 않아 부하 감소)
        if self.current_mode != 'MARSHALLER':
            return
            
        if self.marshal_ai is None: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 오류: {e}")
            return

        # AI 인식
        action, _ = self.marshal_ai.detect_gesture(frame)
        
        # 로봇 제어
        self.process_command(action)

    def process_command(self, action):
        twist = Twist()
        msg_grip = String()
        
        speed_fast = self.get_parameter('speed_fast').value
        speed_slow = self.get_parameter('speed_slow').value
        turn_ang = self.get_parameter('turn_angle').value

        # [로그 출력 강화]
        if action == "FORWARD":
            twist.linear.x = float(speed_fast)
            self.get_logger().info(f"🚗 전진 (FAST) - {action}")
            
        elif action == "APPROACHING":
            twist.linear.x = float(speed_slow)
            self.get_logger().info(f"🚗 접근 (SLOW) - {action}")
            
        elif action == "TURN_LEFT":
            twist.linear.x = float(speed_slow)
            twist.angular.z = float(turn_ang)
            self.get_logger().info(f"↩️ 좌회전 - {action}")
            
        elif action == "TURN_RIGHT":
            twist.linear.x = float(speed_slow)
            twist.angular.z = -float(turn_ang)
            self.get_logger().info(f"↪️ 우회전 - {action}")
            
        elif action == "STOP" or action == "SET_BRAKES":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f"🛑 정지 - {action}")
            
        elif action == "GRIPPER_HOLD":
            msg_grip.data = "GRIP"
            self.gripper_pub.publish(msg_grip)
            self.get_logger().info(f"✊ 그리퍼 잡기 - {action}")
            
        elif action == "GRIPPER_RELEASE":
            msg_grip.data = "OPEN"
            self.gripper_pub.publish(msg_grip)
            self.get_logger().info(f"🖐 그리퍼 놓기 - {action}")
            
        elif action == "RESET":
            msg_grip.data = "INIT"
            self.gripper_pub.publish(msg_grip)
            self.get_logger().info(f"🔄 리셋 - {action}")
            
        else:
            # 인식은 되지만 주행 명령이 아닌 상태 (READY, FACE_ME, STAGE_X 등)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # 1초마다 로그 출력 (로그 폭주 방지)
            self.get_logger().info(f"⏳ 대기 중: {action}", throttle_duration_sec=1.0)

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
