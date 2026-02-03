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
        self.get_logger().info("🚀 마샬러 차량 제어 시작 🚀") 
        self.get_logger().info("====================================")
        # 1. Pub/Sub 설정
        self.mode_sub = self.create_subscription(
            String, '/system_mode', self.mode_callback, 10
        )
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/marshaller/debug_image', 10)

        # 2. 상태 및 객체 초기화
        self.current_mode = "WAITING"  # 초기 상태
        self.bridge = CvBridge()
        self.cap = None
        self.marshaller_ai = None
        self.wait_tick = 0 # 대기 로그 카운터
        
        # 전방 카메라 인덱스 (도킹이 2번이면, 전방은 보통 0번)
        self.CAMERA_INDEX = 0 
        
        # 주기적 실행 (0.1초 단위)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("✅ Marshaller Controller Initialized (Log Enhanced Version)")

    def mode_callback(self, msg):
        previous_mode = self.current_mode
        self.current_mode = msg.data
        if previous_mode != self.current_mode:
            self.get_logger().info(f"🔄 모드 변경 감지: {previous_mode} -> {self.current_mode}")

    def open_camera(self):
        """카메라가 닫혀있으면 열기"""
        if self.cap is None:
            self.get_logger().info(f"📷 Opening Front Camera ({self.CAMERA_INDEX})...")
            self.cap = cv2.VideoCapture(self.CAMERA_INDEX)
            
            # 해상도 및 FPS 설정 (Jetson 부하 줄이기)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 20)
            
            if not self.cap.isOpened():
                self.get_logger().error("❌ Failed to open Front Camera!")
                self.cap = None
            else:
                self.get_logger().info("✅ Camera Open Success")
                # AI 모델도 카메라 켤 때 초기화 (또는 미리 로드)
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
            # 2초(20틱)마다 생존 신고 로그
            self.wait_tick += 1
            #if self.wait_tick % 20 == 0:
            #    self.get_logger().info(f"💤 대기중... (현재 모드: {self.current_mode} / 'MARSHAL' 기다림)")
            return

        self.wait_tick = 0
        
        # 2. MARSHAL 모드면 카메라 켜기
        self.open_camera()
        if self.cap is None: return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️ Camera Read Error", throttle_duration_sec=2.0)
            return

        # 3. AI 추론 및 행동 결정
        action, out_frame = self.marshaller_ai.detect_gesture(frame)

        # [로그 강화] AI가 감지한 액션 실시간 확인 (SKIP, IDLE은 가끔, 나머지는 즉시)
        if action not in ["SKIP", "IDLE"]:
            self.get_logger().info(f"👀 AI 감지 성공: [{action}]")
        else:
            # 아무것도 안 잡혀도 시스템이 도는지 확인하기 위해 2초마다 로그 출력
            self.get_logger().info(f"👀 제스처 찾는 중... (현재: {action})", throttle_duration_sec=2.0)

        # 4. 행동에 따른 차량 제어 (cmd_vel)
        twist = Twist()
        should_publish = True
        
        if action == "FORWARD":
            twist.linear.x = 0.3
            self.get_logger().info(f"🚗 [GO] 전진 명령 생성 (v=0.2)")
            
        elif action == "BACKWARD":
            twist.linear.x = -0.3
            self.get_logger().info(f"🚗 [BACK] 후진 명령 생성 (v=-0.2)")
            
        elif action == "LEFT":
            twist.angular.z = 0.5
            self.get_logger().info(f"🔄 [LEFT] 좌회전 명령 생성")
            
        elif action == "RIGHT":
            twist.angular.z = -0.5
            self.get_logger().info(f"🔄 [RIGHT] 우회전 명령 생성")
            
        elif action == "STOP":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # 정지 명령은 너무 자주 뜨면 시끄러우니 1초에 한번만
            self.get_logger().info(f"🛑 [STOP] 정지 명령", throttle_duration_sec=1.0)
            
        elif action == "MANUAL_MODE":
            self.get_logger().info("🙌 수동 모드(MANUAL_DRIVE) 진입 - 제스처 대기 중", throttle_duration_sec=2.0)
            should_publish = False # MANUAL_MODE 자체는 움직임 명령이 아님
            
        elif action == "DOCKING":
            self.get_logger().info("🚀 [EVENT] 도킹 명령 수신! 도킹 모드로 전환합니다.")
            
            # 1. 차량 정지
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            
            # 2. 시스템 모드 변경
            mode_msg = String()
            mode_msg.data = "DOCKING"
            self.mode_pub.publish(mode_msg)
            
            # 3. 카메라 해제 및 종료
            self.close_camera()
            return  # 루프 종료

        # 제어 명령 발행 및 확인
        if should_publish:
            self.cmd_vel_pub.publish(twist)
            # 실제로 속도가 있을 때만 추가 로그 (확실한 디버깅용)
            if twist.linear.x != 0.0 or twist.angular.z != 0.0:
                 self.get_logger().info(f"📡 CMD 전송됨 -> Lin:{twist.linear.x:.1f}, Ang:{twist.angular.z:.1f}")

        # 5. 디버그 이미지 발행
        if self.debug_pub.get_subscription_count() > 0:
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