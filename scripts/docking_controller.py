#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time
from docking_ai import DockingAI
from rclpy.qos import qos_profile_sensor_data

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # ================= [설정] =================
        self.TARGET_DIST_CM = 16.4   # 정지 및 잡기 시작 거리
        self.BASE_SPEED = -0.15      # 후진 속도
        self.KP_STEER = 0.02         
        
        # test_gripper.py 처럼 단계별로 충분히 기다립니다 (1.5초)
        self.STEP_WAIT_TIME = 1.5    
        # ==========================================

        # [통합 모드 관리]
        self.current_mode = "IDLE"
        self.create_subscription(String, '/robot_mode', self.mode_callback, 10)

        self.create_subscription(Image, '/rear_camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        self.bridge = CvBridge()
        self.docking_ai = DockingAI() 
        
        # state_mode -> 0: 초기화 대기, 1: 도킹 주행, 2: 잡기 시퀀스, 99: 완료
        self.state_mode = 0  
        self.docking_step = 0           
        self.docking_start_time = 0.0
        self.last_log_time = 0

        # 시작 1초 후 초기 자세(INIT) 잡기 (단, DOCKING 모드일 때만)
        self.create_timer(1.0, self.initialize_pose_once)
        self.get_logger().info("✅ Docking Controller Started (Waiting for 'DOCKING' mode)")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        # 모드가 DOCKING으로 바뀌면 상태 리셋 등의 로직을 넣을 수도 있음
        if self.current_mode == 'DOCKING':
             self.get_logger().info("🚩 도킹 모드 활성화!")

    def initialize_pose_once(self):
        # 모드가 DOCKING이 아니면 초기화도 보류
        if self.current_mode != 'DOCKING':
            return

        if self.state_mode == 0:
            self.get_logger().info("🏁 [INIT] Pose Setup (UP & OPEN)")
            self.publish_gripper("INIT") 
            self.state_mode = 1 
            # 초기화 동작 완료 대기
            time.sleep(2.0)

    def image_callback(self, msg):
        # [디버깅 1] 함수가 호출되는지 확인
        # (너무 많이 뜨면 나중에 주석 처리하세요)
        # self.get_logger().info("📷 영상 수신 중...", throttle_duration_sec=2.0)

        # [중요] 내 모드가 아니면 동작 중지
        if self.current_mode != 'DOCKING':
            return

        # 1. 시퀀스 진행 중이면 영상 처리 중단하고 시퀀스 함수 실행
        if self.state_mode == 2:
            self.run_gripper_sequence()
            return
        
        # 완료 상태면 아무것도 안 함
        if self.state_mode == 99 or self.state_mode == 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            # [디버깅 2] 변환 에러 확인
            self.get_logger().error(f"❌ 이미지 변환 실패: {e}")
            return

        # AI 처리
        result = self.docking_ai.process(cv_image)
        
        # 결과 유효성 체크
        if not isinstance(result, (list, tuple)) or len(result) < 2:
            self.stop_robot()
            return

        data, frame = result
        found = data.get("found", False)

        # [디버깅 3] AI 결과 확인 (Searching이 안 뜨는 이유 확인)
        if not found:
            if time.time() - self.last_log_time > 2.0:
                self.get_logger().info("👀 Searching... (마커 찾는 중)")
                self.last_log_time = time.time()
            self.stop_robot()
            return

        dist_cm = data.get("dist_cm", 999.9)
        x_cm = data.get("x_cm", 0.0)
        
        # 거리값이 None이면 안전하게 처리
        if dist_cm is None: dist_cm = 999.9

        # [거리 도달 체크]
        if dist_cm <= self.TARGET_DIST_CM:
            self.get_logger().info(f"🛑 [ARRIVED] Distance {dist_cm:.1f}cm <= {self.TARGET_DIST_CM}cm")
            self.get_logger().info("🚀 Starting Grip Sequence!")
            self.stop_robot()
            
            # 잡기 시퀀스 시작
            self.state_mode = 2
            self.docking_step = 1  
            self.docking_start_time = time.time() 
        else:
            # 주행 (후진)
            # [디버깅 4] 주행 명령이 나가는지 확인
            self.get_logger().info(f"🚗 Approaching... {dist_cm:.1f}cm (x: {x_cm})")
            twist = Twist()
            twist.linear.x = self.BASE_SPEED
            twist.angular.z = self.KP_STEER * x_cm 
            self.cmd_vel_pub.publish(twist)

    def run_gripper_sequence(self):
        """ 
        [잡기 시퀀스]
        """
        elapsed = time.time() - self.docking_start_time

        # Step 1: 리프트 내리기
        if self.docking_step == 1:
            if elapsed > 0.5: # 정지 후 약간 안정화
                self.get_logger().info("🔽 [1/3] Lift DOWN (140)")
                self.publish_gripper("DOWN")
                self.next_step(2)

        # Step 2: 내리기 완료 대기 -> 잡기
        elif self.docking_step == 2:
            if elapsed > self.STEP_WAIT_TIME:
                self.get_logger().info("✊ [2/3] Gripper CLOSE (120)")
                self.publish_gripper("GRIP")
                self.next_step(3)

        # Step 3: 잡기 완료 대기 -> 올리기
        elif self.docking_step == 3:
            if elapsed > self.STEP_WAIT_TIME:
                self.get_logger().info("🔼 [3/3] Lift UP (160)")
                self.publish_gripper("UP")
                self.next_step(4)

        # Step 4: 종료
        elif self.docking_step == 4:
            if elapsed > self.STEP_WAIT_TIME:
                self.get_logger().info("✅ Mission Complete! Object Secured.")
                self.state_mode = 99 # 종료 상태로 전환 (더 이상 주행 안 함)

    def next_step(self, next_step_num):
        self.docking_step = next_step_num
        self.docking_start_time = time.time()

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_gripper(self, cmd):
        msg = String()
        msg.data = cmd
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
