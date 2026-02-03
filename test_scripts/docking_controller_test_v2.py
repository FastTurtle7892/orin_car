#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import sys
import time
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# [중요] 방금 만든 v2 파일을 불러옵니다.
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from docking_ai_test_v2 import DockingAI 

class DockingControllerV2(Node):
    def __init__(self):
        super().__init__('docking_controller_v2')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🚀 도킹 컨트롤러 V2 (Yaw 제어 포함) 시작") 
        self.get_logger().info("====================================")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # 1. ROS 통신 설정
        self.completion_pub = self.create_publisher(String, '/task_completion', 10)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(String, '/gripper_cmd', 10)
        
        self.mode_sub = self.create_subscription(
            String, '/system_mode', self.mode_callback, qos_profile
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat_callback)

        # 2. AI & 카메라 설정
        self.ai = DockingAI()
        self.cap = None 
        self.camera_port = 2  # 카메라 번호 확인 필요
        
        self.is_camera_loading = False 
        self.is_docking_process_started = False
            
        self.create_timer(1.0, self.send_init_gripper)
        self.is_init_sent = False

        # ==========================================
        # [핵심 튜닝 파라미터] - 여기서 조절하세요
        # ==========================================
        self.TARGET_DIST = 17.0       # 목표 정지 거리 (cm)
        self.STOP_TOLERANCE = 1.0     # 정지 오차 범위 (cm)
        self.FIXED_SPEED = -0.25      # 후진 속도 (음수)
        
        # 조향 게인 (값이 클수록 반응이 민감함)
        self.GAIN_LATERAL = 0.06      # 좌우 거리 오차(x_cm)에 대한 게인
        self.GAIN_YAW = 0.03          # 각도 오차(yaw)에 대한 게인 (새로 추가됨)
        
        self.MAX_STEER = 1.0          # 최대 조향값 제한 (-1.0 ~ 1.0)
        # ==========================================

        self.system_mode = "IDLE"
        self.is_docked = False

    def heartbeat_callback(self):
        # 상태 로깅 (필요시 주석 해제)
        pass

    def mode_callback(self, msg):
        if self.system_mode != msg.data:
            self.get_logger().info(f"📨 모드 변경: {self.system_mode} -> {msg.data}")
            self.system_mode = msg.data
            
            if self.system_mode == "DOCKING":
                self.is_docked = False
                self.is_docking_process_started = False 
            else:
                self.stop_robot()
                self.manage_camera_resource()

    def manage_camera_resource(self):
        if self.system_mode == "DOCKING" and not self.is_docked:
            if self.is_camera_loading: return
            if self.cap is not None and self.cap.isOpened(): return

            self.is_camera_loading = True
            self.get_logger().info(f"📷 카메라 연결 시도...")
            try:
                temp_cap = cv2.VideoCapture(self.camera_port)
                temp_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                temp_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                temp_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                temp_cap.set(cv2.CAP_PROP_FPS, 30)

                if temp_cap.isOpened():
                    self.cap = temp_cap
                    self.get_logger().info("✅ 카메라 연결 완료!")
                else:
                    self.get_logger().error("❌ 카메라 연결 실패")
            except Exception as e:
                self.get_logger().error(f"❌ 카메라 에러: {e}")
            finally:
                self.is_camera_loading = False
        else:
            if self.cap is not None:
                try:
                    self.cap.release()
                except: pass 
                finally:
                    self.cap = None
                    self.is_camera_loading = False

    def send_init_gripper(self):
        if not self.is_init_sent:
            self.publish_gripper("INIT")
            self.is_init_sent = True

    def publish_gripper(self, command):
        msg = String()
        msg.data = command
        self.gripper_publisher.publish(msg)

    def execute_grip_sequence(self):
        self.get_logger().info("🚀 잡기 시퀀스 시작")
        self.publish_gripper("DOWN"); time.sleep(2.0) 
        self.publish_gripper("GRIP"); time.sleep(2.0)
        self.publish_gripper("UP"); time.sleep(2.0)
        self.get_logger().info("✅ 잡기 완료")

    # =========================================================
    # [핵심] 조향 제어 로직 수정 부분
    # =========================================================
    def timer_callback(self):
        if self.is_docking_process_started: return

        self.manage_camera_resource()

        if self.system_mode != "DOCKING": return
        if self.is_docked: return
        if self.cap is None or not self.cap.isOpened(): return

        ret, frame = self.cap.read()
        if not ret: return

        try:
            data, processed_frame = self.ai.process(frame)
        except Exception: return

        cmd_msg = Twist()

        if data["found"]:
            dist = data["dist_cm"]
            x_err = data["x_cm"]   # 좌우 거리 (cm)
            yaw_err = data["yaw"]  # 틀어진 각도 (degree)
            
            # 1. 거리 체크 (도킹 완료 여부)
            error_dist = dist - self.TARGET_DIST
            
            if abs(error_dist) <= self.STOP_TOLERANCE:
                self.is_docking_process_started = True 
                self.perform_docking(dist)
                return
            
            # 2. 주행 속도 결정 (거리에 따라)
            speed = self.FIXED_SPEED 
            # 만약 너무 가까우면(목표보다 더 안쪽이면) 전진해서 거리 벌림
            if error_dist < -self.STOP_TOLERANCE: 
                speed = -self.FIXED_SPEED 

            # 3. 조향 계산 (Lateral + Angular Control)
            # 후진 시 조향 방향은 전진과 반대일 수 있으므로 테스트 필요
            # 공식: Steering = (거리오차 * K1) + (각도오차 * K2)
            
            # [테스트 팁]
            # 만약 차가 반대로 튄다면 GAIN 앞에 부호(-)를 붙여보세요.
            # 예: steer = - (x_err * ...) 
            steer = -((x_err * self.GAIN_LATERAL) + (yaw_err * self.GAIN_YAW))
            
            # 최대 조향각 제한
            steer = max(min(steer, self.MAX_STEER), -self.MAX_STEER)
            
            cmd_msg.linear.x = speed
            cmd_msg.angular.z = steer
            
            self.get_logger().info(
                f"Dist:{dist:.1f}cm | X_err:{x_err:.1f} | Yaw:{yaw_err:.1f} | Steer:{steer:.2f}", 
                throttle_duration_sec=0.2
            )
        else:
            # 마커 놓치면 정지
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_publisher.publish(cmd_msg)

    def perform_docking(self, dist):
        self.stop_robot()                         
        self.get_logger().info(f"🎯 도착 완료! ({dist:.1f}cm)")
        self.execute_grip_sequence()
        
        self.is_docked = True
        #self.system_mode = "IDLE" 
        
        self.manage_camera_resource()
        self.stop_robot()
        
        done_msg = String()
        done_msg.data = "DOCKING_COMPLETE"
        self.completion_pub.publish(done_msg)
        self.get_logger().info("📢 도킹 완료 신호 전송")
        
        self.is_docking_process_started = False

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_publisher.publish(stop_msg)

    def __del__(self):
        if self.cap is not None: self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()