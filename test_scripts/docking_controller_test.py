#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# [삭제] 멀티스레드 관련 라이브러리 제거
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2
import sys
import time
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from docking_ai_test import DockingAI

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🔒 [단일 스레드] 순차 실행 버전 🔒") 
        self.get_logger().info("====================================")

        # [삭제] 콜백 그룹 제거 (단일 스레드는 기본 그룹 사용)
        # self.callback_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.completion_pub = self.create_publisher(String, '/task_completion', 10)
        # 1. ROS 설정
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_publisher = self.create_publisher(String, '/gripper_cmd', 10)
        
        self.mode_sub = self.create_subscription(
            String, 
            '/system_mode', 
            self.mode_callback, 
            qos_profile
            # callback_group=self.callback_group [제거]
        )
        
        # 타이머 설정 (콜백 그룹 인자 제거)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.heartbeat_timer = self.create_timer(5.0, self.heartbeat_callback)

        # 2. AI & 카메라 설정
        self.ai = DockingAI()
        self.cap = None 
        self.camera_port = 2 
        
        self.is_camera_loading = False 
        self.is_docking_process_started = False
            
        self.create_timer(1.0, self.send_init_gripper)
        self.is_init_sent = False

        self.TARGET_DIST = 17.0
        self.STOP_TOLERANCE = 1.0 
        self.FIXED_SPEED = -0.25 
        
        self.system_mode = "IDLE"
        self.is_docked = False

    def heartbeat_callback(self):
        status = "ON" if (self.cap is not None and self.cap.isOpened()) else "OFF"
        if self.is_camera_loading: status = "LOADING..."
        
        if self.system_mode == "IDLE":
            self.get_logger().info(f"💤 대기중 (Camera: {status})", throttle_duration_sec=5.0)

    def mode_callback(self, msg):
        if self.system_mode != msg.data:
            self.get_logger().info(f"📨 모드 변경: {self.system_mode} -> {msg.data}")
            self.system_mode = msg.data
            
            if self.system_mode == "DOCKING":
                self.is_docked = False
                self.is_docking_process_started = False # 리셋
            else:
                self.stop_robot()
                self.manage_camera_resource()

    def manage_camera_resource(self):
        """ 안전하게 카메라 자원을 관리하는 함수 """
        
        # 1. 켜야 하는 상황
        if self.system_mode == "DOCKING" and not self.is_docked:
            if self.is_camera_loading: return
            if self.cap is not None and self.cap.isOpened(): return

            self.is_camera_loading = True
            self.get_logger().info(f"📷 카메라({self.camera_port}번) 연결 시도...")
            
            try:
                temp_cap = cv2.VideoCapture(self.camera_port)
                # MJPG 설정
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

        # 2. 꺼야 하는 상황
        else:
            if self.cap is not None:
                self.get_logger().info("💤 카메라 자원 반환 (OFF)")
                try:
                    if self.cap.isOpened():
                        self.cap.release()
                except Exception as e:
                    pass 
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
        # [단일 스레드 특징]
        # 여기서 time.sleep을 하면 로봇의 모든 기능(통신 포함)이 멈추고 이 동작만 수행합니다.
        # 오히려 도킹 중에는 이게 더 안전합니다.
        self.get_logger().info("🚀 잡기 시퀀스 시작")
        self.publish_gripper("DOWN"); time.sleep(2.0) 
        self.publish_gripper("GRIP"); time.sleep(2.0)
        self.publish_gripper("UP"); time.sleep(2.0)
        self.get_logger().info("✅ 잡기 완료")

    def timer_callback(self):
        if self.is_docking_process_started:
            return

        self.manage_camera_resource()

        if self.system_mode != "DOCKING": return
        if self.is_docked: return
        if self.is_camera_loading: return
        if self.cap is None or not self.cap.isOpened(): return

        ret, frame = self.cap.read()
        if not ret: 
            self.get_logger().error("❌ 영상 끊김! 재연결 시도...", throttle_duration_sec=1.0)
            if self.cap: self.cap.release()
            self.cap = None
            return

        try:
            data, processed_frame = self.ai.process(frame)
        except Exception:
            return

        cmd_msg = Twist()

        if data["found"]:
            dist = data["dist_cm"]
            x_err = data["x_cm"]
            
            self.get_logger().info(f"🟢 감지! 거리:{dist:.1f}cm", throttle_duration_sec=0.5)

            error_dist = dist - self.TARGET_DIST
            
            if abs(error_dist) <= self.STOP_TOLERANCE:
                self.is_docking_process_started = True 
                self.perform_docking(dist)
                return
            else:
                speed = self.FIXED_SPEED 
                if error_dist < -self.STOP_TOLERANCE: speed = -self.FIXED_SPEED 
                k_steer = 0.05
                steer = max(min(x_err * k_steer, 0.5), -0.5)
                
                cmd_msg.linear.x = speed
                cmd_msg.angular.z = steer
        else:
            self.get_logger().info(f"🔎 마커 찾는 중...", throttle_duration_sec=2.0)
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        self.cmd_publisher.publish(cmd_msg)

    # [2. perform_docking 함수 수정]
    def perform_docking(self, dist):
        self.stop_robot()
        self.get_logger().info(f"🎯 도착 완료! ({dist:.1f}cm)")
        
        self.execute_grip_sequence()
        
        self.is_docked = True
        self.system_mode = "IDLE" 
        
        self.manage_camera_resource()
        self.stop_robot()
        
        # [핵심 추가] MQTT 노드에게 "나 다 끝났어, 그만 보채!" 라고 신호 보내기
        done_msg = String()
        done_msg.data = "DOCKING_COMPLETE"
        self.completion_pub.publish(done_msg)
        self.get_logger().info("📢 [보고] 도킹 완료 신호 전송 -> MQTT 노드")
        
        self.is_docking_process_started = False

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_publisher.publish(stop_msg)

    def __del__(self):
        if self.cap is not None: 
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    
    # [수정] 멀티스레드 Executor 제거하고 기본 spin 사용
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()