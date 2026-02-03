#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import sys
import time
import os

# [그리퍼 라이브러리]
from adafruit_servokit import ServoKit
import board
import busio

# [비전 라이브러리 경로]
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from docking_ai_test import DockingAI

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # 1. ROS 설정
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # 2. 카메라 & AI
        self.ai = DockingAI()
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("🚨 카메라 에러! (/dev/video2)")
            sys.exit(1)

        # 3. 서보모터 설정
        self.LIFT_CHANNEL = 1
        self.GRIPPER_CHANNEL = 2
        self.kit = None
        self.init_gripper()

        # 4. 주행 파라미터
        self.TARGET_DIST = 14.4
        self.STOP_TOLERANCE = 1.0 
        self.FIXED_SPEED = -0.15
        
        self.is_docked = False
        self.last_known_dist = 999.0 

        self.get_logger().info("✅ Docking Controller Ready! (로그 출력 켜짐)")

    def init_gripper(self):
        try:
            i2c_bus0 = busio.I2C(board.SCL, board.SDA)
            self.kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x60)
            
            # 초기 자세
            self.kit.servo[self.LIFT_CHANNEL].angle = 160
            time.sleep(0.5)
            self.kit.servo[self.GRIPPER_CHANNEL].angle = 70
            time.sleep(0.5)
            self.get_logger().info("✅ 서보 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ 서보 연결 실패: {e}")

    def execute_grip_sequence(self):
        if self.kit is None:
            self.get_logger().error("❌ 서보가 연결되지 않았습니다!")
            return

        self.get_logger().info("🚀 잡기 시퀀스 시작!")
        
        # 동작 수행
        self.kit.servo[self.LIFT_CHANNEL].angle = 140
        time.sleep(2.0)
        self.kit.servo[self.GRIPPER_CHANNEL].angle = 120
        time.sleep(2.0)
        self.kit.servo[self.LIFT_CHANNEL].angle = 160
        time.sleep(2.0)
        
        self.get_logger().info("✅ 잡기 완료!")

    def timer_callback(self):
        if self.is_docked: return

        ret, frame = self.cap.read()
        if not ret: return

        data, processed_frame = self.ai.process(frame)
        cmd_msg = Twist()

        # [상태 판별 로직]
        if data["found"]:
            dist = data["dist_cm"]
            x_err = data["x_cm"]
            yaw = data["yaw"]  # Yaw 값도 가져오기
            self.last_known_dist = dist 

            # [▼▼▼ 여기가 추가된 로그 출력 부분입니다 ▼▼▼]
            self.get_logger().info(f"Dist: {dist:.1f}cm | X: {x_err:.1f} | Yaw: {yaw:.1f}")

            error_dist = dist - self.TARGET_DIST
            
            # (A) 도착 판정
            if abs(error_dist) <= self.STOP_TOLERANCE:
                self.perform_docking(dist)
                return

            # (B) 주행
            else:
                speed = self.FIXED_SPEED
                if error_dist < -self.STOP_TOLERANCE: speed = -self.FIXED_SPEED 

                k_steer = 0.05
                steer = max(min(x_err * k_steer, 0.5), -0.5)

                cmd_msg.linear.x = speed
                cmd_msg.angular.z = steer

        # 2. 마커를 놓쳤을 때
        else:
            if self.last_known_dist <= (self.TARGET_DIST + 3.0):
                self.get_logger().warn(f"⚠️ 마커 놓침! 하지만 도착 간주 (Last: {self.last_known_dist:.1f})")
                self.perform_docking(self.last_known_dist)
                return
            
            else:
                # 멀리서 놓치면 로그 한번만 출력 (도배 방지)
                # self.get_logger().info("Searching...", once=True) 
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0

        self.cmd_publisher.publish(cmd_msg)

    def perform_docking(self, dist):
        self.stop_robot()
        self.get_logger().info(f"🎯 도착 확인 (거리: {dist:.1f}cm)")
        
        # 그리퍼 동작
        self.execute_grip_sequence()
        
        self.is_docked = True
        
        # [핵심 추가] 잡기 끝났으면 스스로 종료! (자폭)
        self.get_logger().info("✅ 임무 완수! 노드를 종료하고 조종 권한을 넘깁니다.")
        
        # 1. 종료 전에 정지 신호 한 번 더
        self.stop_robot()
        
        # 2. 카메라 끄기
        if self.cap.isOpened():
            self.cap.release()
            
        # 3. 프로그램 강제 종료 -> 그래야 Teleop이 먹힘
        sys.exit(0)

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_publisher.publish(stop_msg)

    def __del__(self):
        if self.cap.isOpened(): self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = DockingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.get_logger().info("🛑 비상 정지...")
        stop_msg = Twist()
        for _ in range(10):
            node.cmd_publisher.publish(stop_msg)
            time.sleep(0.05)
            
        if node.cap.isOpened(): node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()