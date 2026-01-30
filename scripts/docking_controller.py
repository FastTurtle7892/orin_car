#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import time

# docking_ai.py가 scripts 폴더 내에 있어야 합니다.
from docking_ai import DockingAI

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # [튜닝] 도킹 시작 거리 및 주행 속도
        self.TARGET_DIST_CM = 35.0   
        self.BASE_SPEED = -0.15      # 후진 속도
        self.KP_STEER = 0.02         # 조향 감도

        # 그리퍼 동작 대기 시간 (test_gripper.py 기준)
        self.WAIT_TIME = 1.5         

        self.create_subscription(Image, '/rear_camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        self.bridge = CvBridge()
        self.docking_ai = DockingAI() 
        
        self.is_docking_process = False 
        self.docking_step = 0           
        self.docking_start_time = 0.0

        self.get_logger().info("✅ Docking Controller (Synced with test_gripper.py) Started")

    def image_callback(self, msg):
        if self.is_docking_process:
            self.run_gripper_sequence()
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        # 마커 분석
        result = self.docking_ai.process(cv_image)
        
        # IndexError 방지를 위한 안전 장치
        if not isinstance(result, (list, tuple)) or len(result) < 4:
            self.stop_robot()
            return

        found = result[0]
        
        if found:
            dist_cm = result[2]
            x_cm = result[3]

            if dist_cm is None or x_cm is None:
                self.stop_robot()
                return

            # 인식 성공 시 거리 정보를 로그로 찍습니다.
            self.get_logger().info(f"Dist: {dist_cm:.1f}cm | X: {x_cm:.1f}")

            # 목표 거리 도달 여부 확인
            if dist_cm <= self.TARGET_DIST_CM:
                self.get_logger().info("🛑 Target Reached! Starting Sequence...")
                self.stop_robot()
                self.is_docking_process = True
                self.docking_start_time = time.time()
                self.docking_step = 1 
                return

            # 후진 주행 명령 전송
            twist = Twist()
            twist.linear.x = self.BASE_SPEED
            twist.angular.z = self.KP_STEER * x_cm 
            self.cmd_vel_pub.publish(twist)

        else:
            # 마커가 보이지 않으면 로그를 남기지 않고 정지합니다.
            self.stop_robot()

    def run_gripper_sequence(self):
        """ test_gripper.py 동작 반영: DOWN -> GRIP -> UP """
        elapsed = time.time() - self.docking_start_time

        # 1단계: 리프트 내림 (DOWN)
        if self.docking_step == 1:
            if elapsed > 1.0:
                self.get_logger().info("🔽 [1/3] Lift DOWN (90)")
                self.publish_gripper("DOWN")
                self.docking_step = 2
                self.docking_start_time = time.time()

        # 2단계: 잡기 (GRIP)
        elif self.docking_step == 2:
            if elapsed > self.WAIT_TIME:
                self.get_logger().info("✊ [2/3] Gripper GRIP (50)")
                self.publish_gripper("GRIP")
                self.docking_step = 3
                self.docking_start_time = time.time()

        # 3단계: 리프트 올림 (UP)
        elif self.docking_step == 3:
            if elapsed > self.WAIT_TIME:
                self.get_logger().info("🔼 [3/3] Lift UP (70)")
                self.publish_gripper("UP")
                self.docking_step = 4 
                self.docking_start_time = time.time()

        # 최종 완료
        elif self.docking_step == 4:
            if elapsed > self.WAIT_TIME:
                self.get_logger().info("✅ Docking Complete")
                self.is_docking_process = False 

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
