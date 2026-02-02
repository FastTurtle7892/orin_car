#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # 1. 후방 카메라 구독 (Jetson 내부 통신이라 'Raw' 이미지도 렉 없이 받음)
        self.subscription = self.create_subscription(
            Image,
            '/rear_camera/image_raw',
            self.image_callback,
            10)
        
        # 2. 로봇 이동 명령 발행 (cmd_vel)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. OpenCV 변환기
        self.br = CvBridge()
        
        self.get_logger().info("Docking Controller Started (Edge Computing Mode)")

    def image_callback(self, data):
        try:
            # [단계 1] ROS 이미지를 OpenCV 이미지로 변환
            # "bgr8" 포맷으로 변환 (컬러)
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
            
            # [단계 2] 여기서 AI / 영상 처리 로직 수행 (사용자님의 알고리즘)
            # 예시: 이미지를 흑백으로 바꾸고 가장 밝은 점을 찾아가기
            # ---------------------------------------------------------
            gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            # (예시 로직) 화면의 가로 크기 구하기
            height, width = gray.shape
            center_x = width // 2
            
            # (예시 로직) 밝은 부분(흰색 라인 등) 찾기 - 임계값 처리
            _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
            
            # 무게 중심(Moments) 찾기
            M = cv2.moments(thresh)
            
            cmd_msg = Twist()
            
            if M['m00'] > 0:
                # 라인(물체)의 중심 좌표 계산
                cx = int(M['m10'] / M['m00'])
                err = cx - center_x
                
                # P-Controller (화면 중심에 맞추기 위해 회전)
                # 오차가 양수면(오른쪽) -> 오른쪽 회전(음수) 필요 (좌표계 주의)
                # 여기서는 간단히 err가 양수(오른쪽)면 -> 왼쪽으로 턴하게 설정 (테스트 필요)
                k_p = 0.005
                angular_z = -float(err) * k_p
                
                cmd_msg.linear.x = 0.05  # 천천히 전진
                cmd_msg.angular.z = angular_z
                
                # 로그 출력 (디버깅용)
                # self.get_logger().info(f"Target found at {cx}, Turning: {angular_z:.3f}")
            else:
                # 아무것도 안 보이면 정지
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                # self.get_logger().info("Target lost - Stopping")
            
            # ---------------------------------------------------------

            # [단계 3] 결과 명령 발행 (이 데이터는 아주 작아서 와이파이 렉 안걸림)
            self.cmd_publisher.publish(cmd_msg)

            # (선택) 처리된 화면을 보고 싶을 때만 띄우기 (Jetson에 모니터 연결 시)
            # cv2.imshow("Camera View", current_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    docking_controller = DockingController()
    rclpy.spin(docking_controller)
    docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
