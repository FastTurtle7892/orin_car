#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data  # [핵심] 통신 호환성 해결

# 같은 폴더에 있는 DockingAI 모듈 임포트
from docking_ai import DockingAI

class DockingDriveTest(Node):
    def __init__(self):
        super().__init__('docking_drive_test')
        
        # =========================================================
        # [1] 튜닝 파라미터 (현장 상황에 맞춰 조절하세요)
        # =========================================================
        # 목표 정지 거리 (docking_ai의 초록색 구간인 8.5cm보다 약간 여유 있게)
        self.TARGET_DIST_CM = 12.0   
        
        self.BASE_SPEED = -0.75      # 후진 기본 속도 (음수)
        
        # 조향 방향 계수 (1.0 또는 -1.0)
        # [수정됨] 1.0 : x_cm가 양수(오른쪽)일 때 angular.z도 양수 
        # -> 핸들 오른쪽 -> 엉덩이 오른쪽으로 이동 (Ackermann 후진 정방향)
        self.STEER_DIR = 1.0        
        
        # PID 게인 설정
        # Yaw Gain: 각도가 틀어졌을 때 얼마나 꺾을지
        self.YAW_GAIN = 0.02         
        
        # X Gain: 좌우(cm)로 벗어났을 때 얼마나 꺾을지
        # 픽셀 단위가 아니라 cm 단위이므로 값을 좀 키웠습니다 (0.05 ~ 0.08 추천)
        self.X_GAIN = 0.05           

        # =========================================================
        
        self.docking_ai = DockingAI()
        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # [핵심] QoS 설정을 sensor_data로 지정하여 데이터 수신 보장
        self.create_subscription(
            Image, 
            '/rear_camera/image_raw', 
            self.image_callback, 
            qos_profile_sensor_data
        )
        
        self.get_logger().info("🚀 도킹 주행 테스트 Final (QoS + cm 제어 + 방향 1.0)")
        self.get_logger().info(f"🎯 목표 거리: {self.TARGET_DIST_CM}cm")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # AI 처리 (수정된 docking_ai.py의 리턴값 사용)
            data, annotated_frame = self.docking_ai.process(frame)
            
            twist = Twist()
            
            if data["found"]:
                dist_cm = data['dist_cm']
                x_cm = data['x_cm']    # cm 단위의 가로 오차 (음수:왼쪽, 양수:오른쪽)
                yaw = data['yaw']
                
                # 로그 출력 (상태 확인용)
                print(f"📏 거리:{dist_cm:.1f}cm | ↔ X오차:{x_cm:.2f}cm | 📐 Yaw:{yaw:.1f}°   ", end='\r')

                if dist_cm > self.TARGET_DIST_CM:
                    # 1. 속도 설정
                    twist.linear.x = self.BASE_SPEED
                    
                    # 2. 조향 계산 (비례 제어)
                    # 조향량 = (각도 오차 * 게인) + (위치 오차 * 게인)
                    steer_amount = (yaw * self.YAW_GAIN) + (x_cm * self.X_GAIN)
                    
                    # 3. 방향 적용 (후진 시 조향 방향 고려)
                    twist.angular.z = self.STEER_DIR * steer_amount
                    
                else:
                    # 목표 도착
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f"\n✅ 도킹 완료! (최종 거리: {dist_cm:.1f}cm, X오차: {x_cm:.1f}cm)")
            
            else:
                # 마커 놓침 -> 정지
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                print("❌ 마커 찾는 중...                   ", end='\r')

            self.cmd_pub.publish(twist)

            # [옵션] 디버깅용 화면 (SSH에서는 주석 처리 권장)
            # cv2.imshow("Docking View", annotated_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = DockingDriveTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
