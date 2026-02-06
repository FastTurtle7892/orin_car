#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class MarshallerSuperSimple(Node):
    def __init__(self):
        super().__init__('marshaller_super_simple')
        self.get_logger().info("🚀 초심플 마샬러 시작 (전진/정지 ONLY) 🚀")
        
        # 1. Jetson GPU 모델 로드
        self.abs_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.engine'
        self.pt_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.pt'
        
        try:
            self.model = YOLO(self.abs_path, task='pose')
            self.get_logger().info("✅ TensorRT Engine 로드됨")
        except:
            self.model = YOLO(self.pt_path)
            self.get_logger().info("⚠️ .pt 모델 로드됨")

        # 2. ROS 설정
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/marshaller/debug_image', 10)
        self.bridge = CvBridge()
        
        self.cap = None
        self.drive_state = "STOP"
        self.frame_count = 0
        self.SKIP_FRAMES = 2 # 3프레임당 1번 추론 (부하 감소)

        self.create_timer(0.1, self.loop)

    def loop(self):
        # 카메라 열기
        if self.cap is None:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, 640); self.cap.set(4, 480)
        
        ret, frame = self.cap.read()
        if not ret: return

        # 프레임 스킵
        self.frame_count += 1
        if self.frame_count % (self.SKIP_FRAMES + 1) != 0:
            return 

        # AI 추론
        results = self.model(frame, verbose=False, conf=0.5, device=0, half=True)
        
        action = "STOP" # 기본은 정지

        # 사람이 감지되었을 때만 판단
        if results[0].keypoints is not None and len(results[0].keypoints.data) > 0:
            kpts = results[0].keypoints.data[0].cpu().numpy()
            
            # 좌표 추출 (Y좌표는 작을수록 위쪽)
            l_sh_y, r_sh_y = kpts[5][1], kpts[6][1]   # 어깨 Y
            l_wr_y, r_wr_y = kpts[9][1], kpts[10][1]  # 손목 Y
            l_wr_x, r_wr_x = kpts[9][0], kpts[10][0]  # 손목 X
            
            # [조건 1] 가슴 모으기 (강력 정지)
            body_center = (kpts[5][0] + kpts[6][0]) / 2
            shoulder_width = abs(kpts[5][0] - kpts[6][0])
            
            # 손목이 몸통 중앙에 모였는가?
            is_gathered = abs(l_wr_x - body_center) < shoulder_width * 0.8 and \
                          abs(r_wr_x - body_center) < shoulder_width * 0.8
            # 손이 어깨보단 아래에 있는가? (만세 아님)
            is_down = l_wr_y > l_sh_y and r_wr_y > r_sh_y

            if is_gathered and is_down:
                action = "STOP_CHEST"
            
            # [조건 2] 전진 (양손 만세)
            elif l_wr_y < l_sh_y and r_wr_y < r_sh_y:
                action = "FORWARD"
            
            # 그 외 (한 손만 들거나, 차렷 자세 등) -> STOP

        # 상태 업데이트 및 모터 제어
        twist = Twist()
        
        if action == "FORWARD":
            self.drive_state = "FORWARD"
            twist.linear.x = 0.35
            
        elif action == "STOP_CHEST":
            self.drive_state = "STOP (CHEST)"
            twist.linear.x = 0.0
            cv2.putText(frame, "CHEST STOP!", (200, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)
            
        else: # STOP
            self.drive_state = "STOP"
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

        # 화면 표시
        color = (0, 255, 0) if "FORWARD" in self.drive_state else (0, 0, 255)
        cv2.putText(frame, f"CMD: {self.drive_state}", (20, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main():
    rclpy.init()
    node = MarshallerSuperSimple()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()