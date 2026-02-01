#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionFinal(Node):
    def __init__(self):
        super().__init__('vision_final')
        
        # 1. 구독: mqtt_final.py가 보내는 모드 메시지 수신
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        
        # 2. 발행: 현재 상태 및 웹서버용 이미지 송출
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # 3. 카메라 설정
        self.FRONT_CAM_IDX = 0
        self.REAR_CAM_IDX = 2
        
        self.cap_front = None
        self.cap_rear = None
        
        # 초기 상태는 IDLE
        self.current_mode = "IDLE"

        # 4. 카메라 2대 동시 초기화 (Raw 방식 - 딜레이 0초)
        self.init_cameras()
        
        # 5. 루프 실행 (30Hz)
        self.timer = self.create_timer(0.033, self.vision_loop)
        
        self.get_logger().info(f"🎥 Vision Final Ready (Waiting for mqtt_final cmd)")

    def init_cameras(self):
        self.get_logger().info("📷 카메라 2대 동시 초기화 중...")
        self.cap_front = self.open_camera(self.FRONT_CAM_IDX)
        self.cap_rear = self.open_camera(self.REAR_CAM_IDX)
        
        if self.cap_front and self.cap_rear:
            self.get_logger().info("✅ 전방/후방 카메라 준비 완료!")
        else:
            self.get_logger().warn("⚠️ 일부 카메라가 열리지 않았습니다. 연결을 확인하세요.")

    def open_camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        # 대역폭 절약을 위해 해상도와 포맷 지정
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if cap.isOpened():
            return cap
        return None

    def mode_callback(self, msg):
        # mqtt_final에서 오는 메시지("DOCKING", "MARSHALLER", "IDLE") 수신
        if self.current_mode != msg.data:
            self.get_logger().info(f"⚡ 모드 변경 감지: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data

    def vision_loop(self):
        status_msg = String()
        target_frame = None
        
        # 1. 버퍼 비우기 (항상 두 카메라를 모두 읽어야 딜레이가 안 생김)
        ret_f, frame_front = False, None
        ret_r, frame_rear = False, None
        
        if self.cap_front: ret_f, frame_front = self.cap_front.read()
        if self.cap_rear:  ret_r, frame_rear = self.cap_rear.read()

        # 2. 모드에 따른 이미지 선택 (mqtt_final이 보내는 단어와 정확히 일치시킴)
        if self.current_mode == "DOCKING":
            if ret_r:
                target_frame = frame_rear
                status_msg.data = "Mode: DOCKING (Rear Cam)"
                # 화면에 텍스트 표시 (확인용)
                cv2.putText(target_frame, "DOCKING MODE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                status_msg.data = "Error: DOCKING (No Rear Cam)"

        elif self.current_mode == "MARSHALLER":
            if ret_f:
                target_frame = frame_front
                status_msg.data = "Mode: MARSHALLER (Front Cam)"
                cv2.putText(target_frame, "MARSHALLER MODE", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                status_msg.data = "Error: MARSHALLER (No Front Cam)"
        
        else:
            # IDLE 이거나 DRIVING 등 기타 상태일 때
            status_msg.data = f"Mode: {self.current_mode}"
            if ret_f:
                # IDLE 상태여도 웹서버가 살아있는지 보여주기 위해 전방 카메라 송출 (선택사항)
                target_frame = frame_front
                cv2.putText(target_frame, f"IDLE / {self.current_mode}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        # 3. 이미지 ROS 토픽 발행 -> web_video_server가 이걸 가져감
        if target_frame is not None:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(target_frame, "bgr8")
                self.image_pub.publish(ros_image)
            except Exception as e:
                pass

        # 상태 메시지 발행
        self.vision_pub.publish(status_msg)

    def destroy_node(self):
        if self.cap_front: self.cap_front.release()
        if self.cap_rear: self.cap_rear.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
