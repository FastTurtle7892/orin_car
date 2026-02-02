#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 멀티스레드 추가
from rclpy.callback_groups import ReentrantCallbackGroup # 콜백 그룹 추가
from std_msgs.msg import String
import cv2
import time

class VisionFinal(Node):
    def __init__(self):
        super().__init__('vision_final')
        
        # 멀티스레드 환경에서 콜백들이 서로 방해하지 않도록 설정
        self.callback_group = ReentrantCallbackGroup()

        # 1. 토픽 구독 (모드 변경을 최우선으로 처리)
        self.create_subscription(
            String, 
            '/system_mode', 
            self.mode_callback, 
            10,
            callback_group=self.callback_group)
            
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        
        self.FRONT_CAM_IDX = 0
        self.REAR_CAM_IDX = 2
        self.cap = None
        self.current_mode = "IDLE"

        # 2. 비전 루프 타이머 (주기를 10Hz 정도로 낮추는 것을 권장)
        self.timer = self.create_timer(0.1, self.vision_loop, callback_group=self.callback_group)
        
        self.get_logger().info(f"🎥 Vision Final Started (Thread-Safe Mode)")

    def mode_callback(self, msg):
        if self.current_mode != msg.data:
            self.get_logger().info(f"⚡ 모드 변경 감지: {self.current_mode} -> {msg.data}")
            self.current_mode = msg.data
            self.switch_camera()

    def switch_camera(self):
        """ 기존 카메라를 끄고 새 카메라를 여는 과정에서 발생하는 프리징 방지 """
        if self.cap is not None:
            self.get_logger().info("🔄 기존 카메라 해제 중...")
            self.cap.release()
            self.cap = None
            cv2.destroyAllWindows()
            time.sleep(1.0) # USB 버스 안정화 시간 확보

        if self.current_mode == "IDLE":
            return

        target_cam = self.REAR_CAM_IDX if self.current_mode == "DOCKING" else self.FRONT_CAM_IDX
        
        try:
            self.cap = self.open_camera(target_cam)
            if self.cap and self.cap.isOpened():
                self.get_logger().info(f"✅ 카메라 연결 성공 (Index: {target_cam})")
            else:
                self.get_logger().error(f"❌ 카메라 열기 실패")
        except Exception as e:
            self.get_logger().error(f"❌ 스위칭 중 예외 발생: {e}")

    def open_camera(self, index):
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 20) # FPS 하향
        return cap

    def vision_loop(self):
        status_msg = String()

        if self.cap is None or not self.cap.isOpened():
            status_msg.data = f"Mode: {self.current_mode} (Wait/Off)"
            self.vision_pub.publish(status_msg)
            return

        try:
            # USB 이탈 감지를 위해 짧은 타임아웃 개념으로 읽기
            ret, frame = self.cap.read()
            
            if ret:
                cv2.putText(frame, f"MODE: {self.current_mode}", (30, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("Robot Camera View", frame)
                cv2.waitKey(1)
                status_msg.data = f"Mode: {self.current_mode} (Active)"
            else:
                raise Exception("Frame Read Failed") # USB 뽑힘 등 발생 시 에러 강제 발생

        except Exception as e:
            self.get_logger().error(f"🚨 카메라 통신 두절: {e}")
            if self.cap:
                self.cap.release()
            self.cap = None
            cv2.destroyAllWindows()
            status_msg.data = "Error: Camera Disconnected"

        self.vision_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionFinal()
    
    # [핵심] 멀티스레드 실행기 사용
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
