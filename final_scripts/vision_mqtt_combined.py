#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import paho.mqtt.client as mqtt
import json
import threading
import cv2
import time

class VisionMqttCombined(Node):
    def __init__(self):
        super().__init__('vision_mqtt_combined')
        self.callback_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # 1. ROS Publisher (제어기 및 AI 노드 전송용)
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', 10)
        self.vision_pub = self.create_publisher(String, '/vision_status', 10)
        self.image_pub = self.create_publisher(Image, '/camera/integrated_stream', 10)

        # 2. 내부 상태 변수
        self.current_mode = "IDLE"
        self.cap = None
        self.FRONT_CAM_IDX = 0
        self.REAR_CAM_IDX = 2

        # 3. MQTT 설정
        self.setup_mqtt()

        # 4. 비전 루프 타이머 (10Hz - Orin Nano 자원 최적화)
        self.timer = self.create_timer(0.1, self.vision_loop, callback_group=self.callback_group)
        
        self.get_logger().info("🚀 Vision + MQTT Integrated Node Started (No-GUI Mode)")

    def setup_mqtt(self):
        MQTT_BROKER = "i14a402.p.ssafy.io"
        MQTT_PORT = 8183
        CAR_ID = "car01"
        self.TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

        self.client = mqtt.Client()
        self.client.on_connect = self.on_mqtt_connect
        self.client.on_message = self.on_mqtt_message
        
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
            self.mqtt_thread.daemon = True
            self.mqtt_thread.start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Error: {e}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"✅ MQTT Connected. Topic: {self.TOPIC_CMD}")
        client.subscribe(self.TOPIC_CMD, qos=1)

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd = payload.get("cmd")
            self.get_logger().info(f"📩 MQTT Received: {cmd}")

            new_mode = self.current_mode
            if cmd == "START_PATH": new_mode = "DRIVING"
            elif cmd == "DOCKING_START": new_mode = "DOCKING"
            elif cmd == "MARSHALLER_START": new_mode = "MARSHALLER"
            elif cmd == "STOP": new_mode = "IDLE"

            if self.current_mode != new_mode:
                self.get_logger().info(f"⚡ Mode Change: {self.current_mode} -> {new_mode}")
                self.current_mode = new_mode
                
                # 다른 노드들에게 모드 변경 알림 (중요: 이걸 보내야 도킹 제어기가 작동함)
                mode_msg = String()
                mode_msg.data = self.current_mode
                self.mode_pub.publish(mode_msg)
                
                # 주행 경로 전달
                if cmd == "START_PATH":
                    path_data = payload.get("path_files") or payload.get("path_file")
                    if path_data:
                        p_msg = String()
                        p_msg.data = json.dumps(path_data)
                        self.path_pub.publish(p_msg)
                
                # 카메라 물리적 전환
                self.switch_camera()

        except Exception as e:
            self.get_logger().error(f"❌ MQTT Parsing Error: {e}")

    def switch_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(1.0) # USB 버스 안정화

        if self.current_mode == "IDLE":
            self.get_logger().info("💤 IDLE: Camera Off")
            return

        # 모드에 따른 카메라 인덱스 선택
        idx = self.REAR_CAM_IDX if self.current_mode == "DOCKING" else self.FRONT_CAM_IDX
        self.get_logger().info(f"📷 Opening Camera Index {idx} for {self.current_mode}")
        
        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)

    def vision_loop(self):
        # 1. 카메라가 안 켜져 있으면 상태만 보고
        if self.cap is None or not self.cap.isOpened():
            status_msg = String()
            status_msg.data = f"Mode: {self.current_mode} (Cam Off)"
            self.vision_pub.publish(status_msg)
            return
        
        # 2. 영상 읽기
        ret, frame = self.cap.read()
        if ret:
            # [수정] cv2.imshow 를 제거하여 GUI 에러 원천 차단
            # 대신 Image 토픽을 발행하여 도킹 제어기(AI)가 볼 수 있게 함
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(ros_image)
                
                status_msg = String()
                status_msg.data = f"Mode: {self.current_mode} (Active)"
                self.vision_pub.publish(status_msg)
            except Exception as e:
                self.get_logger().error(f"Image Pub Error: {e}")
        else:
            self.get_logger().warn("⚠️ Lost Camera Frame")

    def destroy_node(self):
        if self.cap: self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionMqttCombined()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
