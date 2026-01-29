#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import json
import paho.mqtt.client as mqtt
from ament_index_python.packages import get_package_share_directory

# 같은 폴더에 있는 AI 모듈 임포트
from docking_ai import DockingAI
from gesture_ai import MarshallerAI

# ==========================================
# [설정] 하드웨어 튜닝 값
# ==========================================
DOCKING_STOP_DIST_CM = 25.0   # 정지 거리 (cm) - 상황에 맞춰 조절
DOCKING_SPEED = -0.15         # 후진 속도
STEER_GAIN = 0.02             # 조향 민감도

MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_TOPIC = "autowing_car/v1/car01/cmd"

class MissionMaster(Node):
    def __init__(self):
        super().__init__('mission_master')
        
        # 1. 상태 관리
        self.state = "IDLE" 
        self.get_logger().info("🚀 Mission Master Started! State: IDLE")

        # 2. AI 모듈 로드
        # 모델 경로 안전하게 찾기 (패키지 내 config 폴더)
        try:
            pkg_path = get_package_share_directory('orin_car')
            model_file = os.path.join(pkg_path, 'config', 'yolov8n-pose.pt')
            
            self.docking_ai = DockingAI()
            # gesture_ai에 모델 경로 전달 (아래 gesture_ai.py 수정본 참조)
            self.marshaller_ai = MarshallerAI(model_path=model_file) 
            self.get_logger().info("✅ AI Modules Loaded")
        except Exception as e:
            self.get_logger().error(f"❌ AI Load Error: {e}")
            # 에러 나도 노드는 죽지 않게 처리 (테스트용)

        # 3. ROS 통신
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # 카메라 구독
        self.create_subscription(Image, '/rear_camera/image_raw', self.rear_cam_callback, 10)
        self.create_subscription(Image, '/front_camera/image_raw', self.front_cam_callback, 10)

        # 4. MQTT 연결
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        try:
            self.mqtt_client.connect(MQTT_BROKER, 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ MQTT Connected")
        client.subscribe(MQTT_TOPIC)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            cmd = payload.get("cmd")
            self.get_logger().info(f"📩 MQTT Cmd: {cmd}")

            if cmd == "START_DOCKING":
                self.state = "DOCKING"
            elif cmd == "START_MARSHALLING":
                self.state = "MARSHALLING"
            elif cmd == "STOP":
                self.state = "IDLE"
                self.stop_robot()
                
        except Exception as e:
            self.get_logger().error(f"MQTT Parsing Error: {e}")

    # [1] 도킹 로직 (후방 카메라)
    def rear_cam_callback(self, msg):
        if self.state != "DOCKING": return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            data, _ = self.docking_ai.process(frame)

            twist = Twist()
            
            if data["found"]:
                dist_cm = data['dist_cm']
                yaw = data['yaw']

                # 목표 거리보다 멀면 후진
                if dist_cm > DOCKING_STOP_DIST_CM:
                    twist.linear.x = DOCKING_SPEED 
                    # Yaw(각도)에 비례해 조향 (방향 반대면 -1.0 부호 변경)
                    twist.angular.z = -1.0 * (yaw * STEER_GAIN) 
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.get_logger().info(f"✅ Docking Reached! ({dist_cm:.1f}cm)")
            else:
                twist.linear.x = 0.0 # 마커 없으면 정지
            
            self.cmd_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Docking Error: {e}")

    # [2] 마샬러 로직 (전방 카메라)
    def front_cam_callback(self, msg):
        if self.state != "MARSHALLING": return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            action, _ = self.marshaller_ai.detect_gesture(frame)
            
            twist = Twist()
            
            if action == "FORWARD" or action == "APPROACHING":
                twist.linear.x = 0.2
            elif action in ["STOP", "ENGINE_CUT", "SET_BRAKES"]:
                twist.linear.x = 0.0
            
            self.cmd_pub.publish(twist)
        
        except Exception as e:
            self.get_logger().error(f"Marshaller Error: {e}")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = MissionMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
