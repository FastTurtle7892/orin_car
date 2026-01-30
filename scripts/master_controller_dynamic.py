#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

import json
import time
import math
import paho.mqtt.client as mqtt
import subprocess # [핵심] 외부 프로세스 실행용

# ================= 설정 =================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"
# ========================================

class MasterControllerDynamic(Node):
    def __init__(self):
        super().__init__('master_controller')
        
        # 카메라 프로세스 관리 변수
        self.cam_process = None
        self.active_cam = None  # 'FRONT', 'REAR', None

        # 퍼블리셔 & 서브스크라이버
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.current_mode = "IDLE"
        
        # MQTT 설정
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Error: {e}")

        self.get_logger().info("✅ Dynamic Master Controller Started")

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd = payload.get("cmd")
            
            self.get_logger().info(f"📩 CMD: {cmd}")

            if cmd == "START_MISSION" or cmd == "GO_HOME": 
                self.change_mode("NAV")
                self.switch_camera(None) # 주행 중엔 카메라 끔 (라이다 집중)

            elif cmd == "DOCKING": 
                self.change_mode("DOCKING")
                self.switch_camera("REAR") # 후방 카메라 ON

            elif cmd == "MARSHALLING": 
                self.change_mode("MARSHALLING")
                self.switch_camera("FRONT") # 전면 카메라 ON

            elif cmd == "STOP": 
                self.change_mode("IDLE")
                self.switch_camera(None) # 정지 시 카메라 끔

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def change_mode(self, new_mode):
        self.current_mode = new_mode
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def switch_camera(self, target_cam):
        """ 카메라 프로세스를 동적으로 끄고 켭니다 """
        if self.active_cam == target_cam:
            return # 이미 해당 카메라가 켜져 있으면 패스

        self.get_logger().info(f"📷 Switching Camera: {self.active_cam} -> {target_cam}")

        # 1. 기존 카메라 끄기
        if self.cam_process:
            self.cam_process.terminate()
            self.cam_process.wait() # 완전히 꺼질 때까지 대기
            self.cam_process = None
            self.active_cam = None
            self.get_logger().info("⏹ Camera Stopped")

        # 2. 새 카메라 켜기
        if target_cam == 'FRONT':
            self.cam_process = subprocess.Popen(['ros2', 'launch', 'orin_car', 'front_cam.launch.py'])
            self.active_cam = 'FRONT'
            self.get_logger().info("▶ Front Camera Launched")
        
        elif target_cam == 'REAR':
            self.cam_process = subprocess.Popen(['ros2', 'launch', 'orin_car', 'rear_cam.launch.py'])
            self.active_cam = 'REAR'
            self.get_logger().info("▶ Rear Camera Launched")

def main(args=None):
    rclpy.init(args=args)
    node = MasterControllerDynamic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 카메라 프로세스 정리
        if node.cam_process:
            node.cam_process.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
