#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import paho.mqtt.client as mqtt
import subprocess
import os
import signal
import sys

# ================= 설정 =================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
# ========================================

class MasterControllerDynamic(Node):
    def __init__(self):
        super().__init__('master_controller')
        
        self.cam_process = None
        self.active_cam = None 
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
            self.get_logger().info(f"✅ MQTT Connected: {TOPIC_CMD}")
        except Exception as e:
            self.get_logger().error(f"MQTT Error: {e}")

        self.get_logger().info("✅ Dynamic Master Controller Started (Waiting for CMD)")

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd = payload.get("cmd")
            self.get_logger().info(f"📩 RECEIVED CMD: {cmd}")

            if cmd == "DOCKING": 
                self.change_mode("DOCKING")
                self.switch_camera("REAR")

            elif cmd == "MARSHALLING": 
                self.change_mode("MARSHALLING")
                self.switch_camera("FRONT")

            elif cmd == "STOP" or cmd == "START_MISSION": 
                self.change_mode("IDLE" if cmd == "STOP" else "NAV")
                self.switch_camera(None)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def change_mode(self, new_mode):
        msg = String()
        msg.data = new_mode
        self.mode_pub.publish(msg)

    def switch_camera(self, target_cam):
        if self.active_cam == target_cam: return

        # 1. 기존 카메라 끄기
        if self.cam_process:
            self.get_logger().info("🛑 Stopping current camera...")
            try:
                os.killpg(os.getpgid(self.cam_process.pid), signal.SIGTERM)
            except:
                self.cam_process.terminate()
            
            self.cam_process.wait()
            self.cam_process = None
            self.active_cam = None
            time.sleep(1.0) # 카메라 자원 해제 대기

        # 2. 새 카메라 켜기
        if target_cam == 'REAR':
            self.get_logger().info("🚀 Launching REAR Camera...")
            # stdout=None으로 설정하면 자식 프로세스의 로그가 현재 터미널에 그대로 나옵니다.
            self.cam_process = subprocess.Popen(
                ['ros2', 'launch', 'orin_car', 'rear_cam.launch.py'],
                preexec_fn=os.setsid,
                stdout=None, 
                stderr=None
            )
            self.active_cam = 'REAR'
        
        elif target_cam == 'FRONT':
            self.get_logger().info("🚀 Launching FRONT Camera...")
            self.cam_process = subprocess.Popen(
                ['ros2', 'launch', 'orin_car', 'front_cam.launch.py'],
                preexec_fn=os.setsid,
                stdout=None,
                stderr=None
            )
            self.active_cam = 'FRONT'

def main(args=None):
    rclpy.init(args=args)
    node = MasterControllerDynamic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cam_process:
            try:
                os.killpg(os.getpgid(node.cam_process.pid), signal.SIGTERM)
            except:
                node.cam_process.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
