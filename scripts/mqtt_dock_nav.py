#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import json
import paho.mqtt.client as mqtt
import ssl
import os
import subprocess
import math
import time

# =========================================================
# [설정] MQTT 브로커 및 토픽
# =========================================================
MQTT_BROKER = "autowingcar.o-r.kr"
MQTT_PORT = 8883
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"

# 경로 파일 폴더
PATH_DIR = os.path.expanduser("~/ros_ws/src/orin_car/paths")

class MqttDockNav(Node):
    def __init__(self):
        super().__init__('mqtt_dock_nav')
        
        # Nav2 클라이언트
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # MQTT 클라이언트 (SSL)
        self.client = mqtt.Client(client_id=CAR_ID, protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.client.tls_set_context(context)

        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
            self.get_logger().info(f"✅ IDLE Mode Started. Connected to {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Fail: {e}")

        self.docking_process = None

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("📡 MQTT Connected. Waiting for commands...")
        client.subscribe(TOPIC_CMD)
        self.send_mqtt_log("System IDLE. Ready.")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            self.get_logger().info(f"📩 CMD: {payload}")
            data = json.loads(payload)
            cmd = data.get("cmd")

            # [1] 도킹 시작 명령
            if cmd == "DOCKING_START":
                self.run_docking()

            # [2] 자율주행 시작 명령 (JSON 경로 파일)
            elif cmd == "START_PATH":
                filename = data.get("path")
                if filename:
                    self.run_path_navigation(filename)
                else:
                    self.send_mqtt_log("Error: No path file specified.")
            
            else:
                self.send_mqtt_log(f"Unknown CMD: {cmd}")

        except Exception as e:
            self.get_logger().error(f"Parsing Error: {e}")

    # =========================================================
    # [기능 1] 도킹 실행 (Subprocess)
    # =========================================================
    def run_docking(self):
        # 이미 실행 중이면 무시
        if self.docking_process is not None and self.docking_process.poll() is None:
            self.send_mqtt_log("⚠️ Docking is ALREADY running.")
            return

        self.send_mqtt_log("🚀 Launching Docking Controller...")
        
        try:
            # 이 코드가 실행될 때만 'docking_controller'가 켜집니다.
            self.docking_process = subprocess.Popen(["ros2", "run", "orin_car", "docking_controller"])
            self.send_mqtt_log("✅ Docking Process Started.")
        except Exception as e:
            self.send_mqtt_log(f"❌ Launch Failed: {e}")

    # =========================================================
    # [기능 2] JSON 경로 파일 읽기 및 주행
    # =========================================================
    def run_path_navigation(self, filename):
        filepath = os.path.join(PATH_DIR, filename)
        if not os.path.exists(filepath):
            self.send_mqtt_log(f"❌ File not found: {filepath}")
            return

        self.send_mqtt_log(f"📂 Loading Path: {filename}")
        
        try:
            with open(filepath, 'r') as f:
                path_data = json.load(f)

            # [수정] 보내주신 JSON 구조 ("x": [..], "y": [..]) 처리
            xs = path_data.get("x", [])
            ys = path_data.get("y", [])
            yaws = path_data.get("yaw", [])
            
            total = len(xs)
            self.send_mqtt_log(f"🚗 Start Navigation: {total} waypoints.")

            for i in range(total):
                tx = xs[i]
                ty = ys[i]
                # Yaw가 없으면 0.0으로 처리
                tyaw = yaws[i] if i < len(yaws) else 0.0
                
                self.send_mqtt_log(f"👉 Point [{i+1}/{total}]: x={tx:.2f}, y={ty:.2f}")
                
                success = self.send_goal_to_nav2(tx, ty, tyaw)
                
                if not success:
                    self.send_mqtt_log(f"❌ Navigation Failed at point {i+1}.")
                    break
            
            self.send_mqtt_log("🏁 Mission Completed (All Points Reached).")

        except Exception as e:
            self.send_mqtt_log(f"Error reading path: {e}")

    def send_goal_to_nav2(self, x, y, yaw):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 Not Ready")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # Yaw -> Quaternion (z, w) 변환
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        # STATUS_SUCCEEDED = 4
        return result_future.result().status == 4 

    def send_mqtt_log(self, msg):
        self.get_logger().info(msg)
        # JSON 형태로 로그 전송
        self.client.publish(TOPIC_MONITOR, json.dumps({"status": msg, "timestamp": time.time()}))

def main(args=None):
    rclpy.init(args=args)
    node = MqttDockNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()