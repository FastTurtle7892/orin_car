#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import threading
from std_msgs.msg import String

MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

class MqttFinal(Node):
    def __init__(self):
        super().__init__('mqtt_final')
        
        # 1. 시스템 모드 알림 (방송)
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        # 2. 주행 경로 데이터 전달 (운전기사 전용 우편함)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', 10)
        
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
            self.mqtt_thread.daemon = True
            self.mqtt_thread.start()
        except Exception as e:
            self.get_logger().error(f"MQTT Error: {e}")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT. Subscribing to {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd = payload.get("cmd")
            self.get_logger().info(f"📩 MQTT Cmd: {cmd}")

            mode_msg = String()
            
            # [시나리오 1] 주행 시작 (데이터 포함)
            if cmd == "START_PATH":
                # 1. 모드 변경 방송
                mode_msg.data = "DRIVING"
                self.mode_pub.publish(mode_msg)
                
                # 2. 경로 데이터 전달 (리스트를 JSON 문자열로 변환하여 전송)
                path_data = payload.get("path_files") or payload.get("path_file")
                if path_data:
                    path_msg = String()
                    path_msg.data = json.dumps(path_data) # ['path1.json', 'path2.json'] -> String
                    self.path_pub.publish(path_msg)
            
            # [시나리오 2] 도킹 시작
            elif cmd == "DOCKING_START":
                mode_msg.data = "DOCKING"
                self.mode_pub.publish(mode_msg)
                
            # [시나리오 3] 마샬러 시작
            elif cmd == "MARSHALLER_START":
                mode_msg.data = "MARSHALLER"
                self.mode_pub.publish(mode_msg)
                
            # [시나리오 4] 정지
            elif cmd == "STOP":
                mode_msg.data = "IDLE"
                self.mode_pub.publish(mode_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to parse MQTT: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttFinal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()