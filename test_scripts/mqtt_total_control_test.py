#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import paho.mqtt.client as mqtt
import threading
import ssl

# ==========================================
# [설정] MQTT 정보 (본인 환경에 맞게 수정)
# ==========================================
MQTT_BROKER = "autowingcar.o-r.kr"
MQTT_PORT = 8883
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_LOG = f"autowing_car/v1/{CAR_ID}/monitoring"

class MqttTotalControl(Node):
    def __init__(self):
        super().__init__('mqtt_total_control')
        
        # 1. ROS2 퍼블리셔
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', 10)

        # 2. MQTT 클라이언트
        self.client = mqtt.Client(client_id=CAR_ID, protocol=mqtt.MQTTv311)
        
        # SSL 설정 (필요 없으면 주석 처리)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.client.tls_set_context(context)
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # 3. 연결 시작
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            threading.Thread(target=self.client.loop_forever, daemon=True).start()
            self.get_logger().info(f"✅ MQTT Connected to {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Failed: {e}")

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD)
        self.get_logger().info(f"📡 Listening on {TOPIC_CMD}")
        self.change_mode("IDLE")  # 시작 시 대기 모드

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            cmd = data.get("cmd")
            self.get_logger().info(f"📩 CMD Received: {cmd}")

            if cmd == "DOCKING_START":
                self.change_mode("DOCKING")
            
            elif cmd == "MARSHALLER_START":
                self.change_mode("MARSHALLER")
            
            elif cmd == "START_PATH":
                # 주행 모드로 변경 후 경로 전달
                self.change_mode("DRIVING")
                path_data = data.get("path") # 문자열 or 리스트
                if path_data:
                    # JSON 그대로 다시 문자열로 말아서 보냄
                    self.path_pub.publish(String(data=json.dumps(path_data)))
            
            elif cmd == "STOP":
                self.change_mode("IDLE")
                
        except Exception as e:
            self.get_logger().error(f"Parsing Error: {e}")

    def change_mode(self, mode):
        self.mode_pub.publish(String(data=mode))
        # 모니터링 토픽으로 상태 전송
        self.client.publish(TOPIC_LOG, json.dumps({"status": f"Mode changed to {mode}"}))
        self.get_logger().info(f"🔄 System Mode -> {mode}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttTotalControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()