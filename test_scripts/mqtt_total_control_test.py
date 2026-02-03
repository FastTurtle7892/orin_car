#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import paho.mqtt.client as mqtt
import threading
import ssl

class MqttTotalControl(Node):
    def __init__(self):
        super().__init__('mqtt_total_control')
        
        # [확인용] 이 로그가 반드시 떠야 합니다!
        self.get_logger().info("========================================")
        self.get_logger().info("📢 [MQTT] 확성기 모드 (0.5초 반복 발송) 📢")
        self.get_logger().info("========================================")
        
        # 기본 QoS (Reliable)
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', 10) # 경로용
        
        self.current_mode = "IDLE"
        
        # [핵심] 0.5초마다 무조건 상태를 방송 (Nav2가 시끄러워도 뚫고 지나감)
        self.create_timer(0.5, self.publish_mode_periodic)

        # MQTT 설정
        self.client = mqtt.Client(client_id="car01", protocol=mqtt.MQTTv311)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.client.tls_set_context(context)
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect("autowingcar.o-r.kr", 8883, 60)
            threading.Thread(target=self.client.loop_forever, daemon=True).start()
            self.get_logger().info("✅ MQTT Connected")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Failed: {e}")

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe("autowing_car/v1/car01/cmd")
        self.get_logger().info("📡 Listening for Commands...")

    def publish_mode_periodic(self):
        # 현재 상태를 계속 ROS2 토픽으로 쏩니다.
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            cmd = data.get("cmd")
            self.get_logger().info(f"📩 CMD Received: {cmd}")

            if cmd == "DOCKING_START":
                self.current_mode = "DOCKING"
                self.get_logger().info("🔄 Mode Set -> DOCKING")
            elif cmd == "STOP":
                self.current_mode = "IDLE"
                self.get_logger().info("🔄 Mode Set -> IDLE")
            # [추가] 경로 주행 명령
            elif cmd == "START_PATH":
                self.current_mode = "DRIVING"
                self.get_logger().info("🔄 Mode Set -> DRIVING")
                # 경로 데이터가 있다면 별도 처리 가능
                
        except Exception as e:
            self.get_logger().error(f"Parsing Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttTotalControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()