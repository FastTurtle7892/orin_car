#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import threading
from std_msgs.msg import String

# [추가] ROS 2 QoS 설정을 위한 라이브러리
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

class MqttFinal(Node):
    def __init__(self):
        super().__init__('mqtt_final')
        
        # [수정 1] QoS 프로필 설정 (Reliable = TCP처럼 도착 보장)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 반드시 도착해야 함
            history=HistoryPolicy.KEEP_LAST,         # 최근 데이터 유지
            depth=10                                 # 큐 크기 10
        )

        # [수정 2] Publisher에 QoS 프로필 적용
        # 1. 시스템 모드 알림
        self.mode_pub = self.create_publisher(String, '/system_mode', qos_profile)
        # 2. 주행 경로 데이터 전달
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', qos_profile)
        
        # MQTT 클라이언트 설정
        try:
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        except AttributeError:
            self.client = mqtt.Client() 

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_disconnect = self.on_disconnect
        
        try:
            self.get_logger().info(f"🔌 Connecting to MQTT Broker {MQTT_BROKER}:{MQTT_PORT}...")
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            
            self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
            self.mqtt_thread.daemon = True
            self.mqtt_thread.start()
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Error: {e}")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f"✅ Connected to MQTT Broker! (Result Code: {rc})")
            # MQTT QoS도 1로 유지
            client.subscribe(TOPIC_CMD, qos=1)
        else:
            self.get_logger().error(f"❌ Failed to connect, return code {rc}")

    def on_subscribe(self, client, userdata, mid, granted_qos):
        self.get_logger().info(f"📡 Subscribed to {TOPIC_CMD} with QoS: {granted_qos[0]}")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn("⚠️ Unexpected disconnection from MQTT Broker!")

    def on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode("utf-8")
            self.get_logger().info(f"📩 Msg: {payload_str}")

            payload = json.loads(payload_str)
            cmd = payload.get("cmd")

            mode_msg = String()
            
            # [시나리오 1] 주행 시작
            if cmd == "START_PATH":
                mode_msg.data = "DRIVING"
                self.mode_pub.publish(mode_msg)
                
                path_data = payload.get("path_files") or payload.get("path_file")
                if path_data:
                    path_msg = String()
                    path_msg.data = json.dumps(path_data)
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
            self.get_logger().error(f"❌ Failed to parse MQTT message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
