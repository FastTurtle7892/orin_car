#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import json
import paho.mqtt.client as mqtt
import threading
import ssl

# ================= [설정] =================
MQTT_BROKER = "autowingcar.o-r.kr" 
MQTT_PORT = 8883
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

class MqttTotalControl(Node):
    def __init__(self):
        super().__init__('mqtt_total_control')
        
        self.get_logger().info("========================================")
        self.get_logger().info("📢 [MQTT 통합 제어기] 통신 본부 가동 📢")
        self.get_logger().info("========================================")
        
        # [핵심] QoS 설정: Reliable + Transient Local (늦게 켜진 노드에게도 메시지 전달)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.create_subscription(String, '/task_completion', self.completion_callback, 10)
        
        # Publisher 생성
        self.mode_pub = self.create_publisher(String, '/system_mode', qos_profile)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', qos_profile) # 경로 전달용
        
        self.current_mode = "IDLE"
        
        # 0.5초마다 모드 방송 (Nav2 과부하 대비)
        self.create_timer(0.5, self.publish_mode_periodic)

        # MQTT 설정
        self.client = mqtt.Client(client_id=f"{CAR_ID}_bridge", protocol=mqtt.MQTTv311)
        
        # SSL 설정
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.client.tls_set_context(context)
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            threading.Thread(target=self.client.loop_forever, daemon=True).start()
            self.get_logger().info(f"✅ MQTT Connected to {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Failed: {e}")

    def completion_callback(self, msg):
        if msg.data == "DOCKING_COMPLETE":
            if self.current_mode != "IDLE":
                self.current_mode = "IDLE"
                self.get_logger().info("✅ 도킹 완료 보고 수신! 상태를 IDLE로 변경합니다.")
        
    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD)
        self.get_logger().info(f"📡 Listening to {TOPIC_CMD}")

    def publish_mode_periodic(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            cmd = data.get("cmd")
            
            self.get_logger().info(f"📩 MQTT Received: {data}")

            # 1. 도킹 명령
            if cmd == "DOCKING_START":
                self.current_mode = "DOCKING"
                self.get_logger().info("🔄 Mode Set -> DOCKING")

            # 2. 정지 명령
            elif cmd == "STOP":
                self.current_mode = "IDLE"
                self.get_logger().info("🔄 Mode Set -> IDLE")

            # [추가됨] 3. 마샬러(수신호) 주행 명령
            elif cmd == "MARSHALLER_START":
                self.current_mode = "MARSHAL"
                self.get_logger().info("🔄 Mode Set -> MARSHAL (Front Camera ON)")

            # 4. 경로 주행 명령
            elif cmd == "START_PATH":
                # JSON에서 경로 파일명 추출 ("path", "path_file", "path_files" 다 지원)
                path_input = data.get("path") or data.get("path_file") or data.get("path_files")
                
                if path_input:
                    self.current_mode = "DRIVING"
                    self.get_logger().info(f"🔄 Mode Set -> DRIVING | Path: {path_input}")
                    
                    # [중요] 경로 데이터를 ROS2 토픽으로 변환해서 쏨
                    path_msg = String()
                    path_msg.data = json.dumps(path_input) # 리스트나 문자열을 JSON 문자열로 변환
                    self.path_pub.publish(path_msg)
                else:
                    self.get_logger().warn("⚠️ START_PATH received but no path data found.")
                
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