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
        
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', 10)
        
        # 현재 모드 저장 변수
        self.current_mode_str = "IDLE"
        
        # 1초마다 현재 모드 방송 (Heartbeat)
        self.timer = self.create_timer(1.0, self.publish_mode_loop)
        
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

    def publish_mode_loop(self):
        msg = String()
        msg.data = self.current_mode_str
        self.mode_pub.publish(msg)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            
            # [핵심 수정 1] 공백 제거 (.strip())
            # "DOCKING_START " -> "DOCKING_START" 로 변환
            raw_cmd = payload.get("cmd", "")
            cmd = raw_cmd.strip() 
            
            # [핵심 수정 2] 디버깅 로그 강화 (repr 사용)
            # 눈에 안 보이는 엔터키(\n)나 탭(\t)까지 다 보여줌
            self.get_logger().info(f"📩 수신된 원본: {repr(raw_cmd)}")
            self.get_logger().info(f"✂️ 공백 제거후: {repr(cmd)}")

            # 명령 처리
            if cmd == "START_PATH":
                self.current_mode_str = "DRIVING"
                self.get_logger().info("👉 [성공] 모드 변경됨: DRIVING")
                
                path_data = payload.get("path_files") or payload.get("path_file")
                if path_data:
                    path_msg = String()
                    path_msg.data = json.dumps(path_data)
                    self.path_pub.publish(path_msg)
            
            elif cmd == "DOCKING_START":
                self.current_mode_str = "DOCKING"
                self.get_logger().info("👉 [성공] 모드 변경됨: DOCKING")
                
            elif cmd == "MARSHALLER_START":
                self.current_mode_str = "MARSHALLER"
                self.get_logger().info("👉 [성공] 모드 변경됨: MARSHALLER")
                
            elif cmd == "STOP":
                self.current_mode_str = "IDLE"
                self.get_logger().info("👉 [성공] 모드 변경됨: IDLE")
            
            else:
                # [핵심 수정 3] 실패 시 경고 로그
                self.get_logger().warn(f"⚠️ 명령어 불일치! '{cmd}'는 등록된 명령어가 아닙니다.")

            # 변경된 모드 즉시 방송
            self.publish_mode_loop()

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
