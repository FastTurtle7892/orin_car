#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import paho.mqtt.client as mqtt
import threading
import ssl
import time
import math

# ================= [설정] =================
MQTT_BROKER = "autowingcar.o-r.kr" 
MQTT_PORT = 8883
CAR_ID = "car01"

# 1. 수신 토픽 (서버 -> 차량)
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
# 2. 송신 토픽 (차량 -> 서버)
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitor"

# 쿼터니언 -> 오일러각(Yaw) 변환 함수
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return yaw_z

class MqttTotalControl(Node):
    def __init__(self):
        super().__init__('mqtt_total_control')
        
        self.get_logger().info("========================================")
        self.get_logger().info("📢 [MQTT 통합 제어기] 모니터링 강화 버전 📢")
        self.get_logger().info("========================================")
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # ROS -> MQTT로 보낼 정보를 수집하기 위한 구독
        self.create_subscription(String, '/task_completion', self.completion_callback, 10)
        
        # ✅ [추가] AMCL 위치 정보 구독 (실시간 좌표 추적)
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        
        # ROS Publisher
        self.mode_pub = self.create_publisher(String, '/system_mode', qos_profile)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', qos_profile)
        
        # 상태 변수
        self.current_mode = "IDLE"
        self.current_pose = None  # (x, y, heading) 저장용
        self.battery_level = 100  # 배터리는 현재 더미값 (추후 연동 가능)
        
        # 타이머 설정
        self.create_timer(0.5, self.publish_mode_periodic) # 내부용 (0.5초)
        self.create_timer(1.0, self.publish_monitor_status) # 서버 전송용 (1.0초)

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

    # ✅ [추가] 위치 정보 콜백 함수
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def completion_callback(self, msg):
        if msg.data == "DOCKING_COMPLETE":
            if self.current_mode != "IDLE":
                self.current_mode = "IDLE"
                self.get_logger().info("✅ 도킹 완료! 상태를 IDLE로 변경합니다.")
                self.publish_monitor_status()

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD)
        self.get_logger().info(f"📡 Listening to {TOPIC_CMD}")

    def publish_mode_periodic(self):
        """ROS 내부 노드들에게 현재 모드 알림"""
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    # ------------------------------------------------------------------
    # ✅ [업그레이드] 서버로 상세 차량 상태 전송
    # ------------------------------------------------------------------
    def publish_monitor_status(self):
        if not self.client.is_connected():
            return

        # 좌표 및 방향 계산
        x, y, heading_deg = 0.0, 0.0, 0.0
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            
            qx = self.current_pose.orientation.x
            qy = self.current_pose.orientation.y
            qz = self.current_pose.orientation.z
            qw = self.current_pose.orientation.w
            
            yaw_rad = euler_from_quaternion(qx, qy, qz, qw)
            heading_deg = math.degrees(yaw_rad)

        # 서버 프로토콜에 맞춘 데이터 구성
        status_data = {
            "carId": CAR_ID,              # 차량 ID
            "status": self.current_mode,  # 현재 모드 (IDLE, DOCKING, DRIVING...)
            "x": round(x, 2),             # X 좌표
            "y": round(y, 2),             # Y 좌표
            "heading": round(heading_deg, 2), # 방향 (각도)
            "battery": self.battery_level,    # 배터리 잔량
            "timestamp": int(time.time())     # 타임스탬프
        }

        try:
            payload = json.dumps(status_data)
            self.client.publish(TOPIC_MONITOR, payload)
            # self.get_logger().info(f"📤 Mon: {status_data}") # 필요시 주석 해제
        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish monitor data: {e}")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload)
            cmd = data.get("cmd")
            
            self.get_logger().info(f"📩 MQTT Received: {data}")

            if cmd == "DOCKING_START":
                self.current_mode = "DOCKING"
                self.get_logger().info("🔄 Mode Set -> DOCKING")

            elif cmd == "STOP":
                self.current_mode = "IDLE"
                self.get_logger().info("🔄 Mode Set -> IDLE")

            elif cmd == "MARSHALLER_START":
                self.current_mode = "MARSHAL"
                self.get_logger().info("🔄 Mode Set -> MARSHAL")

            elif cmd == "START_PATH":
                path_input = data.get("path") or data.get("path_file") or data.get("path_files")
                if path_input:
                    self.current_mode = "DRIVING"
                    self.get_logger().info(f"🔄 Mode Set -> DRIVING | Path: {path_input}")
                    
                    path_msg = String()
                    path_msg.data = json.dumps(path_input)
                    self.path_pub.publish(path_msg)
                else:
                    self.get_logger().warn("⚠️ START_PATH received but no path data.")
            
            # 명령 수신 즉시 상태 업데이트 반영
            self.publish_monitor_status()
                
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