#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist # ✅ Twist 추가됨
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
        self.get_logger().info("📢 [MQTT 통합 제어기] 도킹 해제(RELEASE) 추가됨 📢")
        self.get_logger().info("========================================")
        
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # ROS -> MQTT로 보낼 정보를 수집하기 위한 구독
        self.create_subscription(String, '/task_completion', self.completion_callback, 10)
        
        # AMCL 위치 정보 구독
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        
        # ROS Publisher
        self.mode_pub = self.create_publisher(String, '/system_mode', qos_profile)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', qos_profile)

        # ✅ [추가] 직접 제어를 위한 퍼블리셔 (그리퍼 및 주행)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 상태 변수
        self.current_mode = "IDLE"
        self.current_pose = None  
        self.battery_level = 100 
        
        # 타이머 설정
        self.create_timer(0.5, self.publish_mode_periodic) 
        self.create_timer(1.0, self.publish_monitor_status) 

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
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def publish_monitor_status(self):
        if not self.client.is_connected():
            return

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

        status_data = {
            "carId": CAR_ID,              
            "status": self.current_mode,  
            "x": round(x, 2),             
            "y": round(y, 2),             
            "heading": round(heading_deg, 2), 
            "battery": self.battery_level,    
            "timestamp": int(time.time())     
        }

        try:
            payload = json.dumps(status_data)
            self.client.publish(TOPIC_MONITOR, payload)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish monitor data: {e}")

    # ✅ [추가] 도킹 해제 및 후진 시퀀스 함수
    def execute_undocking_sequence(self):
        self.get_logger().info("🚀 [도킹 해제] 시퀀스 시작")
        
        # 1. 그리퍼에게 'PLACE' 명령 전송 (Down -> Open -> Up)
        grip_msg = String()
        grip_msg.data = "PLACE"
        self.gripper_pub.publish(grip_msg)
        
        self.get_logger().info("⏳ 물건 내려놓는 중 (5초 대기)...")
        # 서보 움직임이 많으므로 넉넉히 대기
        time.sleep(5.0)
        
        # 2. 후진하여 이탈하기
        self.get_logger().info("🔙 후진 시작 (2초간)")
        twist = Twist() 
        twist.linear.x = -0.3  # 후진 속도 (조절 가능)
        twist.angular.z = 0.0
        
        # 약 2초간 후진 명령 반복 전송
        for _ in range(20): # 0.1s * 20 = 2.0s
            self.cmd_vel_pub.publish(twist) 
            time.sleep(0.1)
            
        # 3. 정지 및 모드 복귀
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        
        self.current_mode = "IDLE"
        self.get_logger().info("✅ 도킹 해제 완료 (IDLE 복귀)")
        self.publish_monitor_status()

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
            
            # ✅ [추가] 도킹 해제 명령 처리
            elif cmd == "DOCKING_RELEASE":
                self.get_logger().info("🔄 Mode Set -> UNDOCKING (Release)")
                self.current_mode = "UNDOCKING"
                # 긴 동작이므로 스레드로 실행 (Main Loop 차단 방지)
                threading.Thread(target=self.execute_undocking_sequence).start()

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