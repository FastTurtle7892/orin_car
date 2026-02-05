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
import os
from datetime import datetime

# ================= [설정] =================
MQTT_BROKER = "autowingcar.o-r.kr" 
MQTT_PORT = 8883
CAR_CODE = "TC01"  # 가이드의 car_code (기존 carId)

# [가이드 v1.1] 토픽 설정
# 1. 수신 토픽 (Server -> Edge)
TOPIC_CMD_CONTROL = f"autowing_car/v1/{CAR_CODE}/cmd/control"
TOPIC_CMD_DRIVE   = f"autowing_car/v1/{CAR_CODE}/cmd/drive"

# 2. 송신 토픽 (Edge -> Server)
TOPIC_MONITORING = "autowing_car/v1/monitoring"
TOPIC_ACK        = "autowing_car/v1/ack"

# 3. 맵 데이터 경로
MAP_DATA_PATH = os.path.expanduser("~/map_data.json")

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
        
        self.get_logger().info("============================================")
        self.get_logger().info(f"📢 [MQTT] v1.1 Protocol Applied ({CAR_CODE})")
        self.get_logger().info("============================================")
        
        self.map_data = {}
        self.load_map_data()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # ROS 통신
        self.create_subscription(String, '/task_completion', self.completion_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        
        self.mode_pub = self.create_publisher(String, '/system_mode', qos_profile)
        self.path_pub = self.create_publisher(String, '/driving/path_cmd', qos_profile)
        
        # 내부 상태 변수
        self.current_mode = "IDLE"  # 내부 로직용 모드
        self.monitor_mode = "IDLE"  # 서버 전송용 모드 (가이드 준수)
        
        self.current_pose = None
        self.battery_level = 85
        self.current_velocity = 0.0
        
        self.pending_final_action = "NONE"
        self.driving_purpose = "MOVING_TO_IDLE" # 주행 목적 (모니터링 상태 표시용)

        self.create_timer(0.5, self.publish_mode_periodic)
        self.create_timer(1.0, self.publish_monitor_status)

        # MQTT Client 설정
        self.client = mqtt.Client(client_id=f"{CAR_CODE}_edge", protocol=mqtt.MQTTv311)
        
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        self.client.tls_set_context(context)
        
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            threading.Thread(target=self.client.loop_forever, daemon=True).start()
            self.get_logger().info(f"✅ Connected to Broker: {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"❌ Connection Failed: {e}")

    def load_map_data(self):
        if os.path.exists(MAP_DATA_PATH):
            try:
                with open(MAP_DATA_PATH, 'r', encoding='utf-8') as f:
                    self.map_data = json.load(f)
                self.get_logger().info(f"🗺️ Map Loaded: {len(self.map_data)} edges")
            except Exception as e:
                self.get_logger().error(f"❌ Map Load Error: {e}")
        else:
            self.get_logger().warn(f"⚠️ Map file missing: {MAP_DATA_PATH}")

    def on_connect(self, client, userdata, flags, rc):
        # [가이드 v1.1] Control과 Drive 토픽 구독
        client.subscribe(TOPIC_CMD_CONTROL)
        client.subscribe(TOPIC_CMD_DRIVE)
        self.get_logger().info(f"📡 Subscribed: {TOPIC_CMD_CONTROL}, {TOPIC_CMD_DRIVE}")
        
        # 연결 성공 ACK 전송
        self.send_ack("CONNECT", "SUCCESS")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        # 속도는 추정치나 별도 오도메트리 토픽에서 가져올 수 있음 (현재는 더미)
        self.current_velocity = 0.0 

    def completion_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"✅ Task Completed: {data}")
        
        if data == "DOCKING_COMPLETE":
            # 내부: IDLE, 외부: TOWING(연결된 상태)
            self.current_mode = "IDLE"
            self.monitor_mode = "TOWING" 
            self.send_ack("CONNECT", "SUCCESS") # 도킹 완료 = CONNECT 성공 간주

        elif data == "RELEASE_COMPLETE":
            self.current_mode = "IDLE"
            self.monitor_mode = "IDLE"
            self.send_ack("DISCONNECT", "SUCCESS")

        elif data == "DRIVING_COMPLETE":
            self.get_logger().info(f"🏁 Driving Finished. Final Action: {self.pending_final_action}")
            
            if self.pending_final_action == "DOCK" or self.pending_final_action == "CONNECT":
                self.current_mode = "DOCKING"
                self.monitor_mode = "LOADING" # 가이드: Connecting to aircraft
                
            elif self.pending_final_action == "UNDOCK" or self.pending_final_action == "DISCONNECT":
                self.current_mode = "RELEASE"
                self.monitor_mode = "UNLOADING" # 가이드: Disconnecting
                
            else:
                self.current_mode = "IDLE"
                self.monitor_mode = "IDLE"
            
            self.pending_final_action = "NONE"

    def publish_mode_periodic(self):
        # ROS 시스템 내부 모드 전파
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    # ----------------------------------------------------
    # [가이드 v1.1] Telemetry Monitoring 구현
    # ----------------------------------------------------
    def publish_monitor_status(self):
        if not self.client.is_connected(): return

        x, y, yaw = 0.0, 0.0, 0.0
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            yaw = euler_from_quaternion(
                self.current_pose.orientation.x, self.current_pose.orientation.y,
                self.current_pose.orientation.z, self.current_pose.orientation.w
            )

        # 현재 상태 결정 (가이드 테이블 준수)
        # 내부 모드(DRIVING)일 경우, 목적에 따라 상태 세분화
        status_code = self.monitor_mode
        if self.current_mode == "DRIVING":
            status_code = self.driving_purpose  # MOVING_TO_LOAD or MOVING_TO_IDLE

        payload = {
            "car_code": CAR_CODE,
            "x": round(x, 2),
            "y": round(y, 2),
            "yaw": round(yaw, 2),
            "v": self.current_velocity,
            "mode": status_code,  # IDLE, MOVING_TO_LOAD, LOADING, TOWING, UNLOADING, ...
            "battery": self.battery_level
        }

        try:
            self.client.publish(TOPIC_MONITORING, json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Publish Error: {e}")

    def send_ack(self, cmd, status):
        """서버 명령에 대한 응답(ACK) 전송"""
        payload = {
            "car_code": CAR_CODE,
            "cmd": cmd,
            "status": status,
            "timestamp": datetime.utcnow().strftime('%Y-%m-%dT%H:%M:%SZ')
        }
        self.client.publish(TOPIC_ACK, json.dumps(payload))

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload_str = msg.payload.decode()
            data = json.loads(payload_str)
            
            self.get_logger().info(f"📩 Recv [{topic}]: {data}")

            # =========================================================
            # [CASE 1] High-Level Control (autowing_car/v1/{code}/cmd/control)
            # =========================================================
            if topic == TOPIC_CMD_CONTROL:
                cmd = data.get("cmd")
                
                if cmd == "CONNECT": # Docking Request
                    self.current_mode = "DOCKING"
                    self.monitor_mode = "LOADING"
                    self.get_logger().info("🕹️ CMD: CONNECT -> Start Docking")
                    
                elif cmd == "DISCONNECT": # Release Request
                    self.current_mode = "RELEASE"
                    self.monitor_mode = "UNLOADING"
                    self.get_logger().info("🕹️ CMD: DISCONNECT -> Start Release")
                    
                elif cmd == "EMERGENCY_STOP":
                    self.current_mode = "IDLE"
                    self.monitor_mode = "STOP"
                    self.get_logger().warn("🚨 CMD: EMERGENCY_STOP")
                    
                elif cmd == "SET_MODE":
                    pass # 필요 시 구현
                
                # 명령 수신 ACK 즉시 전송 (동작 시작 알림)
                self.send_ack(cmd, "PENDING")

            # =========================================================
            # [CASE 2] Autonomous Drive (autowing_car/v1/{code}/cmd/drive)
            # =========================================================
            elif topic == TOPIC_CMD_DRIVE:
                # payload 예시:
                # {"data":{"finalAction":"UNDOCK", "edgeIds":["E1","E2"]...}, "type":"DRIVE", ...}
                
                msg_type = data.get("type")
                if msg_type == "DRIVE":
                    drive_data = data.get("data", {})
                    edge_ids = drive_data.get("edgeIds", [])
                    final_action = drive_data.get("finalAction", "NONE")
                    
                    self.get_logger().info(f"🚗 CMD: DRIVE (Action={final_action}, Edges={len(edge_ids)})")

                    full_path = self.convert_edges_to_waypoints(edge_ids)
                    
                    if full_path:
                        self.pending_final_action = final_action
                        self.current_mode = "DRIVING"
                        
                        # 목적에 따른 모니터링 상태 설정
                        if final_action in ["DOCK", "CONNECT"]:
                            self.driving_purpose = "MOVING_TO_LOAD"
                        elif final_action in ["UNDOCK", "DISCONNECT", "PARK"]:
                            self.driving_purpose = "MOVING_TO_IDLE"
                        else:
                            self.driving_purpose = "MOVING_TO_LOAD" # 기본값

                        # 주행 컨트롤러로 경로 전송
                        path_msg = String()
                        path_msg.data = json.dumps(full_path)
                        self.path_pub.publish(path_msg)
                        
                        self.send_ack("DRIVE", "SUCCESS")
                    else:
                        self.get_logger().warn("⚠️ Path conversion failed")
                        self.send_ack("DRIVE", "FAILED_NO_PATH")

            self.publish_monitor_status()
                
        except Exception as e:
            self.get_logger().error(f"Parsing Error: {e}")

    def convert_edges_to_waypoints(self, edge_ids):
        waypoints = []
        for edge_id in edge_ids:
            if edge_id in self.map_data:
                points = self.map_data[edge_id]
                if isinstance(points, list):
                    waypoints.extend(points)
            else:
                self.get_logger().warn(f"❌ Unknown Edge ID: {edge_id}")
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = MqttTotalControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()