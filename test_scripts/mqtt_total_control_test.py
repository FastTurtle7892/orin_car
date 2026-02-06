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
CAR_CODE = "TC01"

# 토픽 설정
TOPIC_CMD_CONTROL = f"autowing_car/v1/{CAR_CODE}/cmd/control"
TOPIC_CMD_DRIVE   = f"autowing_car/v1/{CAR_CODE}/cmd/drive"
TOPIC_MONITORING = "autowing_car/v1/monitoring"
TOPIC_ACK        = "autowing_car/v1/ack"

MAP_DATA_PATH = os.path.expanduser("~/map_data.json")

# 출발지(Home) 좌표
HOME_X = -0.8893
HOME_Y = 2.5
HOME_THRESHOLD = 0.5  

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
        self.get_logger().info(f"📢 [MQTT] 모든 명령 Fail-Safe (IDLE Reset) 적용")
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
        
        self.current_mode = "IDLE"
        self.monitor_mode = "IDLE"
        self.last_published_mode = None 
        
        self.current_pose = None
        self.battery_level = 85
        self.current_velocity = 0.0
        
        self.pending_final_action = "NONE"
        self.paused_context = None 
        self.latest_drive_path = []

        self.create_timer(0.5, self.publish_mode_periodic)
        self.create_timer(1.0, self.publish_monitor_status) 

        # MQTT Client
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
        client.subscribe(TOPIC_CMD_CONTROL)
        client.subscribe(TOPIC_CMD_DRIVE)
        self.get_logger().info(f"📡 Subscribed: {TOPIC_CMD_CONTROL}, {TOPIC_CMD_DRIVE}")
        self.send_ack("CONNECT", "SUCCESS")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_velocity = 0.0 

    def completion_callback(self, msg):
        if self.monitor_mode == "STOP": return

        data = msg.data
        self.get_logger().info(f"✅ Task Completed: {data}")
        
        if data == "DOCKING_COMPLETE":
            self.current_mode = "IDLE"
            self.monitor_mode = "TOWING"
            self.send_ack("CONNECT", "SUCCESS")
            self.publish_monitor_status()

        # ✅ 그리퍼 해제 -> 마샬러 모드로 전환
        elif data == "RELEASE_COMPLETE":
            self.get_logger().info("🔓 그리퍼 해제 완료 -> 👮 마샬러 모드 자동 진입")
            self.current_mode = "MARSHAL"
            self.monitor_mode = "MARSHALING"
            self.send_ack("DISCONNECT", "SUCCESS")
            self.publish_monitor_status()

        elif data == "DRIVING_COMPLETE":
            self.get_logger().info(f"🏁 Driving Finished. Final Action: {self.pending_final_action}")
            
            if self.pending_final_action in ["DOCK", "CONNECT"]:
                self.current_mode = "DOCKING"
                self.monitor_mode = "DOCKING"
            elif self.pending_final_action in ["UNDOCK", "DISCONNECT"]:
                self.current_mode = "RELEASE"
                self.monitor_mode = "UNDOCKING"
            elif self.pending_final_action == "PARK":
                self.current_mode = "PARK" 
                self.monitor_mode = "RETURNING"
            elif self.pending_final_action == "MARSHAL":
                self.current_mode = "MARSHAL"
                self.monitor_mode = "MARSHALING"
            else:
                self.current_mode = "IDLE"
                self.monitor_mode = "IDLE"
            
            self.pending_final_action = "NONE"
            self.latest_drive_path = []

    def publish_mode_periodic(self):
        if self.current_mode != self.last_published_mode:
            self.get_logger().info(f"📢 System Mode Changed: {self.last_published_mode} -> {self.current_mode}")
            msg = String()
            msg.data = self.current_mode
            self.mode_pub.publish(msg)
            self.last_published_mode = self.current_mode

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

        if self.monitor_mode == "RETURNING":
            dist_to_home = math.sqrt((x - HOME_X)**2 + (y - HOME_Y)**2)
            if dist_to_home < HOME_THRESHOLD:
                self.monitor_mode = "IDLE"
                self.current_mode = "IDLE"
                self.get_logger().info(f"🏠 Arrived Home -> IDLE")

        payload = {
            "car_code": CAR_CODE,
            "x": round(x, 2),
            "y": round(y, 2),
            "yaw": round(yaw, 2),
            "v": self.current_velocity,
            "mode": self.monitor_mode,
            "battery": self.battery_level
        }

        try:
            self.client.publish(TOPIC_MONITORING, json.dumps(payload))
        except Exception as e:
            self.get_logger().error(f"Publish Error: {e}")

    def send_ack(self, cmd, status):
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

            if topic == TOPIC_CMD_CONTROL:
                cmd = data.get("cmd")
                
                # 1. EMERGENCY_STOP
                if cmd == "EMERGENCY_STOP":
                    if self.monitor_mode != "STOP":
                        self.get_logger().warn("🚨 EMERGENCY STOP RECEIVED")
                        self.paused_context = {
                            "internal_mode": self.current_mode,
                            "monitor_mode": self.monitor_mode,
                            "final_action": self.pending_final_action,
                            "drive_path": self.latest_drive_path
                        }
                        self.path_pub.publish(String(data="[]"))
                        self.current_mode = "IDLE"
                        self.monitor_mode = "STOP"
                        self.send_ack(cmd, "SUCCESS_PAUSED")
                    else:
                        self.send_ack(cmd, "ALREADY_STOPPED")

                # 2. RESUME (재개)
                elif cmd == "RESUME":
                    if self.monitor_mode == "STOP":
                        if self.paused_context:
                            self.get_logger().info("▶️ RESUME Command Received")
                            ctx = self.paused_context
                            self.current_mode = ctx["internal_mode"]
                            self.monitor_mode = ctx["monitor_mode"]
                            self.pending_final_action = ctx["final_action"]
                            saved_path = ctx["drive_path"]

                            if self.current_mode == "DRIVING" and saved_path:
                                self.path_pub.publish(String(data=json.dumps(saved_path)))
                            
                            self.paused_context = None
                            self.send_ack(cmd, "SUCCESS_RESUMED")
                        else:
                            # ✅ [핵심 수정] 복구할 정보가 없으면 IDLE로 초기화 (STOP 탈출)
                            self.get_logger().warn("⚠️ RESUME 실패 (저장된 상태 없음) -> IDLE로 초기화")
                            self.current_mode = "IDLE"
                            self.monitor_mode = "IDLE"
                            self.send_ack(cmd, "RESET_TO_IDLE")
                    else:
                        self.send_ack(cmd, "NOT_IN_STOP_MODE")

            elif topic == TOPIC_CMD_DRIVE:
                # 마샬러 모드일 때 차단
                if self.monitor_mode == "MARSHALING" or self.current_mode == "MARSHAL":
                    self.get_logger().warn("🛡️ 마샬러 모드 실행 중! 서버의 주행 명령을 무시합니다.")
                    self.send_ack("DRIVE", "IGNORED_IN_MARSHAL_MODE")
                    return

                if self.monitor_mode == "STOP":
                    self.send_ack("DRIVE", "FAILED_IN_STOP_MODE")
                    return

                msg_type = data.get("type")
                if msg_type == "DRIVE":
                    drive_data = data.get("data", {})
                    edge_ids = drive_data.get("edgeIds", [])
                    final_action = drive_data.get("finalAction", "NONE")
                    
                    full_path = self.convert_edges_to_waypoints(edge_ids)
                    
                    if full_path:
                        self.pending_final_action = final_action
                        self.current_mode = "DRIVING"
                        self.latest_drive_path = full_path
                        
                        if final_action in ["DOCK", "CONNECT"]: self.monitor_mode = "MOVING_TO_GATE"
                        elif final_action in ["UNDOCK", "DISCONNECT"]: self.monitor_mode = "TOWING"
                        elif final_action in ["PARK"]: self.monitor_mode = "RETURNING"
                        else: self.monitor_mode = "MOVING_TO_GATE"

                        self.path_pub.publish(String(data=json.dumps(full_path)))
                        self.send_ack("DRIVE", "SUCCESS")
                    else:
                        # ✅ [핵심 수정] 주행 실패 시에도 IDLE로 초기화
                        self.get_logger().warn("⚠️ 경로 생성 실패 -> IDLE로 초기화")
                        self.current_mode = "IDLE"
                        self.monitor_mode = "IDLE"
                        self.pending_final_action = "NONE"
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
        return waypoints

def main(args=None):
    rclpy.init(args=args)
    node = MqttTotalControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()