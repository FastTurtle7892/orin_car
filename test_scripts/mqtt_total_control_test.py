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
CAR_CODE = "TC01"  # 가이드의 car_code

# [가이드 v1.1] 토픽 설정
TOPIC_CMD_CONTROL = f"autowing_car/v1/{CAR_CODE}/cmd/control"
TOPIC_CMD_DRIVE   = f"autowing_car/v1/{CAR_CODE}/cmd/drive"
TOPIC_MONITORING = "autowing_car/v1/monitoring"
TOPIC_ACK        = "autowing_car/v1/ack"

MAP_DATA_PATH = os.path.expanduser("~/map_data.json")

# ✅ [수정] 출발지(Home) 좌표 설정 (사용자 요청)
HOME_X = -0.8893
HOME_Y = 2.5
# Y값이 2.3 정도 되면 도착으로 간주 (2.5 - 0.2 = 2.3)
# X좌표 오차까지 고려하여 반경 0.5m 이내면 도착 처리
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
        self.get_logger().info(f"📢 [MQTT] v1.4 Home Check Logic Update ({CAR_CODE})")
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
        self.current_mode = "IDLE"  # 로봇 내부 제어 모드
        self.monitor_mode = "IDLE"  # 서버 모니터링용 상태
        
        self.current_pose = None
        self.battery_level = 85
        self.current_velocity = 0.0
        
        self.pending_final_action = "NONE"
        
        # 작업 저장 컨텍스트 (비상 정지/재개용)
        self.paused_context = None 
        self.latest_drive_path = []

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

        elif data == "RELEASE_COMPLETE":
            self.current_mode = "IDLE"
            self.monitor_mode = "WAITING_FOR_RETURN"
            self.send_ack("DISCONNECT", "SUCCESS")

        elif data == "DRIVING_COMPLETE":
            self.get_logger().info(f"🏁 Driving Finished. Final Action: {self.pending_final_action}")
            
            if self.pending_final_action == "DOCK" or self.pending_final_action == "CONNECT":
                self.current_mode = "DOCKING"
                self.monitor_mode = "DOCKING"
                
            elif self.pending_final_action == "UNDOCK" or self.pending_final_action == "DISCONNECT":
                self.current_mode = "RELEASE"
                self.monitor_mode = "UNDOCKING"
                
            else:
                self.current_mode = "IDLE"
                self.monitor_mode = "IDLE"
            
            self.pending_final_action = "NONE"
            self.latest_drive_path = []

    def publish_mode_periodic(self):
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

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

        # ✅ [수정] 복귀 중(RETURNING)일 때 집 도착 감지 로직
        # "Y값이 2.3이 되면(2.5에 근접하면)" 조건 적용
        if self.monitor_mode == "RETURNING":
            # 1. Y축 기준 검사: 목표(2.5)에 대해 2.3 이상 올라오면 도착으로 간주 (y > 2.3)
            #    혹은 단순 거리 계산 (안전하게 반경 0.5m)
            dist_to_home = math.sqrt((x - HOME_X)**2 + (y - HOME_Y)**2)
            
            # (옵션) 사용자 요청대로 Y축 값만 명시적으로 볼 수도 있음
            # if y >= 2.3: ...
            
            if dist_to_home < HOME_THRESHOLD:
                self.monitor_mode = "IDLE"
                self.current_mode = "IDLE"
                self.get_logger().info(f"🏠 Arrived Home (y={y:.2f}, dist={dist_to_home:.2f}) -> IDLE")

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
                
                if cmd == "EMERGENCY_STOP":
                    if self.monitor_mode != "STOP":
                        self.get_logger().warn("🚨 EMERGENCY STOP RECEIVED")
                        self.paused_context = {
                            "internal_mode": self.current_mode,
                            "monitor_mode": self.monitor_mode,
                            "final_action": self.pending_final_action,
                            "drive_path": self.latest_drive_path
                        }
                        
                        # 정지 명령 (빈 경로)
                        self.path_pub.publish(String(data="[]"))
                        
                        self.current_mode = "IDLE"
                        self.monitor_mode = "STOP"
                        self.send_ack(cmd, "SUCCESS_PAUSED")
                    else:
                        self.send_ack(cmd, "ALREADY_STOPPED")

                elif cmd == "RESUME":
                    if self.monitor_mode == "STOP" and self.paused_context:
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
                        self.send_ack(cmd, "FAILED_NO_CONTEXT")
                else:
                    self.send_ack(cmd, "PENDING")

            elif topic == TOPIC_CMD_DRIVE:
                if self.monitor_mode == "STOP":
                    self.send_ack("DRIVE", "FAILED_IN_STOP_MODE")
                    return

                msg_type = data.get("type")
                if msg_type == "DRIVE":
                    drive_data = data.get("data", {})
                    edge_ids = drive_data.get("edgeIds", [])
                    final_action = drive_data.get("finalAction", "NONE")
                    
                    self.get_logger().info(f"🚗 CMD: DRIVE (Action={final_action})")

                    full_path = self.convert_edges_to_waypoints(edge_ids)
                    
                    if full_path:
                        self.pending_final_action = final_action
                        self.current_mode = "DRIVING"
                        self.latest_drive_path = full_path
                        
                        if final_action in ["DOCK", "CONNECT"]:
                            self.monitor_mode = "MOVING_TO_GATE"
                        elif final_action in ["UNDOCK", "DISCONNECT"]:
                            self.monitor_mode = "TOWING"
                        elif final_action in ["PARK"]:
                            self.monitor_mode = "RETURNING"
                        else:
                            self.monitor_mode = "MOVING_TO_GATE"

                        self.path_pub.publish(String(data=json.dumps(full_path)))
                        self.send_ack("DRIVE", "SUCCESS")
                    else:
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