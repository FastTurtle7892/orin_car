#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
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

# [중요] JSON 파일들이 있는 절대 경로 폴더 (홈 디렉토리 기준)
DATA_ROOT_DIR = os.path.expanduser("~/trailer_paths5")

# 토픽 설정
TOPIC_CMD_CONTROL = f"autowing_car/v1/{CAR_CODE}/cmd/control"
TOPIC_CMD_DRIVE   = f"autowing_car/v1/{CAR_CODE}/cmd/drive"
TOPIC_MONITORING = "autowing_car/v1/monitoring"
TOPIC_ACK        = "autowing_car/v1/ack"

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
        self.get_logger().info(f"📢 [MQTT] 하이브리드 매핑 모드 (CMD + Nav2)")
        self.get_logger().info(f"📂 타겟 폴더: {DATA_ROOT_DIR}")
        self.get_logger().info("============================================")
        
        # ✅ [1] 출발지 좌표 파라미터 설정 (요청값 적용)
        self.declare_parameter('init_x', -0.8893)
        self.declare_parameter('init_y', 2.3)
        
        # 파라미터 값 읽어오기 (self 변수에 저장하여 동적 활용)
        self.home_x = self.get_parameter('init_x').value
        self.home_y = self.get_parameter('init_y').value
        self.get_logger().info(f"🏠 홈(출발지) 좌표 설정됨: X={self.home_x}, Y={self.home_y}")
        
        # ✅ [매핑 설정] 요청하신 대로 n4~n7은 하드코딩, n7~n8은 Nav2(파일)로 설정
        self.edge_to_file_map = {
            # 1. 초반 Nav2 구간
            "E_n1_to_n2": "P1-1_origin",
            "E_n2_to_n3": "P2-1_origin",
            "E_n3_to_n4": "P3-1_origin",
            
            # 2. 중간 하드코딩 구간
            "E_n4_to_n5": "CMD_HARD_RIGHT_2S",       # 우회전 30도, 전진 2초
            "E_n5_to_n6": "CMD_HARD_LEFT_BACK_2S",   # 좌회전 30도, 후진 2초
            "E_n6_to_n7": "CMD_HARD_RIGHT_40_3S",    # 우회전 40도, 전진 3초
            
            # 3. 마지막 n8로 가는 구간 (Nav2 사용 요청 반영)
            "E_n7_to_n8": "CMD_HARD_FWD_1S"          # 파일 이름은 기존 규칙에 따름
        }
        self.get_logger().info(f"🗺️ 매핑 로드됨: {self.edge_to_file_map}")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # ROS 통신
        self.create_subscription(String, '/task_completion', self.completion_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10) # [유지] 속도 구독

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
        self.latest_drive_paths = [] 

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

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD_CONTROL)
        client.subscribe(TOPIC_CMD_DRIVE)
        self.get_logger().info(f"📡 Subscribed: {TOPIC_CMD_CONTROL}, {TOPIC_CMD_DRIVE}")
        self.send_ack("CONNECT", "SUCCESS")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        # self.current_velocity = 0.0  <- [수정] cmd_vel_callback에서 갱신하므로 여기서는 0으로 초기화하지 않음 (선택사항이나 원본 유지 차원)
    
    def cmd_vel_callback(self, msg):
        # [유지] DrivingController가 보낸 선속도(linear.x)를 저장
        self.current_velocity = msg.linear.x

    def completion_callback(self, msg):
        if self.monitor_mode == "STOP": return

        data = msg.data
        self.get_logger().info(f"✅ Task Completed: {data}")
        
        if data == "DOCKING_COMPLETE":
            self.current_mode = "IDLE"
            self.monitor_mode = "TOWING"
            self.send_ack("CONNECT", "SUCCESS")
            self.publish_monitor_status()

        elif data == "RELEASE_COMPLETE":
            self.current_mode = "IDLE"
            self.monitor_mode = "WAITING_FOR_RETURN"
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
                # ⭐️ [수정 2] n8 도착 시(명령어 없음) -> 마샬러 모드 자동 전환
                self.get_logger().info("🚀 n8 도착! -> 마샬러 모드(MARSHAL) 자동 진입")
                self.current_mode = "MARSHAL"
                self.monitor_mode = "MARSHALING"
            
            self.pending_final_action = "NONE"
            self.latest_drive_paths = []

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

        # ✅ [수정 3] 마샬링/복귀 모드일 때 출발지 거리 체크 -> IDLE 자동 전환
        # 상수가 아닌 파라미터 변수(self.home_x, self.home_y) 사용
        if self.monitor_mode in ["MARSHALING", "RETURNING"]:
            dist_to_home = math.sqrt((x - self.home_x)**2 + (y - self.home_y)**2)
            
            # 거리 1.0m 이내로 들어오면
            if dist_to_home < 1.0:
                self.get_logger().info(f"🎉 출발지 복귀 완료 (거리: {dist_to_home:.2f}m) -> IDLE 대기 상태로 전환")
                self.monitor_mode = "IDLE"
                self.current_mode = "IDLE"
                # 안전을 위해 주행 경로 초기화 (정지)
                self.path_pub.publish(String(data="[]"))

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
                        self.get_logger().warn("🚨 EMERGENCY STOP")
                        self.paused_context = {
                            "internal_mode": self.current_mode,
                            "monitor_mode": self.monitor_mode,
                            "final_action": self.pending_final_action,
                            "drive_paths": self.latest_drive_paths
                        }
                        self.path_pub.publish(String(data="[]"))
                        self.current_mode = "IDLE"
                        self.monitor_mode = "STOP"
                        self.send_ack(cmd, "SUCCESS_PAUSED")
                    else:
                        self.send_ack(cmd, "ALREADY_STOPPED")

                elif cmd == "RESUME":
                    if self.monitor_mode == "STOP":
                        if self.paused_context:
                            self.get_logger().info("▶️ RESUME")
                            ctx = self.paused_context
                            self.current_mode = ctx["internal_mode"]
                            self.monitor_mode = ctx["monitor_mode"]
                            self.pending_final_action = ctx["final_action"]
                            saved_paths = ctx["drive_paths"]

                            if self.current_mode == "DRIVING" and saved_paths:
                                self.path_pub.publish(String(data=json.dumps(saved_paths)))
                            
                            self.paused_context = None
                            self.send_ack(cmd, "SUCCESS_RESUMED")
                        else:
                            self.current_mode = "IDLE"
                            self.monitor_mode = "IDLE"
                            self.send_ack(cmd, "RESET_TO_IDLE")
                    else:
                        self.send_ack(cmd, "NOT_IN_STOP_MODE")

            elif topic == TOPIC_CMD_DRIVE:
                if self.monitor_mode == "STOP":
                    self.send_ack("DRIVE", "FAILED_IN_STOP_MODE")
                    return

                msg_type = data.get("type")
                if msg_type == "DRIVE":
                    drive_data = data.get("data", {})
                    edge_ids = drive_data.get("edgeIds", [])
                    final_action = drive_data.get("finalAction", "NONE")
                    
                    # ✅ [변환] 엣지 ID -> (절대 경로 or CMD 문자열) 리스트
                    abs_path_list = self.convert_edges_to_absolute_paths(edge_ids)
                    
                    if abs_path_list:
                        self.pending_final_action = final_action
                        self.current_mode = "DRIVING"
                        self.latest_drive_paths = abs_path_list
                        
                        if final_action in ["DOCK", "CONNECT"]: self.monitor_mode = "MOVING_TO_GATE"
                        elif final_action in ["UNDOCK", "DISCONNECT"]: self.monitor_mode = "TOWING"
                        elif final_action in ["PARK"]: self.monitor_mode = "RETURNING"
                        else: self.monitor_mode = "MOVING_TO_GATE"

                        self.get_logger().info(f"📤 경로 전송: {abs_path_list}")
                        self.path_pub.publish(String(data=json.dumps(abs_path_list)))
                        self.send_ack("DRIVE", "SUCCESS")
                    else:
                        self.get_logger().warn("⚠️ 매핑된 파일 없음 -> IDLE")
                        self.current_mode = "IDLE"
                        self.monitor_mode = "IDLE"
                        self.send_ack("DRIVE", "FAILED_MAPPING")

            self.publish_monitor_status()
                
        except Exception as e:
            self.get_logger().error(f"Parsing Error: {e}")

    def convert_edges_to_absolute_paths(self, edge_ids):
        """ 엣지 ID를 받아서, CMD는 그대로, 파일은 절대 경로로 변환 """
        path_list = []
        for edge_id in edge_ids:
            if edge_id in self.edge_to_file_map:
                mapped_val = self.edge_to_file_map[edge_id]
                
                # CMD_ 로 시작하면 파일 변환 없이 그대로 문자열 전달
                if mapped_val.startswith("CMD_"):
                    path_list.append(mapped_val)
                else:
                    filename = mapped_val
                    if not filename.endswith('.json'):
                        filename += '.json'
                    full_path = os.path.join(DATA_ROOT_DIR, filename)
                    path_list.append(full_path)
            else:
                self.get_logger().error(f"❌ 맵핑 안 된 엣지 ID: {edge_id}")
                return None 
        return path_list

def main(args=None):
    rclpy.init(args=args)
    node = MqttTotalControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()