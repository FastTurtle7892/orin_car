#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import paho.mqtt.client as mqtt
import json
import math
import threading
import time

# ================= 설정 =================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"

TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

# 좌표 매핑
WAYPOINT_MAP = {
    "E_GATE_TW1": {"x": -0.037, "y": -1.243, "yaw": -1.50},
    "E_TW1_RWY":  {"x": -0.072, "y": -0.744, "yaw": -1.50},
    "HOME":       {"x": -0.024, "y": 0.032,  "yaw": -1.56}
}
# =======================================

class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client_node')
        
        # Nav2 네비게이터 초기화
        self.navigator = BasicNavigator()
        
        # MQTT 설정
        self.mqtt = mqtt.Client(client_id=f"AutoWing_{CAR_ID}_Mission")
        self.mqtt.on_connect = self.on_connect
        self.mqtt.on_message = self.on_message
        
        try:
            self.mqtt.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt.loop_start()
            self.get_logger().info(f"✅ Mission Client Connected: {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Fail: {e}")

    def on_connect(self, client, userdata, flags, rc):
        # [디버그] 연결 확인용 print
        print(f"\n[DEBUG] MQTT Connected to Broker! Result Code: {rc}")
        print(f"[DEBUG] Subscribing to: {TOPIC_CMD}")
        
        self.get_logger().info(f"Subscribed to: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        # [디버그] 메시지 수신 확인용 print (가장 먼저 실행됨)
        raw_msg = msg.payload.decode('utf-8')
        print(f"\n[DEBUG] 🔥 MQTT Message Received! Topic: {msg.topic}")
        print(f"[DEBUG] Raw Payload: {raw_msg}")

        try:
            payload = raw_msg
            data = json.loads(payload)
            cmd = data.get("cmd")
            
            self.get_logger().info(f"Command Received: {cmd}")
            print(f"[DEBUG] Parsed Command: {cmd}") 

            target_name = ""

            # 1. START_MISSION (리스트로 와도 첫 번째 목적지만 이동)
            if cmd == "START_MISSION":
                paths = data.get("path", [])
                print(f"[DEBUG] Path List: {paths}") 

                if paths:
                    target_name = paths[0] # 첫 번째 목적지만 추출

            # 2. MOVE (단일 타겟)
            elif cmd == "MOVE":
                target_name = data.get("target")
                print(f"[DEBUG] Target: {target_name}")

            # 실행 (목표가 유효하면 이동 스레드 시작)
            if target_name:
                self.get_logger().info(f"📩 Move Command: {target_name}")
                threading.Thread(target=self.execute_navigation, args=(target_name,)).start()
            
            # 3. 정지 명령
            elif cmd in ["STOP", "PAUSE"]:
                print("[DEBUG] STOP Command Triggered")
                self.navigator.cancelTask()
                self.get_logger().warn("🛑 Stop Command Received")

        except Exception as e:
            print(f"[ERROR] JSON Parsing Failed: {e}")
            self.get_logger().error(f"Message Parse Error: {e}")

    def execute_navigation(self, target_name):
        """ RViz 2D Goal Pose와 완전히 동일한 기능 (goToPose) """
        
        # 1. Nav2 활성화 대기 (필수)
        if not self.navigator.lifecycleStartup():
             self.get_logger().info("Waiting for Nav2 to become active...")
             self.navigator.waitUntilNav2Active()

        # 2. 좌표 변환
        if target_name not in WAYPOINT_MAP:
            self.get_logger().error(f"❌ Unknown Location: {target_name}")
            print(f"[ERROR] Unknown Location: {target_name}")
            return

        coords = WAYPOINT_MAP[target_name]
        goal_pose = self.create_pose(coords['x'], coords['y'], coords.get('yaw', 0.0))

        # 3. [핵심] RViz에서 화살표 찍는 것과 동일한 함수 호출
        self.get_logger().info(f"🚀 Going to {target_name} (Single Goal)...")
        self.navigator.goToPose(goal_pose)

        # 4. 도착 대기
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)

        # 5. 결과 확인
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("🏁 Arrived at Destination!")
            print("[DEBUG] Navigation Succeeded!")
        else:
            self.get_logger().warn(f"⚠️ Navigation Failed: {result}")
            print(f"[DEBUG] Navigation Failed with code: {result}")

    def create_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        
        # Yaw -> Quaternion
        q = self.euler_to_quaternion(0, 0, yaw)
        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]
        return p

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = MissionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
