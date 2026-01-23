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
CAR_ID = "car01"  # 차량 ID

# 토픽 정의
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

# [중요] 노드 이름 <-> 좌표 매핑
# (사용자님이 주신 좌표로 설정했습니다)
WAYPOINT_MAP = {
    "E_GATE_TW1": {"x": -0.037, "y": -1.243, "yaw": -1.50},
    "E_TW1_RWY":  {"x": -0.072, "y": -0.744, "yaw": -1.50},
    "HOME":       {"x": -0.024, "y": 0.032,  "yaw": -1.56}
    # 필요시 추가...
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
            self.get_logger().info(f"✅ Mission Client Connected: {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Fail: {e}")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Subscribed to Command Topic: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
            cmd = data.get("cmd")

            self.get_logger().info(f"📩 Command Received: {cmd}")

            # 1. 경로 주행 (여러 지점 순차 이동)
            if cmd == "START_MISSION":
                path_names = data.get("path", [])
                if path_names:
                    threading.Thread(target=self.execute_waypoints, args=(path_names,)).start()
                else:
                    self.get_logger().warn("⚠️ Path is empty!")

            # 2. 단일 지점 이동 (MOVE 명령어 추가됨!)
            elif cmd == "MOVE":
                target_name = data.get("target")
                if target_name:
                    threading.Thread(target=self.execute_waypoints, args=([target_name],)).start()
                else:
                    self.get_logger().warn("⚠️ Target is missing for MOVE command")
            
            # 3. 정지
            elif cmd == "PAUSE" or cmd == "STOP":
                self.navigator.cancelTask()
                self.get_logger().warn("⏸  Mission Paused (Task Canceled)")
                
        except Exception as e:
            self.get_logger().error(f"Message Parse Error: {e}")

    def execute_waypoints(self, path_names):
        """ 실제 Nav2에게 이동 명령을 내리는 함수 """
        
        # === [핵심 수정] Nav2가 준비될 때까지 대기 ===
        # 이 부분이 없으면 로봇이 명령을 받고도 무시합니다.
        if not self.navigator.lifecycleStartup():
             self.get_logger().info("Waiting for Nav2 to become active...")
             self.navigator.waitUntilNav2Active()
        # ==========================================

        goals = []
        for name in path_names:
            if name in WAYPOINT_MAP:
                coords = WAYPOINT_MAP[name]
                pose = self.create_pose(coords['x'], coords['y'], coords.get('yaw', 0.0))
                goals.append(pose)
                self.get_logger().info(f"➕ Added Goal: {name}")
            else:
                self.get_logger().error(f"❌ Unknown Waypoint: {name}")

        if not goals:
            self.get_logger().warn("⚠️ No valid goals found!")
            return

        # [중요] 주행 시작 명령 전송
        self.get_logger().info(f"🚀 Moving to {len(goals)} waypoints...")
        self.navigator.followWaypoints(goals)
        
        # 주행 상태 모니터링 (블로킹)
        while not self.navigator.isTaskComplete():
            # feedback = self.navigator.getFeedback()
            time.sleep(1.0)
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("🏁 Mission Complete!")
        else:
            self.get_logger().warn(f"⚠️ Mission Failed or Canceled: {result}")

    def create_pose(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        
        # Euler -> Quaternion 변환
        q = self.euler_to_quaternion(0, 0, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        return goal_pose

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
