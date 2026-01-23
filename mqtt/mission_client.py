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
CAR_ID = "car01"  # 차량 ID (고유값 설정)

# 토픽 정의
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"

# [중요] 노드 이름 <-> 좌표 매핑 (Rviz에서 딴 좌표를 여기에 입력하세요)
# 예: 서버가 "E_GATE_TW1"을 보내면 로봇은 (1.5, 0.5)로 이동합니다.
WAYPOINT_MAP = {
    "E_GATE_TW1": {"x": -0.037, "y": -1.243, "yaw": -1.50},
    "E_TW1_RWY":  {"x": -0.072, "y": -0.744, "yaw": -1.50},
    "HOME":       {"x": -0.024, "y": 0.032, "yaw": -1.56}
    # ... 나머지 노드들도 추가 필요
}
# =======================================

class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client_node')
        
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

            if cmd == "START_MISSION":
                path_names = data.get("path", [])
                self.start_mission(path_names)
            
            elif cmd == "PAUSE":
                self.navigator.cancelTask()
                self.get_logger().warn("⏸ Mission Paused (Task Canceled)")
            
            elif cmd == "RESUME":
                self.get_logger().warn("▶ Resume not fully implemented (Re-send START_MISSION)")
                
        except Exception as e:
            self.get_logger().error(f"Message Parse Error: {e}")

    def start_mission(self, path_names):
        """ 노드 이름 리스트를 받아서 연속 주행 시작 """
        goals = []
        
        for name in path_names:
            if name in WAYPOINT_MAP:
                coords = WAYPOINT_MAP[name]
                pose = self.create_pose(coords['x'], coords['y'], coords.get('yaw', 0.0))
                goals.append(pose)
                self.get_logger().info(f"➕ Added Waypoint: {name} ({coords['x']}, {coords['y']})")
            else:
                self.get_logger().error(f"❌ Unknown Waypoint: {name}")

        if goals:
            # 별도 스레드에서 주행 시작 (블로킹 방지)
            threading.Thread(target=self.execute_waypoints, args=(goals,)).start()

    def execute_waypoints(self, goals):
        self.navigator.followWaypoints(goals)
        
        while not self.navigator.isTaskComplete():
            time.sleep(1.0)
            
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("🏁 Mission Complete!")
        else:
            self.get_logger().warn("⚠️ Mission Canceled or Failed")

    def create_pose(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        
        # Euler -> Quaternion
        q_x, q_y, q_z, q_w = self.euler_to_quaternion(0, 0, yaw)
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
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
