#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import json
import math
import paho.mqtt.client as mqtt
import threading
import os

# ==========================================
# 설정 정보
# ==========================================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
PATH_FOLDER = os.path.expanduser("~/trailer_paths")  # JSON 파일들이 저장될 폴더 (절대경로 권장)

class MqttPathFollower(Node):
    def __init__(self):
        super().__init__('mqtt_path_follower')
        
        # 1. Nav2 FollowPath Action Client
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info("Waiting for Nav2 Controller Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("✅ Nav2 FollowPath Client Ready")

        # 2. MQTT Client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Subscribed: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd = payload.get("cmd")
            path_filename = payload.get("path_file") # 예: "path_Start_to_Goal.json"

            if cmd == "START_PATH" and path_filename:
                self.execute_json_path(path_filename)
            elif cmd == "STOP":
                self.stop_robot()
                
        except Exception as e:
            self.get_logger().error(f"MQTT Error: {e}")

    def execute_json_path(self, filename):
        full_path = os.path.join(PATH_FOLDER, filename)
        if not os.path.exists(full_path):
            self.get_logger().error(f"❌ File not found: {full_path}")
            return

        # JSON 로드 및 파싱
        with open(full_path, 'r') as f:
            data = json.load(f)

        # JSON 포맷 파싱 (PathPlanner의 출력 포맷인 x: [], y: [] 형태 대응)
        xs = data.get("x", [])
        ys = data.get("y", [])
        yaws = data.get("yaw", [0.0]*len(xs))

        if not xs or not ys:
            self.get_logger().warn("⚠️ Empty path data in JSON")
            return

        # Nav2 Path 메시지 생성
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(xs)):
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(xs[i])
            pose.pose.position.y = float(ys[i])
            pose.pose.position.z = 0.0
            
            # Yaw -> Quaternion
            yaw = float(yaws[i])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            ros_path.poses.append(pose)

        # Nav2에 FollowPath 액션 전송
        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = "FollowPath" # Nav2 파라미터에 정의된 컨트롤러 ID
        goal_msg.goal_checker_id = "general_goal_checker"

        self.get_logger().info(f"🚀 Sending Path ({len(xs)} points) to Nav2...")
        self._action_client.send_goal_async(goal_msg)

    def stop_robot(self):
        # 액션 취소 로직 등을 추가할 수 있음
        self.get_logger().info("Stop command received (Logic not implemented)")

def main(args=None):
    rclpy.init(args=args)
    node = MqttPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
