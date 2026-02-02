#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import json
import math
import paho.mqtt.client as mqtt
import threading
import os
import time

# ==========================================
# 설정 정보
# ==========================================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
PATH_FOLDER = os.path.expanduser("~/trailer_paths")  # JSON 파일들이 저장될 폴더

class MqttPathFollower(Node):
    def __init__(self):
        super().__init__('mqtt_path_follower')

        # [추가됨] 초기 위치 파라미터 선언 (기본값: ORIGIN_GOAL과 동일)
        self.declare_parameter('init_x', -1.111)
        self.declare_parameter('init_y', 0.201)
        self.declare_parameter('init_yaw', -1.57)

        # [추가됨] 초기 위치 퍼블리셔 (AMCL 깨우기용)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        
        # 1. Nav2 FollowPath Action Client
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        
        # [수정됨] Nav2 서버 대기 방식을 'Blocking'에서 'Timer'로 변경
        # (기존 wait_for_server()는 초기화 전에는 영원히 멈추므로 제거함)
        self.is_nav2_ready = False
        self.nav2_check_timer = self.create_timer(2.0, self.check_nav2_server_ready)
        self.get_logger().info("⏳ Waiting for Nav2 Controller Server (Non-blocking)...")

        # [추가됨] 10초 뒤에 자동으로 초기 위치를 쏘는 타이머
        self.get_logger().info("⏳ 10초 뒤에 초기 위치를 자동으로 설정합니다...")
        self.timer_init = self.create_timer(10.0, self.set_initial_pose_once)

        self.path_queue = []
        self.is_moving = False

        # 2. MQTT Client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")
        
        self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

    def check_nav2_server_ready(self):
        """ 주기적으로 Nav2 서버가 준비되었는지 확인 """
        if self._action_client.server_is_ready():
            self.get_logger().info("✅ Nav2 FollowPath Client Ready!")
            self.is_nav2_ready = True
            self.nav2_check_timer.cancel() # 준비되면 타이머 종료

    def set_initial_pose_once(self):
        """ 10초 뒤 실행되어 초기 위치를 퍼블리시함 """
        self.timer_init.cancel()
        
        # 파라미터 값 가져오기
        init_x = self.get_parameter('init_x').value
        init_y = self.get_parameter('init_y').value
        init_yaw = self.get_parameter('init_yaw').value
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.pose.position.x = float(init_x)
        pose_msg.pose.pose.position.y = float(init_y)
        pose_msg.pose.pose.position.z = 0.0
        
        # Yaw -> Quaternion 변환
        pose_msg.pose.pose.orientation.z = math.sin(init_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(init_yaw / 2.0)
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 Auto Initial Pose Set: ({init_x}, {init_y})")

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Subscribed: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
	try:
	    payload = json.loads(msg.payload.decode("utf-8"))
	    cmd = payload.get("cmd")
	    
	    # [수정] 리스트(path_files) 또는 단일 문자열(path_file) 모두 지원하도록 변경
	    path_input = payload.get("path_files") or payload.get("path_file")

	    if cmd == "START_PATH" and path_input:
	        if not self.is_nav2_ready:
	            self.get_logger().warn("⛔ Nav2 Not Ready.")
	            return
	        
	        # [수정] 기존 큐 초기화 후 새 경로 추가
	        self.path_queue = []
	        
	        # [수정] 입력이 리스트인지 확인하여 큐에 추가
	        if isinstance(path_input, list):
	            self.path_queue.extend(path_input)
	        else:
	            self.path_queue.append(path_input)
	        
	        self.get_logger().info(f"📥 Received Path Queue: {self.path_queue}")
	        
	        # [추가] 큐 처리 시작 함수 호출
	        self.process_next_path()

	    elif cmd == "STOP":
	        self.path_queue = []
	        self.stop_robot()
	        
	except Exception as e:
	    self.get_logger().error(f"MQTT Error: {e}")


    def process_next_path(self):

	if not self.path_queue:
	    self.get_logger().info("✅ All paths completed!")
	    self.is_moving = False
	    return

	next_file = self.path_queue.pop(0)
	self.get_logger().info(f"▶ Executing Path File: {next_file} (Remaining: {len(self.path_queue)})")
	self.execute_json_path(next_file)

    def execute_json_path(self, filename):
        full_path = os.path.join(PATH_FOLDER, filename)
        if not os.path.exists(full_path):
            self.get_logger().error(f"❌ File not found: {full_path}")
            return

        # JSON 로드 및 파싱
        with open(full_path, 'r') as f:
            data = json.load(f)

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
            
            yaw = float(yaws[i])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            ros_path.poses.append(pose)

        # Nav2에 FollowPath 액션 전송
        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = "FollowPath" 
        goal_msg.goal_checker_id = "general_goal_checker"

        self.get_logger().info(f"🚀 Sending Path ({len(xs)} points) to Nav2...")
        self._action_client.send_goal_async(goal_msg)

    def stop_robot(self):
        self.get_logger().info("Stop command received")

def main(args=None):
    rclpy.init(args=args)
    node = MqttPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
