#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

import json
import time
import threading
import math
import paho.mqtt.client as mqtt

# ==========================================
# 1. 설정 정보 (SSAFY 환경)
# ==========================================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"

TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"
MONITOR_INTERVAL = 1.0

# ==========================================
# 2. 목표 지점 데이터
# ==========================================
ORIGIN_GOAL = { 'x': -1.111, 'y': 0.201, 'z': 0.0, 'yaw': -1.57 }
NODE_1_COORD = { 'x': -1.111, 'y': -0.707, 'z': 0.0, 'yaw': -1.57 }
NODE_2_COORD = { 'x': -1.111, 'y': -1.615, 'z': 0.0, 'yaw': -1.57 }
NODE_3_COORD = { 'x': 1.15, 'y': -1.615, 'z': 0.0, 'yaw': 1.57 }
NODE_3_WAYPOINT = { 'x': -0.195, 'y': -2.39, 'z': 0.0, 'yaw': 0.0 }

GOAL_MAP = {
    "NODE_1_COORD": NODE_1_COORD,
    "NODE_2_COORD": NODE_2_COORD,
    "NODE_3_COORD": NODE_3_COORD,
    "NODE_3_WAYPOINT": NODE_3_WAYPOINT,
    "NODE_3_TOTAL": [NODE_3_WAYPOINT, NODE_3_COORD],
    "ORIGIN_GOAL": ORIGIN_GOAL
}

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class MasterController(Node):
    def __init__(self):
        super().__init__('master_controller')
        
        self.declare_parameter('init_x', -1.111)
        self.declare_parameter('init_y', 0.201)
        self.declare_parameter('init_yaw', -1.57)

        # [모드 시스템] 현재 모드 방송 퍼블리셔
        # modes: "IDLE", "NAV", "DOCKING", "MARSHALLING"
        self.mode_pub = self.create_publisher(String, '/system_mode', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 비상 정지용

        self.current_mode = "IDLE"
        self.get_logger().info(f"🚀 Master Controller Started. Current Mode: {self.current_mode}")

        # Nav2 Action Client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.is_nav2_ready = False
        self.nav2_check_timer = self.create_timer(1.0, self.check_nav2_server_ready)

        # 초기 위치 설정 타이머 (10초 후 실행)
        self.timer_init = self.create_timer(10.0, self.set_initial_pose_once)

        # 상태 변수
        self.current_pose = None
        self.robot_status = "IDLE"
        self.battery_level = 100
        self.goal_queue = [] 

        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)

        # MQTT Client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.get_logger().info(f"Connected to MQTT Broker: {MQTT_BROKER}")
            self.client.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

        # 모니터링 및 모드 방송 (1초 주기)
        self.timer = self.create_timer(MONITOR_INTERVAL, self.publish_periodic_status)

    def check_nav2_server_ready(self):
        if self._action_client.server_is_ready():
            self.is_nav2_ready = True
            self.nav2_check_timer.cancel()
            self.get_logger().info("✅ Nav2 Action Server Ready")

    def set_initial_pose_once(self):
        self.timer_init.cancel()
        init_x = self.get_parameter('init_x').value
        init_y = self.get_parameter('init_y').value
        init_yaw = self.get_parameter('init_yaw').value
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = float(init_x)
        pose_msg.pose.pose.position.y = float(init_y)
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(init_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(init_yaw / 2.0)
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"✅ Initial Pose Published: ({init_x}, {init_y})")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Subscribed: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
            cmd = payload.get("cmd")
            path = payload.get("path", [])
            
            self.get_logger().info(f"📩 Command: {cmd}, Path: {path}")

            # [모드 전환 로직]
            if cmd == "START_MISSION": # -> NAV 모드
                self.change_mode("NAV")
                self.start_nav_sequence(path)

            elif cmd == "DOCKING": # -> DOCKING 모드
                self.change_mode("DOCKING")

            elif cmd == "MARSHALLING": # -> MARSHALLING 모드
                self.change_mode("MARSHALLING")

            elif cmd == "STOP": # -> IDLE 모드 (정지)
                self.change_mode("IDLE")
                self.stop_robot()
            
            elif cmd == "GO_HOME":
                self.change_mode("NAV")
                self.goal_queue = [ORIGIN_GOAL]
                self.process_next_goal()

        except Exception as e:
            self.get_logger().error(f"JSON Error: {e}")

    def change_mode(self, new_mode):
        """ 모드를 변경하고 즉시 전파 """
        self.current_mode = new_mode
        self.get_logger().info(f"🔄 Mode Switched to: {self.current_mode}")
        
        # 모드 변경 메시지 즉시 발행
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)

    def start_nav_sequence(self, path_names):
        self.goal_queue = []
        for name in path_names:
            if name in GOAL_MAP:
                target = GOAL_MAP[name]
                if isinstance(target, list): self.goal_queue.extend(target)
                else: self.goal_queue.append(target)
            else:
                self.get_logger().warn(f"Unknown Goal: {name}")

        if self.goal_queue:
            self.process_next_goal()

    def process_next_goal(self):
        # NAV 모드가 아니면 주행 중단
        if self.current_mode != "NAV":
            return

        if not self.goal_queue:
            self.get_logger().info("All goals completed!")
            self.robot_status = "ARRIVED"
            return

        next_goal = self.goal_queue.pop(0)
        self.send_goal_to_nav2(next_goal)

    def send_goal_to_nav2(self, target_data):
        if not self.is_nav2_ready:
             self.get_logger().warn("Nav2 Not Ready")
             return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_data['x']
        goal_msg.pose.pose.position.y = target_data['y']
        
        if 'yaw' in target_data:
            yaw = target_data['yaw']
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal_msg.pose.pose.orientation.z = target_data.get('qz', 0.0)
            goal_msg.pose.pose.orientation.w = target_data.get('qw', 1.0)
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.robot_status = "MOVING"

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.robot_status = "IDLE"
            return
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.process_next_goal()
        else:
            self.robot_status = "IDLE"

    def stop_robot(self):
        # 즉시 정지 명령
        self.cmd_vel_pub.publish(Twist())
        self.goal_queue = [] # 경로 취소

    def publish_periodic_status(self):
        # 1. 모드 지속 방송 (새로 켜진 노드를 위해)
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)

        # 2. 모니터링 정보 전송
        x, y, heading = 0.0, 0.0, 0.0
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            heading = math.degrees(euler_from_quaternion(
                self.current_pose.orientation.x, self.current_pose.orientation.y,
                self.current_pose.orientation.z, self.current_pose.orientation.w))

        monitor_data = {
            "carId": CAR_ID,
            "status": self.robot_status,
            "mode": self.current_mode,
            "x": round(x, 2),
            "y": round(y, 2),
            "heading": round(heading, 2),
            "battery": self.battery_level,
            "timestamp": int(time.time())
        }
        try:
            self.client.publish(TOPIC_MONITOR, json.dumps(monitor_data))
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
