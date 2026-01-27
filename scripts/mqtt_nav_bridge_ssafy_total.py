#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan 
from action_msgs.msg import GoalStatus

import json
import time
import threading
import math
import paho.mqtt.client as mqtt

# ==========================================
# 1. 설정 정보
# ==========================================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"

TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"
MONITOR_INTERVAL = 1.0

# ==========================================
# 2. 목표 지점 및 전체 경로 순서 설정
# ==========================================
ORIGIN_GOAL = { 'x': -1.111, 'y': 0.201, 'z': 0.0, 'yaw': -1.57 }
NODE_1_COORD = { 'x': -1.111, 'y': -0.707, 'z': 0.0, 'yaw': -1.57 }
NODE_2_COORD = { 'x': -1.111, 'y': -1.615, 'z': 0.0, 'yaw': -1.57 }
NODE_3_COORD = { 'x': 1.15, 'y': -1.615, 'z': 0.0, 'yaw': 1.57 }
NODE_3_WAYPOINT = { 'x': -0.195, 'y': -2.39, 'z': 0.0, 'yaw': 0.0 }

PREVIEW_ROUTE = [
    "ORIGIN_GOAL", 
    "NODE_1_COORD", 
    "NODE_2_COORD", 
    "NODE_3_WAYPOINT", 
    "NODE_3_COORD"
]

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
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class MqttNavBridge(Node):
    def __init__(self):
        super().__init__('mqtt_nav_bridge')
        
        self.declare_parameter('init_x', -1.111)
        self.declare_parameter('init_y', 0.201)
        self.declare_parameter('init_yaw', -1.57)
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.whole_path_pub = self.create_publisher(Path, '/whole_path', 10)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.plan_client = self.create_client(GetPlan, '/planner_server/get_plan')
        
        self.is_nav2_ready = False
        self.nav2_check_timer = self.create_timer(1.0, self.check_nav2_server_ready)
        self.get_logger().info("⏳ Waiting for Nav2 Server...")

        # 초기 위치는 10초 뒤에 쏨 (AMCL 등 초기화 시간 고려)
        self.timer_init = self.create_timer(10.0, self.set_initial_pose_once)
        
        # [수정] 경로 미리보기는 스레드 실행 시점을 조금 더 당겨도 됨 (어차피 안에서 대기함)
        self.timer_preview = self.create_timer(5.0, self.start_preview_thread)

        self.current_pose = None
        self.robot_status = "IDLE" 
        self.battery_level = 100
        self.goal_queue = [] 

        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.get_logger().info(f"Connected to MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

        self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

        self.timer = self.create_timer(MONITOR_INTERVAL, self.publish_periodic_status)

    def check_nav2_server_ready(self):
        if self._action_client.server_is_ready():
            self.is_nav2_ready = True
            self.nav2_check_timer.cancel()

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
        self.get_logger().info(f"✅ Initial Pose Published")

    def start_preview_thread(self):
        self.timer_preview.cancel()
        threading.Thread(target=self.calculate_preview_path).start()

    # [핵심 수정] 서비스가 준비될 때까지 '무한 대기'하도록 변경
    def calculate_preview_path(self):
        self.get_logger().info("⏳ Waiting for Planner Service (/planner_server/get_plan)...")
        
        # 서비스가 나올 때까지 2초마다 확인하며 무한 루프
        while not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("⏳ Planner Service not ready yet... waiting...")

        self.get_logger().info("✅ Planner Service Ready! Starting Preview Calculation...")

        full_path_msg = Path()
        full_path_msg.header.frame_id = 'map'
        full_path_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.get_logger().info(f"🚀 Calculating Total Path: {PREVIEW_ROUTE}")

        for i in range(len(PREVIEW_ROUTE) - 1):
            start_key = PREVIEW_ROUTE[i]
            end_key = PREVIEW_ROUTE[i+1]
            
            start_pos = GOAL_MAP.get(start_key)
            end_pos = GOAL_MAP.get(end_key)
            
            if not start_pos or not end_pos: continue

            req = GetPlan.Request()
            req.start.header.frame_id = 'map'
            req.start.pose.position.x = float(start_pos['x'])
            req.start.pose.position.y = float(start_pos['y'])
            req.start.pose.orientation.w = 1.0 
            
            req.goal.header.frame_id = 'map'
            req.goal.pose.position.x = float(end_pos['x'])
            req.goal.pose.position.y = float(end_pos['y'])
            req.goal.pose.orientation.w = 1.0
            req.tolerance = 0.5

            future = self.plan_client.call_async(req)
            while not future.done():
                time.sleep(0.1)
            
            try:
                result = future.result()
                if result and result.plan.poses:
                    full_path_msg.poses.extend(result.plan.poses)
            except Exception: pass

        if full_path_msg.poses:
            self.whole_path_pub.publish(full_path_msg)
            self.get_logger().info(f"✨ Total Path Visualized! (Points: {len(full_path_msg.poses)})")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode("utf-8")
            data = json.loads(payload_str)
            cmd = data.get("cmd")
            path = data.get("path", []) 

            if cmd == "START_MISSION":
                self.goal_queue = []
                if not path: return
                for name in path:
                    if name in GOAL_MAP:
                        target = GOAL_MAP[name]
                        if isinstance(target, list): self.goal_queue.extend(target)
                        else: self.goal_queue.append(target)
                if self.goal_queue: self.process_next_goal()
            elif cmd == "GO_HOME": pass

        except Exception: pass

    def process_next_goal(self):
        if not self.goal_queue:
            self.robot_status = "ARRIVED"
            return
        next_goal = self.goal_queue.pop(0)
        self.send_goal_to_nav2(next_goal)

    def send_goal_to_nav2(self, target_data):
        if not self.is_nav2_ready: return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_data['x']
        goal_msg.pose.pose.position.y = target_data['y']
        goal_msg.pose.pose.position.z = target_data['z']
        
        if 'yaw' in target_data:
            yaw_rad = target_data['yaw']
            goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
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
            self.goal_queue = []
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.process_next_goal()
        else:
            time.sleep(1.0)
            self.process_next_goal()
            self.robot_status = "IDLE"

    def publish_periodic_status(self):
        x = 0.0; y = 0.0; heading_deg = 0.0
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            qx = self.current_pose.orientation.x
            qy = self.current_pose.orientation.y
            qz = self.current_pose.orientation.z
            qw = self.current_pose.orientation.w
            yaw_rad = euler_from_quaternion(qx, qy, qz, qw)
            heading_deg = math.degrees(yaw_rad)

        monitor_data = {
            "carId": CAR_ID, "status": self.robot_status,
            "x": round(x, 2), "y": round(y, 2), "heading": round(heading_deg, 2),
            "battery": self.battery_level, "timestamp": int(time.time())
        }
        try:
            self.client.publish(TOPIC_MONITOR, json.dumps(monitor_data))
        except Exception: pass

def main(args=None):
    rclpy.init(args=args)
    node = MqttNavBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()