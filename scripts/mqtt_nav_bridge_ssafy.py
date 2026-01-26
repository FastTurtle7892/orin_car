#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus

import json
import time
import threading
import math
import paho.mqtt.client as mqtt

# ==========================================
# 1. 설정 정보 (사용자 환경)
# ==========================================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"

TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"
MONITOR_INTERVAL = 1.0

# ==========================================
# 2. 목표 지점 및 초기 위치 설정
# ==========================================
# [1] 시작 위치 (이 값을 초기 위치로 사용합니다)
ORIGIN_GOAL = { 'x': -1.111, 'y': 0.201, 'z': 0.0, 'yaw': -1.57 }
NODE_1_COORD = { 'x': -0.957, 'y': -0.707, 'z': 0.0, 'yaw': -1.57 }
NODE_2_WAYPOINT_1 = { 'x': -0.915, 'y': -1.76, 'z': 0.0, 'yaw': -0.92 }
NODE_2_WAYPOINT_2 = { 'x': -0.333, 'y': -2.39, 'z': 0.0, 'yaw': 0.0 }
NODE_2_WAYPOINT_3 = { 'x': 0.842, 'y': -1.81, 'z': 0.0, 'yaw': 1.08 }
NODE_2_COORD = { 'x': 1.15, 'y': -0.384,'z': 0.0, 'yaw': 1.57 }

GOAL_MAP = {
    "NODE_1": NODE_1_COORD,
    "NODE_2": [NODE_2_WAYPOINT_1, NODE_2_WAYPOINT_2, NODE_2_WAYPOINT_3, NODE_2_COORD],
 	"ORIGIN": ORIGIN_GOAL
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
        
        # [추가됨] 초기 위치(Initial Pose)를 설정하기 위한 퍼블리셔 생성
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # 1. Nav2 Action Client 설정
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # [중요] 프로그램 시작 시, 자동으로 초기 위치를 쏴줍니다!
        # 약간의 딜레이를 주어 Nav2가 켜진 후 받도록 함
        self.get_logger().info("Setting Initial Pose in 2 seconds...")
        self.timer_init = self.create_timer(2.0, self.set_initial_pose_once)

        # 2. 로봇 상태 및 주행 큐
        self.current_pose = None
        self.robot_status = "IDLE" 
        self.battery_level = 100
        self.goal_queue = [] 

        # AMCL 위치 구독
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )

        # 3. MQTT Client 설정
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.get_logger().info(f"Connected to MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            self.get_logger().error(f"MQTT Connection Failed: {e}")

        # MQTT 쓰레드 시작
        self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

        # 4. 모니터링 타이머
        self.timer = self.create_timer(MONITOR_INTERVAL, self.publish_periodic_status)
        self.get_logger().info(f"Monitoring Active (Interval: {MONITOR_INTERVAL}s)")

    # [추가된 함수] 자동으로 초기 위치를 쏘는 함수
    def set_initial_pose_once(self):
        self.timer_init.cancel() # 한 번만 실행하고 타이머 끔
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # ORIGIN_GOAL 값을 사용
        pose_msg.pose.pose.position.x = ORIGIN_GOAL['x']
        pose_msg.pose.pose.position.y = ORIGIN_GOAL['y']
        pose_msg.pose.pose.position.z = 0.0
        
        # Yaw -> Quaternion 변환
        yaw = ORIGIN_GOAL['yaw']
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"✅ Initial Pose Set to: ({ORIGIN_GOAL['x']}, {ORIGIN_GOAL['y']})")
        
        # Nav2 서버 대기 시작 (이제 준비될 확률이 높음)
        self.wait_for_nav2()

    def wait_for_nav2(self):
        self.get_logger().info("Waiting for Nav2 Action Server...")
        # 5초만 기다려보고 안 되면 로그 띄우고 패스 (모니터링 멈춤 방지)
        if self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().info("Nav2 Action Client Ready!")
        else:
            self.get_logger().warn("Nav2 Server not ready yet. Will retry on next command.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"MQTT Subscribed: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode("utf-8")
            data = json.loads(payload_str)
            cmd = data.get("cmd")
            path = data.get("path", []) 
            
            self.get_logger().info(f"Received MQTT Command: {cmd}, Path: {path}")

            if cmd == "START_MISSION":
                self.goal_queue = []
                
                if not path:
                    self.get_logger().warn("Path is empty!")
                    return

                for name in path:
                    if name in GOAL_MAP:
                        target = GOAL_MAP[name]
                        if isinstance(target, list):
                            self.goal_queue.extend(target)
                        else:
                            self.goal_queue.append(target)
                    else:
                        self.get_logger().warn(f"Unknown Goal: {name}")

                if self.goal_queue:
                    self.process_next_goal()

            elif cmd == "GO_HOME":
                self.goal_queue = [ORIGIN_GOAL]
                self.process_next_goal()

            elif cmd == "PAUSE":
                pass 
                
        except Exception as e:
            self.get_logger().error(f"JSON Parse Error: {e}")

    def process_next_goal(self):
        if not self.goal_queue:
            self.get_logger().info("All goals completed!")
            self.robot_status = "ARRIVED"
            return

        next_goal = self.goal_queue.pop(0)
        self.get_logger().info(f"Next goal... Remaining: {len(self.goal_queue)}")
        self.send_goal_to_nav2(next_goal)

    def send_goal_to_nav2(self, target_data):
        # 서버 준비 상태 짧게 확인
        if not self._action_client.server_is_ready():
             self.get_logger().warn("Nav2 Server Not Ready! Skipping this goal.")
             return

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

        self.get_logger().info(f"Sending Goal: ({target_data['x']}, {target_data['y']})")
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.robot_status = "MOVING"

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.robot_status = "IDLE"
            self.goal_queue = []
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal Reached!')
            self.process_next_goal()
        else:
            self.get_logger().warn(f'Goal Failed: {status}')
            self.robot_status = "IDLE"
            self.goal_queue = []

    def publish_periodic_status(self):
        x = 0.0
        y = 0.0
        heading_deg = 0.0

        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            
            qx = self.current_pose.orientation.x
            qy = self.current_pose.orientation.y
            qz = self.current_pose.orientation.z
            qw = self.current_pose.orientation.w
            
            yaw_rad = euler_from_quaternion(qx, qy, qz, qw)
            heading_deg = math.degrees(yaw_rad)
        
        # 터미널에 현재 상태 출력 (디버깅용)
        # self.get_logger().info(f"Status - x:{x:.2f}, y:{y:.2f}, Battery:{self.battery_level}%")

        monitor_data = {
            "carId": CAR_ID,
            "status": self.robot_status,
            "x": round(x, 2),
            "y": round(y, 2),
            "heading": round(heading_deg, 2),
            "battery": self.battery_level,
            "timestamp": int(time.time())
        }
        
        try:
            json_str = json.dumps(monitor_data)
            self.client.publish(TOPIC_MONITOR, json_str)
        except Exception as e:
            self.get_logger().error(f"Publish Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
