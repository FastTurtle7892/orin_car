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
        
        # [수정] 런치 파일에서 초기 위치를 받기 위한 파라미터 선언
        self.declare_parameter('init_x', -1.111)
        self.declare_parameter('init_y', 0.201)
        self.declare_parameter('init_yaw', -1.57)
        
        # 퍼블리셔 생성
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # 1. Nav2 Action Client 설정
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Nav2 서버 상태 확인
        self.is_nav2_ready = False
        self.nav2_check_timer = self.create_timer(1.0, self.check_nav2_server_ready)
        self.get_logger().info("⏳ Waiting for Nav2 Server to come online...")

        # [중요] 런치 파일 실행 후 Nav2가 켜질 때까지 충분히 대기 (10초) 후 초기화
        self.get_logger().info("⏳ 10초 뒤에 런치 파일에서 설정한 위치로 초기화합니다...")
        self.timer_init = self.create_timer(10.0, self.set_initial_pose_once)

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

    def check_nav2_server_ready(self):
        if self._action_client.server_is_ready():
            self.get_logger().info("✅ [알림] Nav2 액션 서버 연결 성공!")
            self.is_nav2_ready = True
            self.nav2_check_timer.cancel()
        else:
            pass

    def set_initial_pose_once(self):
        self.timer_init.cancel()
        
        # [수정] 파라미터로 받은 좌표값 사용
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
                # GO_HOME 시에도 필요하다면 초기 파라미터 위치로 갈 수 있음
                # self.goal_queue = [ORIGIN_GOAL] 
                pass

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
        if not self.is_nav2_ready:
             self.get_logger().warn("⛔ [경고] Nav2 서버 준비 안됨. 명령 무시.")
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
            time.sleep(1.0)
            self.process_next_goal()
            self.robot_status = "IDLE"

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