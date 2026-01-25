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

# 토픽 정의
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"

# 모니터링 전송 주기 (초 단위)
MONITOR_INTERVAL = 1.0 

# ==========================================
# 2. 목표 지점 설정
# ==========================================

# [목표 1] 테스트 지점
TEST_GOAL = {
    'x': 0.49,
    'y': 0.35,
    'z': 0.0,
    'yaw': 0.52
}

# [목표 2] 초기 위치 (원점)
ORIGIN_GOAL = {
    'x': 0.014,
    'y': 0.196,
    'z': 0.0,
    'yaw': 0.0
}

# [목표 3] 경유지 1 (User Provided)
WAYPOINT_1 = {
    'x': 0.82,
    'y': 0.76,
    'z': 0.0,
    'yaw' : 1.45
}

# [목표 4] 경유지 2 (User Provided)
WAYPOINT_2 = {
    'x': 0.77,
    'y': 1.50,
    'z': 0.0,
    'yaw' : 2.34
}

# 목표 지점 이름 매핑
GOAL_MAP = {
    "TEST_GOAL": TEST_GOAL,
    "ORIGIN_GOAL": ORIGIN_GOAL,
    "WAYPOINT_1": WAYPOINT_1,
    "WAYPOINT_2": WAYPOINT_2
}

def euler_from_quaternion(x, y, z, w):
    """ 쿼터니언 -> 오일러 각(Roll, Pitch, Yaw) 변환 """
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
        
        # 1. Nav2 Action Client 설정
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 Action Server...")
        self.get_logger().info("Nav2 Action Client Ready")

        # 2. 로봇 상태 및 주행 큐
        self.current_pose = None
        self.robot_status = "IDLE" 
        self.battery_level = 100
        self.goal_queue = []  # 순차 주행을 위한 큐

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
                # 기존 큐 초기화 후 새로운 경로 등록
                self.goal_queue = []
                
                if not path:
                    self.get_logger().warn("Path is empty! Staying idle.")
                    return

                # 경로 리스트를 순회하며 큐에 추가
                for name in path:
                    if name in GOAL_MAP:
                        self.goal_queue.append(GOAL_MAP[name])
                    else:
                        self.get_logger().warn(f"Unknown Goal Name: {name}")

                # 첫 번째 목표로 이동 시작
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
        """ 큐에서 다음 목표를 꺼내 이동 명령 전송 """
        if not self.goal_queue:
            self.get_logger().info("All goals completed!")
            self.robot_status = "ARRIVED"
            return

        # 큐의 첫 번째 목표 꺼내기
        next_goal = self.goal_queue.pop(0)
        self.get_logger().info(f"Processing next goal... Remaining: {len(self.goal_queue)}")
        self.send_goal_to_nav2(next_goal)

    def send_goal_to_nav2(self, target_data):
        # 서버 준비 대기
        if not self._action_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().warn("Nav2 Server is still not ready after 5s wait.")
             return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 설정
        goal_msg.pose.pose.position.x = target_data['x']
        goal_msg.pose.pose.position.y = target_data['y']
        goal_msg.pose.pose.position.z = target_data['z']
        
        # Yaw -> Quaternion 변환 또는 기존 Quaternion 사용
        if 'yaw' in target_data:
            yaw_rad = target_data['yaw']
            goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
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
            self.goal_queue = [] # 실패 시 남은 경로 취소
            return

        self.get_logger().info('Goal accepted! Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal Reached!')
            # 도착 후 다음 목표가 있는지 확인하여 이동
            self.process_next_goal()
        else:
            self.get_logger().warn(f'Goal Failed: {status}')
            self.robot_status = "IDLE"
            self.goal_queue = [] # 실패 시 남은 경로 취소

    def publish_periodic_status(self):
        """ 주기적 모니터링 데이터 전송 """
        x = 0.0
        y = 0.0
        heading_deg = 0.0

        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            
            # 쿼터니언 -> 도(Degree) 변환
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
