#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import json
import time
import threading
import paho.mqtt.client as mqtt

# ==========================================
# 1. 설정 정보 (사용자 환경)
# ==========================================
MQTT_BROKER = "i14a402.p.ssafy.io"  # 또는 "localhost"
MQTT_PORT = 8183
CAR_ID = "car01"  # 테스트용 ID

# 토픽 정의
TOPIC_CMD = f"autowing_car/v1/{CAR_ID}/cmd"
TOPIC_MONITOR = f"autowing_car/v1/{CAR_ID}/monitoring"

# [테스트용] 검증된 목표 좌표 (사용자가 준 값)
TEST_GOAL = {
    'x': 0.884853,
    'y': 0.673968,
    'z': 0.0,
    'qz': 0.377434,
    'qw': 0.926037
}

class MqttNavBridge(Node):
    def __init__(self):
        super().__init__('mqtt_nav_bridge')
        
        # 1. Nav2 Action Client 설정
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("⏳ Waiting for Nav2 Action Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("✅ Nav2 Connected!")

        # 2. MQTT Client 설정
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        # MQTT 접속 시도
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.get_logger().info(f"✅ Connected to MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            self.get_logger().error(f"❌ MQTT Connection Failed: {e}")

        # MQTT 쓰레드 시작 (Non-blocking)
        self.mqtt_thread = threading.Thread(target=self.client.loop_forever)
        self.mqtt_thread.daemon = True
        self.mqtt_thread.start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"📡 MQTT Subscribed: {TOPIC_CMD}")
        client.subscribe(TOPIC_CMD)

    def on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode("utf-8")
            data = json.loads(payload_str)
            cmd = data.get("cmd")
            
            self.get_logger().info(f"📩 Received MQTT: {cmd}")

            # [핵심] START_MISSION 명령이 오면 Nav2로 이동 명령 전송
            if cmd == "START_MISSION":
                self.send_goal_to_nav2()
                
            elif cmd == "PAUSE":
                # (추후 구현) Nav2 취소 로직 등
                pass
                
        except Exception as e:
            self.get_logger().error(f"⚠️ JSON Parse Error: {e}")

    def send_goal_to_nav2(self):
        goal_msg = NavigateToPose.Goal()

        # 좌표 설정 (Map 좌표계 기준)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # 사용자가 준 검증된 좌표 적용
        goal_msg.pose.pose.position.x = TEST_GOAL['x']
        goal_msg.pose.pose.position.y = TEST_GOAL['y']
        goal_msg.pose.pose.position.z = TEST_GOAL['z']
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = TEST_GOAL['qz']
        goal_msg.pose.pose.orientation.w = TEST_GOAL['qw']

        self.get_logger().info(f"🚀 Sending Goal to Nav2: ({TEST_GOAL['x']}, {TEST_GOAL['y']})")
        
        # Nav2에게 전송
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        # (옵션) MQTT로 상태 전송 (출발 알림)
        self.publish_status("MOVING")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by Nav2')
            return

        self.get_logger().info('✅ Goal accepted! Robot is moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('🏁 Goal Reached! (Success)')
            self.publish_status("ARRIVED")
        else:
            self.get_logger().warn(f'⚠️ Goal Failed with status: {status}')

    def publish_status(self, status_text):
        # 로봇->서버 상태 보고 (Monitoring)
        monitor_data = {
            "carId": CAR_ID,
            "status": status_text,
            "timestamp": int(time.time())
        }
        json_str = json.dumps(monitor_data)
        self.client.publish(TOPIC_MONITOR, json_str)
        self.get_logger().info(f"📤 Sent Status: {status_text}")

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
