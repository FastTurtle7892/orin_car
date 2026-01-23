#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import math

# ================= 설정 =================
MQTT_BROKER = "i14a402.p.ssafy.io"
MQTT_PORT = 8183
CAR_ID = "car01"

TOPIC_MONITORING = f"autowing_car/v1/{CAR_ID}/monitoring"
# =======================================

class MqttBridge(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        # 데이터 저장 변수
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.current_status = "IDLE"
        self.battery_level = 100  # 배터리는 현재 더미값 (필요시 센서 연동)

        # MQTT 연결
        self.client = mqtt.Client(client_id=f"AutoWing_{CAR_ID}_Monitor")
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
            self.get_logger().info(f"✅ Monitoring Bridge Connected to {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"❌ Connection Error: {e}")

        # ROS 구독
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(String, '/robot_status', self.status_callback, 10)

        # 주기적 전송 (0.5초마다) - 데이터가 너무 자주 바뀌는 것을 방지
        self.timer = self.create_timer(0.5, self.publish_monitoring_data)

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Quaternion -> Heading (Degree)
        q = msg.pose.pose.orientation
        yaw_rad = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.current_heading = math.degrees(yaw_rad)

    def status_callback(self, msg):
        self.current_status = msg.data

    def publish_monitoring_data(self):
        # 명세서 포맷 준수
        payload = {
            "x": round(self.current_x, 2),
            "y": round(self.current_y, 2),
            "battery": self.battery_level,
            "status": self.current_status,
            "heading": round(self.current_heading, 2)
        }
        
        try:
            self.client.publish(TOPIC_MONITORING, json.dumps(payload))
            # self.get_logger().info(f"Sent: {payload}") # 디버깅용
        except Exception as e:
            self.get_logger().warn(f"Publish fail: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
