#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
import json
import math
import os

# [설정] 경로 파일 폴더
PATH_FOLDER = os.path.expanduser("~/trailer_paths2")

class DrivingController(Node):
    def __init__(self):
        super().__init__('driving_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🚗 [주행 컨트롤러] ROS2 토픽 대기 모드 🚗") 
        self.get_logger().info("====================================")

        # [핵심] QoS 설정 (MQTT 노드와 짝을 맞춤)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # 1. 구독 (MQTT 브릿지가 보내주는 토픽을 받음)
        self.create_subscription(String, '/system_mode', self.mode_callback, qos_profile)
        self.create_subscription(String, '/driving/path_cmd', self.path_callback, qos_profile)
        
        # 2. 퍼블리셔 & 액션 클라이언트
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        
        # 3. 파라미터
        self.declare_parameter('init_x',-0.8893)
        self.declare_parameter('init_y',  2.5)
        self.declare_parameter('init_yaw', -1.57)

        self.current_mode = "IDLE"
        self.path_queue = []
        self.current_goal_handle = None

        self.get_logger().info("⏳ Driving Controller Ready. (Auto Init Pose in 10s)")
        self.timer_init = self.create_timer(10.0, self.set_initial_pose_once)

    def set_initial_pose_once(self):
        self.timer_init.cancel()
        x = self.get_parameter('init_x').value
        y = self.get_parameter('init_y').value
        yaw = self.get_parameter('init_yaw').value
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 Initial Pose Set: ({x}, {y})")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode != "DRIVING" and self.current_goal_handle:
            self.cancel_nav2()

    def path_callback(self, msg):
        """ MQTT 노드로부터 경로 토픽을 받았을 때 실행됨 """
        if self.current_mode != "DRIVING":
            self.get_logger().warn("⚠️ Not in DRIVING mode. Ignoring path.")
            return

        try:
            # JSON 문자열을 파싱해서 리스트로 변환
            path_input = json.loads(msg.data)
            self.get_logger().info(f"📥 New Path Received: {path_input}")
            
            # 큐 초기화 및 등록
            self.path_queue = []
            if isinstance(path_input, list):
                self.path_queue.extend(path_input)
            else:
                self.path_queue.append(path_input)
                
            self.process_next_path()
            
        except Exception as e:
            self.get_logger().error(f"❌ Path Decode Error: {e}")

    def process_next_path(self):
        if not self.path_queue:
            self.get_logger().info("✅ All paths completed.")
            return

        filename = self.path_queue.pop(0)
        self.execute_json_path(filename)

    def execute_json_path(self, filename):
        full_path = os.path.join(PATH_FOLDER, filename)
        if not os.path.exists(full_path):
            self.get_logger().error(f"❌ File not found: {full_path}")
            self.process_next_path()
            return

        with open(full_path, 'r') as f:
            data = json.load(f)

        xs = data.get("x", [])
        ys = data.get("y", [])
        yaws = data.get("yaw", [0.0]*len(xs))

        # Nav2 Path 메시지 생성
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(xs)):
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(xs[i])
            pose.pose.position.y = float(ys[i])
            yaw = float(yaws[i])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            ros_path.poses.append(pose)

        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = "FollowPath"
        goal_msg.goal_checker_id = "general_goal_checker"

        self.get_logger().info(f"🚀 Sending Path: {filename}")
        self._action_client.wait_for_server() # Nav2 켜질 때까지 대기
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Path Rejected by Nav2")
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4: # SUCCEEDED
            self.get_logger().info("🏁 Path Segment Finished.")
            self.process_next_path()
        else:
            self.get_logger().warn(f"⚠️ Path Ended with status: {result.status}")
            self.current_goal_handle = None

    def cancel_nav2(self):
        if self.current_goal_handle:
            self.get_logger().warn("⚠️ Cancelling Nav2...")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        
        msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
        self.path_queue = []

def main(args=None):
    rclpy.init(args=args)
    node = DrivingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()