#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import json
import math
import os

# [설정] 경로 파일 폴더 (경로 확인 필수!)
PATH_FOLDER = os.path.expanduser("~/ros_ws/src/orin_car/paths")

class DrivingController(Node):
    def __init__(self):
        super().__init__('driving_controller')
        
        # 1. 구독
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        self.create_subscription(String, '/driving/path_cmd', self.path_callback, 10)
        
        # 2. Nav2 클라이언트
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        
        # 3. 초기 위치 설정 퍼블리셔
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        
        # 초기 위치 파라미터 (기본값 설정)
        self.declare_parameter('init_x', -1.111)
        self.declare_parameter('init_y', 0.201)
        self.declare_parameter('init_yaw', -1.57)

        self.current_mode = "IDLE"
        self.path_queue = []
        
        # 10초 뒤에 초기 위치 자동 설정
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
        if self.current_mode != "DRIVING":
            # 필요 시 주행 취소 로직 추가 가능
            pass

    def path_callback(self, msg):
        if self.current_mode != "DRIVING":
            self.get_logger().warn("⚠️ Not in DRIVING mode. Command ignored.")
            return

        try:
            path_input = json.loads(msg.data)
            self.path_queue = []
            
            if isinstance(path_input, list):
                self.path_queue.extend(path_input)
            else:
                self.path_queue.append(path_input)
                
            self.process_next_path()
            
        except Exception as e:
            self.get_logger().error(f"❌ Path Error: {e}")

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
            return

        with open(full_path, 'r') as f:
            data = json.load(f)

        xs = data.get("x", [])
        ys = data.get("y", [])
        yaws = data.get("yaw", [0.0]*len(xs))

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
        self._action_client.wait_for_server()
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Path Rejected by Nav2")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("🏁 Path Finished.")
        self.process_next_path()

def main(args=None):
    rclpy.init(args=args)
    node = DrivingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()