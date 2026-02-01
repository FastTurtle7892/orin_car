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

# 경로 파일 폴더 (필요시 수정)
PATH_FOLDER = os.path.expanduser("~/trailer_paths")

class DrivingControllerFinal(Node):
    def __init__(self):
        # 이름도 driving_controller로 변경!
        super().__init__('driving_controller_final') 

        # 1. 구독: 시스템 모드 (켜고 끄기용)
        self.create_subscription(String, '/system_mode', self.mode_callback, 10)
        # 2. 구독: 경로 명령 (일감 받기용)
        self.create_subscription(String, '/driving/path_cmd', self.path_callback, 10)
        
        self.current_mode = "IDLE"
        
        # Nav2 Action Client
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self._goal_handle = None 

        # 초기 위치 설정용 퍼블리셔
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.declare_parameter('init_x', -1.111)
        self.declare_parameter('init_y', 0.201)
        self.declare_parameter('init_yaw', -1.57)
        
        # 10초 뒤 초기 위치 설정 (한번만)
        self.timer_init = self.create_timer(10.0, self.set_initial_pose_once)

        self.path_queue = []
        self.get_logger().info("✅ Driving Controller Ready (Waiting for ROS Topic)")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        # 내가 일할 시간(DRIVING)이 아니면 하던 거 멈춤
        if self.current_mode != "DRIVING":
            self.cancel_nav2()

    def path_callback(self, msg):
        """ MQTT Bridge가 던져준 경로 리스트를 받음 """
        try:
            # JSON 문자열 -> 리스트 변환
            path_input = json.loads(msg.data)
            self.get_logger().info(f"📥 Path Received: {path_input}")
            
            # 모드가 DRIVING일 때만 수락
            if self.current_mode == "DRIVING":
                self.path_queue = []
                if isinstance(path_input, list): self.path_queue.extend(path_input)
                else: self.path_queue.append(path_input)
                self.process_next_path()
            else:
                self.get_logger().warn("⚠️ Path received but mode is NOT DRIVING. Ignored.")
                
        except Exception as e:
            self.get_logger().error(f"Path Parsing Error: {e}")

    def cancel_nav2(self):
        if self._goal_handle is not None and self._goal_handle.accepted:
            self.get_logger().warn("🛑 Stopping Driving (Mode Changed)")
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            self.path_queue = []

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
        pose_msg.pose.pose.orientation.z = math.sin(init_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(init_yaw / 2.0)
        
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 Initial Pose Set")

    def process_next_path(self):
        if not self.path_queue:
            self.get_logger().info("✅ All paths finished.")
            return

        next_file = self.path_queue.pop(0)
        self.execute_json_path(next_file)

    def execute_json_path(self, filename):
        full_path = os.path.join(PATH_FOLDER, filename)
        if not os.path.exists(full_path):
            self.get_logger().error(f"❌ File not found: {full_path}")
            self.process_next_path() # 다음 파일 시도
            return

        with open(full_path, 'r') as f:
            data = json.load(f)

        xs, ys, yaws = data.get("x", []), data.get("y", []), data.get("yaw", [])
        if not xs: return

        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(xs)):
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(xs[i])
            pose.pose.position.y = float(ys[i])
            yaw = float(yaws[i]) if i < len(yaws) else 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            ros_path.poses.append(pose)

        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = "FollowPath"
        goal_msg.goal_checker_id = "general_goal_checker"

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.process_next_path() # 하나 끝나면 다음 거 실행

def main(args=None):
    rclpy.init(args=args)
    node = DrivingControllerFinal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()