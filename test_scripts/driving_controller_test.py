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

# [설정] 경로 파일 폴더 (파일명을 쓸 때만 사용됨)
PATH_FOLDER = os.path.expanduser("~/trailer_paths2")

class DrivingController(Node):
    def __init__(self):
        super().__init__('driving_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🚗 [주행 컨트롤러] 좌표 수신 모드 지원 🚗") 
        self.get_logger().info("====================================")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.create_subscription(String, '/system_mode', self.mode_callback, qos_profile)
        self.create_subscription(String, '/driving/path_cmd', self.path_callback, qos_profile)
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.completion_pub = self.create_publisher(String, '/task_completion', 10) # 완료 신호용
        
        self.declare_parameter('init_x', -0.8893)
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
        previous_mode = self.current_mode
        self.current_mode = msg.data
        if previous_mode != self.current_mode:
            self.get_logger().info(f"🔄 System Mode Changed: {previous_mode} -> {self.current_mode}")

        if self.current_mode != "DRIVING" and self.current_goal_handle:
            self.cancel_nav2()

    def path_callback(self, msg):
        """ MQTT 노드로부터 경로 토픽을 받았을 때 실행됨 """
        if self.current_mode != "DRIVING":
            self.get_logger().warn("⚠️ 경로 수신됨 (Auto Switch to DRIVING)")
            self.current_mode = "DRIVING"

        try:
            path_input = json.loads(msg.data)
            
            # [수정된 부분] 수신된 데이터가 좌표 리스트인지 파일명인지 확인
            
            # CASE 1: 좌표 리스트가 직접 온 경우 [[x,y,yaw], [x,y,yaw], ...]
            if isinstance(path_input, list) and len(path_input) > 0 and isinstance(path_input[0], list):
                self.get_logger().info(f"📥 Raw Coordinates Received ({len(path_input)} points)")
                self.execute_raw_path(path_input)
                return

            # CASE 2: 파일명 리스트가 온 경우 ["P1.json", "P2.json"] (기존 방식 호환)
            self.get_logger().info(f"📥 File Paths Received: {path_input}")
            self.path_queue = []
            if isinstance(path_input, list):
                self.path_queue.extend(path_input)
            else:
                self.path_queue.append(path_input)
                
            self.process_next_queue_file()
            
        except Exception as e:
            self.get_logger().error(f"❌ Path Decode Error: {e}")

    def process_next_queue_file(self):
        if not self.path_queue:
            self.get_logger().info("✅ All file paths completed.")
            self.publish_completion()
            return

        filename = self.path_queue.pop(0)
        self.execute_json_file(filename)

    # -------------------------------------------------------------------
    # [신규] 좌표 리스트를 바로 Nav2 경로로 변환하여 실행
    # -------------------------------------------------------------------
    def execute_raw_path(self, points):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for p in points:
            if len(p) < 3: continue # [x, y, yaw] 형식이 아니면 스킵
            
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            yaw = float(p[2])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            ros_path.poses.append(pose)

        self.get_logger().info(f"🚀 Sending Raw Path Goal (Points: {len(ros_path.poses)})")
        self.send_nav2_goal(ros_path)

    # -------------------------------------------------------------------
    # [기존] 파일을 읽어서 실행하는 함수
    # -------------------------------------------------------------------
    def execute_json_file(self, filename):
        full_path = os.path.join(PATH_FOLDER, filename)
        if not os.path.exists(full_path):
            self.get_logger().error(f"❌ File not found: {full_path}")
            self.process_next_queue_file()
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

        self.get_logger().info(f"🚀 Sending File Path Goal: {filename}")
        self.send_nav2_goal(ros_path)

    # -------------------------------------------------------------------
    # [공통] Nav2로 Goal 전송
    # -------------------------------------------------------------------
    def send_nav2_goal(self, ros_path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = "FollowPath"
        goal_msg.goal_checker_id = "general_goal_checker"

        self._action_client.wait_for_server()
        
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
            
            # 큐에 남은 파일이 있다면 계속 진행 (파일 모드일 때)
            if self.path_queue:
                self.process_next_queue_file()
            else:
                self.publish_completion()
        else:
            self.get_logger().warn(f"⚠️ Path Ended with status: {result.status}")
            self.current_goal_handle = None

    def publish_completion(self):
        msg = String()
        msg.data = "DRIVING_COMPLETE"
        self.completion_pub.publish(msg)
        self.get_logger().info("📢 주행 완료 신호 전송 (DRIVING_COMPLETE)")

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