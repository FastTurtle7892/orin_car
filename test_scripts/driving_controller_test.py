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
        self.get_logger().info("🚗 [주행 컨트롤러] 고정 초기 위치 모드 🚗") 
        self.get_logger().info("====================================")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.create_subscription(String, '/system_mode', self.mode_callback, qos_profile)
        self.create_subscription(String, '/driving/path_cmd', self.path_callback, qos_profile)
        
        # ✅ [복구] 고정 초기 위치 발행기 활성화
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.completion_pub = self.create_publisher(String, '/task_completion', 10)
        
        # ✅ [설정] 출발지 좌표 (사용자 환경에 맞게 확인 필요)
        # 만약 180도 돌아가 있다면 init_yaw를 수정해야 합니다. (예: -1.57 -> 1.57)
        self.declare_parameter('init_x', -0.8893)
        self.declare_parameter('init_y',  2.3)
        self.declare_parameter('init_yaw', -1.57) # -90도 (남쪽)

        self.current_mode = "IDLE"
        self.path_queue = []
        self.current_goal_handle = None

        self.get_logger().info("⏳ Ready. (Setting Initial Pose in 5s...)")
        self.timer_init = self.create_timer(5.0, self.set_initial_pose_once)

    def set_initial_pose_once(self):
        """ 로봇에게 '너는 여기에 있어'라고 강제로 알려줌 """
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
        
        # 초기 분산(Covariance) 설정: 작을수록 "이 위치가 확실해!"라는 뜻
        # 아까 AMCL 파라미터에서 initial_cov를 키웠으므로, 여기서도 약간 여유를 줍니다.
        pose_msg.pose.covariance[0] = 0.25  # X 분산
        pose_msg.pose.covariance[7] = 0.25  # Y 분산
        pose_msg.pose.covariance[35] = 0.06 # Yaw 분산 (약간의 회전 오차 허용)

        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 Initial Pose Set: ({x}, {y}, yaw={yaw})")

    def mode_callback(self, msg):
        previous_mode = self.current_mode
        self.current_mode = msg.data
        if previous_mode != self.current_mode:
            self.get_logger().info(f"🔄 System Mode Changed: {previous_mode} -> {self.current_mode}")

        if self.current_mode != "DRIVING" and self.current_goal_handle:
            self.cancel_nav2()

    def path_callback(self, msg):
        if self.current_mode != "DRIVING":
            self.get_logger().warn("⚠️ 경로 수신됨 (Auto Switch to DRIVING)")
            self.current_mode = "DRIVING"

        try:
            path_input = json.loads(msg.data)
            
            # CASE 1: Raw Coordinates
            if isinstance(path_input, list) and len(path_input) > 0 and isinstance(path_input[0], list):
                self.get_logger().info(f"📥 Raw Coordinates Received ({len(path_input)} points)")
                self.execute_raw_path(path_input)
                return

            # CASE 2: File Names
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

    def execute_raw_path(self, points):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for p in points:
            if len(p) < 3: continue 
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
            if self.path_queue:
                self.process_next_queue_file()
            else:
                self.publish_completion()
        elif result.status == 6: # ABORTED
            self.get_logger().warn("⚠️ Path Aborted (Status 6). Not sending COMPLETION signal.")
            self.current_goal_handle = None
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