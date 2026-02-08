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
import numpy as np

# [설정] 경로 파일 폴더
PATH_FOLDER = os.path.expanduser("~/trailer_paths5")

class DrivingController(Node):
    def __init__(self):
        super().__init__('driving_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🚗 [주행 컨트롤러] 경로 자동 병합(Merge) 모드 🚗") 
        self.get_logger().info("   - 여러 파일이 들어오면 하나로 합쳐서 실행합니다.")
        self.get_logger().info("   - 연결 부위에서 멈추지 않습니다.")
        self.get_logger().info("====================================")
        
        self.get_logger().info(f"📂 [Target Folder]: {PATH_FOLDER}")
        
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
        self.completion_pub = self.create_publisher(String, '/task_completion', 10)
        
        # 초기 위치 파라미터
        self.declare_parameter('init_x', -0.8893)
        self.declare_parameter('init_y',  2.3)
        self.declare_parameter('init_yaw', -1.57) 

        self.current_mode = "IDLE"
        self.current_goal_handle = None
        self.timer_init = self.create_timer(5.0, self.set_initial_pose_once)

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
        pose_msg.pose.covariance[0] = 0.25
        pose_msg.pose.covariance[7] = 0.25
        pose_msg.pose.covariance[35] = 0.06

        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f"📍 Initial Pose Set: ({x}, {y}, yaw={yaw})")

    def mode_callback(self, msg):
        self.current_mode = msg.data
        if self.current_mode != "DRIVING" and self.current_goal_handle:
            self.cancel_nav2()

    def path_callback(self, msg):
        if self.current_mode != "DRIVING":
            self.current_mode = "DRIVING"

        try:
            path_input = json.loads(msg.data)
            
            # 1. 파일명 리스트가 들어온 경우 -> [핵심] 병합 실행
            if isinstance(path_input, list) and len(path_input) > 0 and isinstance(path_input[0], str):
                self.get_logger().info(f"📥 [파일 리스트 수신]: {path_input}")
                self.execute_merged_path(path_input)
                
            # 2. 파일명 하나만 들어온 경우
            elif isinstance(path_input, str):
                self.execute_merged_path([path_input])
                
            # 3. 좌표 데이터가 직접 들어온 경우 (예외 처리)
            elif isinstance(path_input, list) and len(path_input) > 0 and isinstance(path_input[0], list):
                self.execute_raw_path(path_input)

        except Exception as e:
            self.get_logger().error(f"❌ Message Parsing Error: {e}")

    # ✅ [핵심 기능] 여러 파일을 하나로 합치는 함수
    def execute_merged_path(self, filenames):
        all_xs, all_ys, all_yaws = [], [], []
        
        self.get_logger().info("🔄 경로 병합 시작...")

        for filename in filenames:
            # 절대 경로가 아니면 폴더 경로 결합
            if not filename.endswith('.json'): filename += '.json'
            
            if filename.startswith("/"):
                full_path = filename
            else:
                full_path = os.path.join(PATH_FOLDER, filename)
            
            if not os.path.exists(full_path):
                self.get_logger().error(f"❌ 파일 없음 (건너뜀): {full_path}")
                continue

            try:
                with open(full_path, 'r') as f:
                    data = json.load(f)
                    # 데이터 이어 붙이기 (extend)
                    all_xs.extend(data.get("x", []))
                    all_ys.extend(data.get("y", []))
                    # yaw가 없으면 0.0으로 채움
                    all_yaws.extend(data.get("yaw", [0.0] * len(data.get("x", []))))
                    self.get_logger().info(f"   + {filename} 로드 완료 ({len(data.get('x', []))} points)")
            except Exception as e:
                self.get_logger().error(f"❌ 파일 읽기 에러 {filename}: {e}")

        if not all_xs:
            self.get_logger().error("⚠️ 유효한 경로 데이터가 없습니다.")
            return

        # ✅ [옵션] 보간(Smoothing) 적용 - 점 사이를 촘촘하게 채움
        # (만약 데이터가 이미 충분히 많다면 이 부분 주석 처리 가능)
        final_xs, final_ys, final_yaws = self.interpolate_points(all_xs, all_ys, all_yaws, step=0.05)
        
        self.get_logger().info(f"✨ 병합 및 보간 완료! 총 {len(final_xs)}개 포인트 전송")
        
        # Nav2 메시지 생성
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(final_xs)):
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(final_xs[i])
            pose.pose.position.y = float(final_ys[i])
            yaw = float(final_yaws[i])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            ros_path.poses.append(pose)

        # 한 번에 전송 (Goal 1개)
        self.send_nav2_goal(ros_path)

    def interpolate_points(self, xs, ys, yaws, step=0.05):
        new_xs, new_ys, new_yaws = [], [], []
        for i in range(len(xs) - 1):
            curr_x, curr_y = xs[i], ys[i]
            next_x, next_y = xs[i+1], ys[i+1]
            curr_yaw, next_yaw = yaws[i], yaws[i+1]
            
            dist = math.sqrt((next_x - curr_x)**2 + (next_y - curr_y)**2)
            if dist < step:
                new_xs.append(curr_x); new_ys.append(curr_y); new_yaws.append(curr_yaw)
                continue
            
            num_points = int(dist / step)
            for j in range(num_points):
                alpha = j / num_points
                interp_x = curr_x * (1 - alpha) + next_x * alpha
                interp_y = curr_y * (1 - alpha) + next_y * alpha
                
                diff_yaw = next_yaw - curr_yaw
                while diff_yaw > math.pi: diff_yaw -= 2*math.pi
                while diff_yaw < -math.pi: diff_yaw += 2*math.pi
                interp_yaw = curr_yaw + diff_yaw * alpha
                
                new_xs.append(interp_x); new_ys.append(interp_y); new_yaws.append(interp_yaw)
        
        new_xs.append(xs[-1]); new_ys.append(ys[-1]); new_yaws.append(yaws[-1])
        return new_xs, new_ys, new_yaws

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
            self.get_logger().error("❌ Nav2 경로 거부됨.")
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result.status == 4: # SUCCEEDED
            self.get_logger().info("🏁 전체 경로 주행 완료.")
            self.publish_completion()
        else:
            self.get_logger().warn(f"⚠️ 주행 비정상 종료 (Status: {result.status})")
            self.current_goal_handle = None

    def publish_completion(self):
        msg = String()
        msg.data = "DRIVING_COMPLETE"
        self.completion_pub.publish(msg)

    def execute_raw_path(self, points):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        ros_path.header.stamp = self.get_clock().now().to_msg()
        for p in points:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            ros_path.poses.append(pose)
        self.send_nav2_goal(ros_path)

    def cancel_nav2(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = DrivingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()