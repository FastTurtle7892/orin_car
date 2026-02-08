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
import threading
import time

# [설정] 경로 파일 폴더
PATH_FOLDER = os.path.expanduser("~/trailer_paths5")

class DrivingController(Node):
    def __init__(self):
        super().__init__('driving_controller')
        
        self.get_logger().info("====================================")
        self.get_logger().info("🚗 [주행 컨트롤러] 하이브리드 시퀀스 모드 🚗") 
        self.get_logger().info("   - CMD(하드코딩) + Nav2(파일) 자동 전환")
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
        
        # Ackermann Driver 정보 (Wheelbase)
        self.wheelbase = 0.145

        # 초기 위치 파라미터 (사용자 요청 값 적용)
        self.declare_parameter('init_x', -0.895)
        self.declare_parameter('init_y',  2.3)
        self.declare_parameter('init_yaw', -1.57) 

        self.current_mode = "IDLE"
        self.current_goal_handle = None
        self.stop_signal = False  # 스레드 제어용
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
        if self.current_mode != "DRIVING":
            self.stop_signal = True
            if self.current_goal_handle:
                self.cancel_nav2()
            self._stop_robot()

    def path_callback(self, msg):
        if self.current_mode != "DRIVING":
            self.current_mode = "DRIVING"

        try:
            path_input = json.loads(msg.data)
            self.stop_signal = False
            
            # ✅ 리스트가 들어오면 하이브리드 시퀀스 실행 (파일+CMD 혼합 처리)
            if isinstance(path_input, list) and len(path_input) > 0:
                self.get_logger().info(f"📜 [작업 큐 수신] 총 {len(path_input)} 단계")
                threading.Thread(target=self._run_hybrid_sequence, args=(path_input,), daemon=True).start()
                
            elif isinstance(path_input, str):
                 threading.Thread(target=self._run_hybrid_sequence, args=([path_input],), daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"❌ Message Parsing Error: {e}")

    # ================= [핵심] 하이브리드 시퀀스 실행기 =================
    def _run_hybrid_sequence(self, execution_queue):
        idx = 0
        while idx < len(execution_queue) and not self.stop_signal:
            item = execution_queue[idx]
            
            # 1. 하드코딩 명령어 처리
            if "CMD_" in item:
                self.get_logger().info(f"▶ [Step {idx+1}] 하드코딩 실행: {item}")
                self._execute_hardcoded_step_sync(item)
                idx += 1
                
            # 2. 파일 경로(Nav2) 처리
            else:
                # 연속된 파일은 하나로 병합해서 Nav2에 전달 (효율성)
                files_to_merge = []
                while idx < len(execution_queue) and "CMD_" not in execution_queue[idx]:
                    files_to_merge.append(execution_queue[idx])
                    idx += 1
                
                self.get_logger().info(f"▶ [Step {idx}] Nav2 주행 시작 (파일 {len(files_to_merge)}개 병합)")
                success = self._execute_nav2_step_sync(files_to_merge)
                if not success:
                    self.get_logger().error("❌ Nav2 주행 실패로 전체 시퀀스 중단")
                    return

            # 단계 전환 시 잠시 안정화
            if not self.stop_signal:
                time.sleep(0.5)

        if not self.stop_signal:
            self.get_logger().info("🏁 모든 시퀀스 완료!")
            self.publish_completion()

    # ----------------- [A] 하드코딩 동기 실행 (Blocking) -----------------
    def _execute_hardcoded_step_sync(self, cmd):
        steps = self._parse_command_multi_step(cmd)
        if not steps:
            self.get_logger().warn(f"⚠️ 정의되지 않은 명령어: {cmd}")
            return

        for step_idx, (deg, dur, direct, v_start, v_end) in enumerate(steps):
            if self.stop_signal: return
            self._run_single_motion(deg, dur, direct, v_start, v_end)
        
        self._stop_robot()

    def _parse_command_multi_step(self, cmd):
        """ 정의된 하드코딩 패턴 반환 """
        if cmd == "CMD_HARD_RIGHT_2S":
            # P4: 완만한 우회전
            return [
                    (0.0, 5.0, 1, 2.0, 1.5),
                    (-10.0, 5.0, 1, 1.5, 0.0)
                ]
        elif cmd == "CMD_HARD_LEFT_BACK_2S":
            # P5: 후진 (회전 -> 직진 -> 회전)
            return [
                (40.0, 2.5, -1, 2.0, 1.5),  # 진입 회전
                (0.0,  4.0, -1, 1.5, 1.0)  # 중간 직진
            ]
        elif cmd == "CMD_HARD_RIGHT_40_3S":
            # P6: 전진 (긴 직진 -> 꺾어서 진입)
            return [
                (-13.0, 14.0, 1, 1.2, 0.0)   # 꺾어서 진입
            ]
        elif cmd == "CMD_HARD_FWD_1S":
            # P7: 단순 직진 (사용 여부에 따라 유지)
            return [(0.0, 4.0, 1, 2.0, 0.0)]
        else:
            return []

    def _run_single_motion(self, steering_deg, duration, direction, start_speed, end_speed):
        rate_hz = 50 
        dt = 1.0 / rate_hz
        steps = int(duration * rate_hz)
        
        twist = Twist()
        for i in range(steps):
            if self.stop_signal or self.current_mode != "DRIVING": 
                self._stop_robot()
                return

            alpha = i / float(steps)
            current_v = (start_speed * (1.0 - alpha) + end_speed * alpha) * direction
            
            if abs(current_v) < 0.01:
                current_w = 0.0
            else:
                rad_steering = math.radians(steering_deg)
                current_w = (current_v * math.tan(rad_steering)) / self.wheelbase

            twist.linear.x = float(current_v)
            twist.angular.z = float(current_w)
            
            self.cmd_vel_pub.publish(twist)
            time.sleep(dt)

    def _stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    # ----------------- [B] Nav2 동기 실행 (Blocking) -----------------
    def _execute_nav2_step_sync(self, filenames):
        # 1. 파일 읽기 및 병합
        all_xs, all_ys, all_yaws = [], [], []
        for filename in filenames:
            if not filename.endswith('.json'): filename += '.json'
            if filename.startswith("/"): full_path = filename
            else: full_path = os.path.join(PATH_FOLDER, filename)
            
            if os.path.exists(full_path):
                try:
                    with open(full_path, 'r') as f:
                        data = json.load(f)
                        all_xs.extend(data.get("x", []))
                        all_ys.extend(data.get("y", []))
                        all_yaws.extend(data.get("yaw", [0.0] * len(data.get("x", []))))
                except: pass

        if not all_xs: return False
        
        # 보간
        final_xs, final_ys, final_yaws = self.interpolate_points(all_xs, all_ys, all_yaws, step=0.05)
        
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

        # 2. Nav2 Action 전송 및 대기
        goal_msg = FollowPath.Goal()
        goal_msg.path = ros_path
        goal_msg.controller_id = "FollowPath"
        goal_msg.goal_checker_id = "general_goal_checker"

        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)
        
        # Future 대기 루프 (스레드 블로킹 방지하며 대기)
        while not send_future.done():
            if self.stop_signal: 
                self.cancel_nav2()
                return False
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Nav2 경로 거부됨.")
            return False
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()

        while not result_future.done():
            if self.stop_signal:
                self.cancel_nav2()
                return False
            time.sleep(0.1)

        status = result_future.result().status
        self.current_goal_handle = None
        
        if status == 4 or status == 6: 
            self.get_logger().info(f"✨ Nav2 주행 완료 (Status: {status}) - 다음 단계 진행")
            return True
        else:
            self.get_logger().warn(f"⚠️ Nav2 주행 실패 (Status: {status})")
            return False

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

    def cancel_nav2(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
        self.cmd_vel_pub.publish(Twist())

    def publish_completion(self):
        msg = String()
        msg.data = "DRIVING_COMPLETE"
        self.completion_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrivingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()