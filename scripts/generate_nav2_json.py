#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import json
import math
import os
import time

# ==========================================
# 1. 좌표 정의 (simul_nav_launch.py와 동일하게 설정)
# ==========================================
NODES = {
    "ORIGIN_GOAL":   {'x': -1.111, 'y': 0.201,  'yaw': -1.57},
    "NODE_1_COORD":  {'x': -1.111, 'y': -0.707, 'yaw': -1.57},
    "NODE_2_COORD":  {'x': -1.111, 'y': -1.615, 'yaw': -1.57},
    "NODE_3_COORD":  {'x': 1.15,   'y': -1.615, 'yaw': 1.57},
    "NODE_3_WAYPOINT": {'x': -0.195, 'y': -2.39, 'yaw': 0.0}
}

# 생성할 경로 목록 (출발지 -> 도착지)
PATH_SEQUENCE = [
    ("ORIGIN_GOAL", "NODE_1_COORD"),
    ("NODE_1_COORD", "NODE_2_COORD"),
    ("NODE_2_COORD", "NODE_3_COORD"),
    ("NODE_3_COORD", "ORIGIN_GOAL") # 되돌아오는 경로
]

SAVE_DIR = os.path.expanduser("~/trailer_paths") # 저장할 폴더

def euler_from_quaternion(x, y, z, w):
    """ 쿼터니언 -> 오일러 각 변환 """
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

class Nav2PathGenerator(Node):
    def __init__(self):
        super().__init__('nav2_path_generator')
        
        # Nav2의 ComputePathToPose 액션 클라이언트 (계획만 요청)
        self._action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        self.get_logger().info("⏳ Waiting for Nav2 Planner Server...")
        self._action_client.wait_for_server()
        self.get_logger().info("✅ Nav2 Planner Ready!")

        # 저장 폴더 생성
        if not os.path.exists(SAVE_DIR):
            os.makedirs(SAVE_DIR)

    def generate_all_paths(self):
        for start_name, goal_name in PATH_SEQUENCE:
            self.get_logger().info(f"generating: {start_name} -> {goal_name} ...")
            
            start_pose = NODES[start_name]
            goal_pose = NODES[goal_name]
            
            path_msg = self.get_nav2_path(start_pose, goal_pose)
            
            if path_msg:
                self.save_to_json(path_msg, start_name, goal_name)
            else:
                self.get_logger().error(f"❌ Failed to generate path: {start_name} -> {goal_name}")
            
            time.sleep(1.0) # 안전을 위해 약간 대기

    def get_nav2_path(self, start_data, goal_data):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.planner_id = "GridBased" # nav2_params.yaml에 설정된 플래너 ID (보통 GridBased)
        goal_msg.use_start = True # 현재 로봇 위치가 아닌, 지정한 시작점 사용
        
        # 시작점 설정
        goal_msg.start.header.frame_id = "map"
        goal_msg.start.header.stamp = self.get_clock().now().to_msg()
        goal_msg.start.pose.position.x = float(start_data['x'])
        goal_msg.start.pose.position.y = float(start_data['y'])
        yaw = float(start_data['yaw'])
        goal_msg.start.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.start.pose.orientation.w = math.cos(yaw / 2.0)

        # 도착점 설정
        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = float(goal_data['x'])
        goal_msg.goal.pose.position.y = float(goal_data['y'])
        yaw = float(goal_data['yaw'])
        goal_msg.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.goal.pose.orientation.w = math.cos(yaw / 2.0)

        # 동기적으로 요청 전송 및 대기
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Planner rejected request")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        
        return result.path

    def save_to_json(self, path_msg, start_name, goal_name):
        xs, ys, yaws = [], [], []

        for pose_stamped in path_msg.poses:
            p = pose_stamped.pose
            xs.append(round(p.position.x, 3))
            ys.append(round(p.position.y, 3))
            
            # Quaternion -> Yaw
            yaw = euler_from_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            yaws.append(round(yaw, 3))

        json_data = {
            "x": xs,
            "y": ys,
            "yaw": yaws,
            "start_id": start_name,
            "goal_id": goal_name
        }

        filename = f"path_{start_name}_to_{goal_name}_nav2.json"
        filepath = os.path.join(SAVE_DIR, filename)
        
        with open(filepath, 'w') as f:
            json.dump(json_data, f, indent=4)
        
        self.get_logger().info(f"💾 Saved {len(xs)} points to {filename}")

def main(args=None):
    rclpy.init(args=args)
    generator = Nav2PathGenerator()
    generator.generate_all_paths()
    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
