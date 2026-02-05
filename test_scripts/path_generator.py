import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import json
import math
import time

# 쿼터니언 -> Yaw 변환 (저장용)
def euler_from_quaternion(x, y, z, w):
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

# Yaw -> 쿼터니언 변환 (입력용)
def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.path_pub = self.create_publisher(Path, '/visualized_plan', 10)

    def get_path(self, start_pose, goal_pose):
        if not self.planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Planner Server 안켜짐! launch 파일 확인하세요.')
            return None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose
        goal_msg.planner_id = "GridBased" 
        goal_msg.use_start = True

        self.get_logger().info(f'경로 계산 시작: {start_pose.pose.position.x},{start_pose.pose.position.y} -> {goal_pose.pose.position.x},{goal_pose.pose.position.y}')
        
        send_goal_future = self.planner_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('요청 거부됨 (출발지나 목적지가 벽일 수 있음)')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.path

    def save_and_visualize(self, path_msg, filename="generated_path.json"):
        if not path_msg or len(path_msg.poses) == 0:
            self.get_logger().warn("경로를 못 찾았습니다! (장애물에 막힘)")
            return

        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'RViz 발행 완료! 총 {len(path_msg.poses)}개의 점')

        path_data = []
        for pose_stamped in path_msg.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            qx = pose_stamped.pose.orientation.x
            qy = pose_stamped.pose.orientation.y
            qz = pose_stamped.pose.orientation.z
            qw = pose_stamped.pose.orientation.w
            yaw = euler_from_quaternion(qx, qy, qz, qw)

            path_data.append({
                "x": round(px, 4),
                "y": round(py, 4),
                "yaw": round(yaw, 4)
            })

        with open(filename, 'w') as f:
            json.dump(path_data, f, indent=4)
        self.get_logger().info(f'저장 완료: {filename}')

# [수정됨] 각도 단위를 Radian으로 변경 (변수명: yaw_rad)
def create_pose(x, y, yaw_rad=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = x
    p.pose.position.y = y
    
    # 입력받은 라디안 값을 변환 없이 그대로 사용
    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw_rad)
    
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p

def main():
    rclpy.init()
    generator = PathGenerator()

    # ==========================================
    # ▼ 여기 좌표만 수정해서 쓰세요! ▼
    # ==========================================
    
    # 예: 출발지 (x=0.0, y=0.0, yaw=0 rad)
    start = create_pose(x=-0.2, y=0.50, yaw_rad=-1.48)
    
    # 예: 목적지 (x=0.5, y=0.0, yaw=1.57 rad -> 약 90도)
    goal = create_pose(x=-0.0307, y=-1.00, yaw_rad=-1.57)
    
    # ==========================================

    path = generator.get_path(start, goal)
    
    if path:
        generator.save_and_visualize(path)
        time.sleep(2.0) 

    generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
# 엣지 1
# start = create_pose(x=-0.778518, y=2.98053, yaw_rad=-1.58) 
# goal = create_pose(x=-0.222045, y=1.49575, yaw_rad=-1.00292)

# 엣지 2
# start = create_pose(x=-0.222045, y=1.49575, yaw_rad=-1.00292)
# goal = create_pose(x=-0.7781, y=2.2457, yaw_rad=-1.00292)

# 도킹 부분
# start = create_pose(x=-0.7781, y=2.2457, yaw_rad=-1.00292)
# goal = create_pose(x=-0.8893, y=2.3957, yaw_rad=-1.00292)

# 엣지 3
# start = create_pose(x=-0.8893, y=2.3957, yaw_rad=-1.00292)
# goal = create_pose(x=-0.2, y=0.50, yaw_rad=-1.48)

# 엣지 4
# start = create_pose(x=-0.2, y=0.50, yaw_rad=-1.48)
# goal = create_pose(x=-0.0307, y=-1.00, yaw_rad=-1.57)

# 엣지 5
# start = create_pose(x=-0.0307, y=-1.00, yaw_rad=-1.57)
# goal = create_pose(x=-0.48518, y=-2.60, yaw_rad=-2.20)

# 엣지 6
# start = create_pose(x=-0.48518, y=-2.60, yaw_rad=-2.20)
# goal = create_pose(x=1.00518, y=-1.80, yaw_rad=-3.10)

# 엣지 7
# start = create_pose(x=1.00518, y=-1.80, yaw_rad=-3.10)
# goal = create_pose(x=-0.8893, y=-1.00, yaw_rad=1.57)

# 엣지 8
# start = create_pose(x=-0.8893, y=-1.00, yaw_rad=1.57)
# goal = create_pose(x=-0.8893, y=2.00, yaw_rad=1.57)
