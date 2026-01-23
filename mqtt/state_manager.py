#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class RobotStateManager(Node):
    def __init__(self):
        super().__init__('robot_state_manager')

        # === [1] 상태 정의 ===
        # 초기 상태
        self.current_mission = "IDLE"   # 논리적 임무 상태 (예: DOCKING, RETURNING)
        self.is_moving = False          # 물리적 이동 여부
        self.last_cmd_time = 0.0

        # === [2] Publisher (최종 상태 발행) ===
        # 이 토픽(/robot_status)을 mqtt_bridge가 구독해서 서버로 보냄
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # 0.5초마다 상태 업데이트 및 발행
        self.timer = self.create_timer(0.5, self.publish_status)

        # === [3] Subscriber ===
        # A. 속도 명령 감시 (실제로 움직이는지 판단)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # B. 외부 명령 감시 (Nav2나 MissionClient가 보낸 상태 변경 요청 수신)
        self.create_subscription(String, '/mission_cmd', self.mission_callback, 10)

        self.get_logger().info("🤖 Robot State Manager Started")

    def cmd_vel_callback(self, msg):
        # 선속도나 각속도가 0이 아니면 움직이는 것으로 간주
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.is_moving = True
            self.last_cmd_time = time.time()
        else:
            # 멈췄더라도 바로 False로 두지 않고, 잠시 대기할 수도 있음(여기선 즉시 반영)
            self.is_moving = False

    def mission_callback(self, msg):
        # 외부(MissionClient 등)에서 강제로 상태를 변경하고 싶을 때 사용
        command = msg.data
        self.get_logger().info(f"State Command Received: {command}")

        if command == "dock":
            self.current_mission = "DOCKING"
        elif command == "return":
            self.current_mission = "RETURNING"
        elif command == "stop" or command == "idle":
            self.current_mission = "IDLE"
        elif command == "moving":
            self.current_mission = "MOVING"

    def publish_status(self):
        # === [4] 최종 상태 결정 로직 ===
        final_status = "IDLE"

        # 1. 속도 명령이 있어서 실제로 움직이고 있다면 기본적으로 MOVING
        if self.is_moving:
            # 특수 임무(도킹, 복귀) 중이라면 그 상태를 유지
            if self.current_mission in ["DOCKING", "RETURNING"]:
                final_status = self.current_mission
            else:
                final_status = "MOVING"
        else:
            # 움직이지 않을 때
            # 최근에 움직였었다면(2초 내) 잠시 대기 중일 수 있으므로 상태 유지 고려 가능
            # 여기서는 단순하게 IDLE로 처리하거나, 미션 상태가 있으면 유지
            if time.time() - self.last_cmd_time < 2.0 and self.current_mission == "MOVING":
                 final_status = "MOVING" # 잠시 멈춤은 주행 중으로 간주
            elif self.current_mission in ["DOCKING", "RETURNING"]:
                 final_status = self.current_mission # 미션 중 대기
            else:
                 final_status = "IDLE"

        # 상태 메시지 발행
        msg = String()
        msg.data = final_status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
