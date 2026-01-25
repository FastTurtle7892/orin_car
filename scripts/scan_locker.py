#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class ScanLocker(Node):
    def __init__(self):
        super().__init__('scan_locker')
        
        # 설정: 속도가 이 값보다 작으면 정지로 간주
        self.stop_threshold = 0.01
        self.is_moving = False
        self.locked_scan = None
        
        # 1. 원본 스캔 구독
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 2. 명령 속도 구독 (로봇이 멈췄는지 알기 위해)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # 3. 오도메트리용 스캔 발행 (RF2O가 이걸 구독하게 함)
        self.scan_pub = self.create_publisher(LaserScan, '/scan_for_odom', 10)
        
        self.get_logger().info("Scan Locker Started: Publishing /scan_for_odom")

    def cmd_callback(self, msg):
        # 선속도나 회전속도가 있으면 움직이는 것으로 간주
        if abs(msg.linear.x) > self.stop_threshold or abs(msg.angular.z) > self.stop_threshold:
            self.is_moving = True
        else:
            self.is_moving = False

    def scan_callback(self, msg):
        if self.is_moving:
            # 움직일 때는 실시간 데이터를 그대로 넘김
            self.locked_scan = msg
            self.scan_pub.publish(msg)
        else:
            # 멈췄을 때는 '마지막 스캔'을 재활용
            if self.locked_scan is not None:
                # 단, 시간(Timestamp)은 현재 시간으로 갱신해줘야 TF 에러가 안 남!
                self.locked_scan.header.stamp = self.get_clock().now().to_msg()
                self.scan_pub.publish(self.locked_scan)
            else:
                # 초기 상태라 저장된 게 없으면 그냥 넘김
                self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanLocker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
