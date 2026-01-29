#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time
import struct
import serial

# ==========================================
# 1. STM32 통신 프로토콜
# ==========================================
PKT_MAGIC0 = 0xAA
PKT_MAGIC1 = 0x55
PKT_LEN    = 8

FLAG_ENABLE     = 1 << 0  # 0x01
FLAG_ESTOP      = 1 << 1  # 0x02
FLAG_DOCK_START = 1 << 2  # 0x04 (잡기)
FLAG_DOCK_ABORT = 1 << 3  # 0x08 (놓기)

def crc16_ibm(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def clamp_i8(x: int) -> int:
    return max(-128, min(127, int(x)))

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        # ---------------------------------------------------
        # 1. 하드웨어 스펙 설정 (여기가 핵심!)
        # ---------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyACM0') 
        self.declare_parameter('baudrate', 115200)
        
        # [물리적 스펙]
        self.declare_parameter('wheelbase', 0.145)       # 축간 거리 (m)
        self.declare_parameter('max_steering_deg', 40.0) # 로봇의 물리적 최대 꺾임각 (도)
        self.declare_parameter('max_speed_mps', 1.0)     # 1.0 m/s 일 때 STM 값 100 전송

        # [방향 반전] 필요시 True로 변경
        self.declare_parameter('invert_steering', False) 
        self.declare_parameter('invert_throttle', False)

        # 파라미터 로드
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer_deg = self.get_parameter('max_steering_deg').value
        self.max_speed_mps = self.get_parameter('max_speed_mps').value
        self.invert_steer = self.get_parameter('invert_steering').value
        self.invert_throttle = self.get_parameter('invert_throttle').value

        # ---------------------------------------------------
        # 2. UART 연결
        # ---------------------------------------------------
        self.ser = None
        self.seq = 0
        self.connect_serial()

        # ---------------------------------------------------
        # 3. 상태 변수
        # ---------------------------------------------------
        self.target_speed_val = 0  # STM으로 보낼 값 (-100 ~ 100)
        self.target_steer_val = 0  # STM으로 보낼 값 (-100 ~ 100)
        self.current_flags = FLAG_ENABLE # 기본 Enable

        # ---------------------------------------------------
        # 4. 토픽 구독
        # ---------------------------------------------------
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/gripper_cmd', self.gripper_callback, 10)

        # ---------------------------------------------------
        # 5. Heartbeat 타이머 (20Hz)
        # ---------------------------------------------------
        self.create_timer(0.05, self.send_packet_callback)

        self.get_logger().info(f"✅ Ackermann Driver Started!")
        self.get_logger().info(f"⚙️ Mapping: {self.max_steer_deg}°(Physical) -> 100(STM Value)")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"🔌 Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            self.ser = None

    def cmd_vel_callback(self, msg):
        """
        [매핑 공식 적용]
        입력: cmd_vel (속도 v, 회전 w)
        계산: 물리적 각도 (예: 20도)
        출력: STM 값 (예: 50) -> [ 40도일 때 100 ]
        """
        v = msg.linear.x
        w = msg.angular.z

        # 1. 아커만 기구학: 물리적 조향각(Degree) 계산
        if abs(v) < 0.01:
            # 정지 상태에서 회전 명령이 있으면 최대 조향
            if abs(w) > 0.01:
                calc_deg = self.max_steer_deg if w > 0 else -self.max_steer_deg
            else:
                calc_deg = 0.0
        else:
            steer_rad = math.atan((w * self.wheelbase) / v)
            calc_deg = math.degrees(steer_rad)

        # 2. 물리적 한계 자르기 (-40 ~ 40도)
        # 여기서 40보다 큰 값이 나와도 40으로 자릅니다.
        phys_deg = max(-self.max_steer_deg, min(self.max_steer_deg, calc_deg))

        # 3. STM 값으로 변환 (Mapping)
        # 공식: (현재각도 / 40도) * 100
        # 예: 40도 -> 1.0 * 100 = 100
        # 예: 20도 -> 0.5 * 100 = 50
        stm_steer_val = (phys_deg / self.max_steer_deg) * 100.0

        # 반전 처리
        if self.invert_steer:
            stm_steer_val = -stm_steer_val

        # 4. 속도 변환
        stm_speed_val = (v / self.max_speed_mps) * 100.0
        if self.invert_throttle:
            stm_speed_val = -stm_speed_val

        # 최종 저장 (int 형변환 및 -100~100 안전장치)
        self.target_steer_val = int(max(-100, min(100, stm_steer_val)))
        self.target_speed_val = int(max(-100, min(100, stm_speed_val)))

    def gripper_callback(self, msg):
        cmd = msg.data.upper()
        self.get_logger().info(f"🦾 Gripper: {cmd}")
        
        base = FLAG_ENABLE
        if cmd in ["GRIP", "DOWN"]:
            self.current_flags = base | FLAG_DOCK_START
        elif cmd in ["RELEASE", "UP"]:
            self.current_flags = base | FLAG_DOCK_ABORT
        elif cmd == "STOP":
            self.current_flags = base
        else:
            self.get_logger().warn(f"Unknown Cmd: {cmd}")

    def send_packet_callback(self):
        """ 20Hz 주기로 STM32에 값 전송 """
        if not self.ser or not self.ser.is_open:
            return

        try:
            # 패킷 조립
            header = struct.pack(
                "<BBBBbb",
                PKT_MAGIC0,
                PKT_MAGIC1,
                self.seq & 0xFF,
                self.current_flags & 0xFF,
                clamp_i8(self.target_speed_val),
                clamp_i8(self.target_steer_val)
            )
            crc = crc16_ibm(header)
            pkt = header + struct.pack("<H", crc)

            self.ser.write(pkt)
            self.seq = (self.seq + 1) & 0xFF

        except Exception as e:
            self.get_logger().error(f"Write Error: {e}")

    def stop_robot(self):
        if self.ser and self.ser.is_open:
            for _ in range(5):
                header = struct.pack("<BBBBbb", PKT_MAGIC0, PKT_MAGIC1, 0, 0, 0, 0)
                crc = crc16_ibm(header)
                pkt = header + struct.pack("<H", crc)
                self.ser.write(pkt)
                time.sleep(0.01)
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
