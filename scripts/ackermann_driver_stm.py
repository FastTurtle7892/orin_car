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
# 1. STM32 프로토콜 정의 (motor_test_stm.py 기반)
# ==========================================
PKT_MAGIC0 = 0xAA
PKT_MAGIC1 = 0x55
PKT_LEN    = 8

FLAG_ENABLE     = 1 << 0  # 0x01 (필수: 이 비트가 1이어야 모터가 돔)
FLAG_ESTOP      = 1 << 1  # 0x02
FLAG_DOCK_START = 1 << 2  # 0x04
FLAG_DOCK_ABORT = 1 << 3  # 0x08

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
    # int 변환 및 범위 제한
    val = int(x)
    return max(-128, min(127, val))

def build_packet(seq: int, flags: int, speed: int, steer: int) -> bytes:
    """ motor_test_stm.py에서 검증된 패킷 생성 함수 """
    speed = clamp_i8(speed)
    steer = clamp_i8(steer)
    # <BBBBbb : Magic0, Magic1, Seq, Flags, Speed(s8), Steer(s8)
    header = struct.pack("<BBBBbb", PKT_MAGIC0, PKT_MAGIC1, seq & 0xFF, flags & 0xFF, speed, steer)
    crc = crc16_ibm(header)
    return header + struct.pack("<H", crc)

class AckermannDriver(Node):
    def __init__(self):
        super().__init__('ackermann_driver')

        # ---------------------------------------------------
        # 2. 파라미터 설정
        # ---------------------------------------------------
        self.declare_parameter('serial_port', '/dev/ttyACM0') 
        self.declare_parameter('baudrate', 115200)
        
        self.declare_parameter('wheelbase', 0.145)       
        self.declare_parameter('max_steering_deg', 40.0) 
        self.declare_parameter('max_speed_mps', 1.0)     

        self.declare_parameter('invert_steering', False) 
        self.declare_parameter('invert_throttle', False)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer_deg = self.get_parameter('max_steering_deg').value
        self.max_speed_mps = self.get_parameter('max_speed_mps').value
        self.invert_steer = self.get_parameter('invert_steering').value
        self.invert_throttle = self.get_parameter('invert_throttle').value

        # ---------------------------------------------------
        # 3. UART 연결
        # ---------------------------------------------------
        self.ser = None
        self.seq = 0
        self.connect_serial()

        # ---------------------------------------------------
        # 4. 상태 변수
        # ---------------------------------------------------
        self.target_speed_val = 0
        self.target_steer_val = 0
        self.current_flags = FLAG_ENABLE  # 초기값: Enable (1)
        self.last_cmd_time = self.get_clock().now()

        # ---------------------------------------------------
        # 5. 토픽 구독 (절대 경로 /cmd_vel 사용)
        # ---------------------------------------------------
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/gripper_cmd', self.gripper_callback, 10)

        # ---------------------------------------------------
        # 6. 전송 타이머 (20Hz)
        # ---------------------------------------------------
        self.create_timer(0.05, self.send_packet_callback)
        
        # 디버그용 타이머 (1초마다 상태 출력)
        self.create_timer(1.0, self.debug_callback)

        self.get_logger().info(f"✅ Ackermann Driver Ready on {self.port}")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            # 버퍼 비우기 (중요)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"🔌 Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")
            self.ser = None

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        
        v = msg.linear.x
        w = msg.angular.z

        # [계산 로직]
        if abs(v) < 0.01:
            if abs(w) > 0.01:
                calc_deg = self.max_steer_deg if w > 0 else -self.max_steer_deg
            else:
                calc_deg = 0.0
        else:
            steer_rad = math.atan((w * self.wheelbase) / v)
            calc_deg = math.degrees(steer_rad)

        phys_deg = max(-self.max_steer_deg, min(self.max_steer_deg, calc_deg))

        # STM 값 변환
        stm_steer_val = (phys_deg / self.max_steer_deg) * 100.0
        stm_speed_val = (v / self.max_speed_mps) * 100.0

        if self.invert_steer: stm_steer_val = -stm_steer_val
        if self.invert_throttle: stm_speed_val = -stm_speed_val

        self.target_steer_val = int(stm_steer_val)
        self.target_speed_val = int(stm_speed_val)

        # 수신 로그 (너무 자주 뜨면 주석 처리)
        # self.get_logger().info(f"📩 CMD Recv: v={v:.2f}, STM_Spd={self.target_speed_val}")

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

    def send_packet_callback(self):
        """ 20Hz 주기로 STM32에 패킷 전송 """
        if not self.ser or not self.ser.is_open:
            return

        # [안전장치] 0.5초 이상 cmd_vel 없으면 정지
        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_diff > 0.5:
            self.target_speed_val = 0
            self.target_steer_val = 0

        try:
            # build_packet 함수 사용 (검증된 방식)
            pkt = build_packet(
                self.seq, 
                self.current_flags, 
                self.target_speed_val, 
                self.target_steer_val
            )
            
            self.ser.write(pkt)
            self.seq = (self.seq + 1) & 0xFF

        except Exception as e:
            self.get_logger().error(f"Write Error: {e}")
            self.connect_serial() # 재연결 시도

    def debug_callback(self):
        """ 1초마다 현재 전송중인 값을 로그로 출력 """
        if self.ser and self.ser.is_open:
            self.get_logger().info(
                f"📤 Serial TX | Speed: {self.target_speed_val:3d} | "
                f"Steer: {self.target_steer_val:3d} | "
                f"Flag: {self.current_flags} | Seq: {self.seq}"
            )
        else:
            self.get_logger().warn("⚠️ Serial NOT Connected")

    def stop_robot(self):
        if self.ser and self.ser.is_open:
            # 종료 시 Flag 0(Disable) 및 속도 0 전송
            pkt = build_packet(0, 0, 0, 0)
            for _ in range(5):
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
