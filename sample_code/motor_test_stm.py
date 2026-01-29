#!/usr/bin/env python3
import struct
import time
import serial

# ==========================================
# 1. 설정 (STM32 연결 정보)
# ==========================================
SERIAL_PORT = "/dev/ttyACM0"   # 포트 확인!
BAUDRATE    = 115200
HZ          = 20.0             # 초당 20번 전송

# 프로토콜 정의
PKT_MAGIC0 = 0xAA
PKT_MAGIC1 = 0x55
FLAG_ENABLE = 1 << 0  # 이 플래그가 있어야 모터가 돕니다.

# ==========================================
# 2. 프로토콜 헬퍼 함수 (그대로 사용)
# ==========================================
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
    if x < -128: return -128
    if x > 127:  return 127
    return x

def build_packet(seq: int, flags: int, speed: int, steer: int) -> bytes:
    speed = clamp_i8(speed)
    steer = clamp_i8(steer)
    header = struct.pack("<BBBBbb", PKT_MAGIC0, PKT_MAGIC1, seq & 0xFF, flags & 0xFF, speed, steer)
    crc = crc16_ibm(header)
    return header + struct.pack("<H", crc)

# ==========================================
# 3. 핵심: 지정된 시간 동안 계속 명령 보내기
# ==========================================
def drive_motor(ser, start_seq, speed, duration_sec):
    """
    STM32는 안전을 위해 계속 신호를 줘야 합니다.
    duration_sec 동안 speed 명령을 20Hz로 계속 전송합니다.
    """
    packet_count = int(duration_sec * HZ)
    period = 1.0 / HZ
    seq = start_seq

    for _ in range(packet_count):
        # 조향(Steer)은 0으로 고정, 속도(Speed)만 제어
        pkt = build_packet(seq, FLAG_ENABLE, speed, 0)
        ser.write(pkt)
        
        seq = (seq + 1) & 0xFF
        time.sleep(period)
    
    return seq # 다음 시퀀스 번호 반환

# ==========================================
# 4. 메인 로직
# ==========================================
def main():
    print(f"🔌 Opening Serial: {SERIAL_PORT} @ {BAUDRATE}")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        return

    print("✅ Serial Connected!")
    print("🚀 모터 테스트 시작 (Ctrl+C로 종료)")
    
    seq = 0
    
    try:
        while True:
            # 1. 전진 (Forward) - 50% 속도
            print(f"Define: Motor Forward (Speed: 50, 5sec)")
            seq = drive_motor(ser, seq, 50, 5.0)

            # 2. 후진 (Backward) - 50% 속도
            # (STM32에서는 음수 값이 후진입니다)
            print(f"Define: Motor Backward (Speed: -50, 5sec)")
            seq = drive_motor(ser, seq, -50, 5.0)

            # 3. 정지 (Stop)
            print(f"Define: Motor Stop (Speed: 0, 2sec)")
            seq = drive_motor(ser, seq, 0, 2.0)

    except KeyboardInterrupt:
        print("\n🚫 강제 종료됨.")

    finally:
        if ser.is_open:
            print("🛑 Stopping Motor...")
            # 종료 시 안전하게 0 전송 (Flag 0 = Disable)
            for _ in range(5):
                pkt = build_packet(seq, 0, 0, 0)
                ser.write(pkt)
                time.sleep(0.01)
            ser.close()

if __name__ == "__main__":
    main()
