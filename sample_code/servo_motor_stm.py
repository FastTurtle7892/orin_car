#!/usr/bin/env python3
import struct
import time
import serial

# ==========================================
# 1. 설정 (User Config)
# ==========================================
SERIAL_PORT = "/dev/ttyACM0"   # 포트 이름 확인!
BAUDRATE    = 115200
HZ          = 20.0 

# [각도 제한] 
# 원본은 30~145도였지만, STM은 -100~100 퍼센트 제어입니다.
# 안전을 위해 50 정도로 설정합니다. (원하시면 100까지 늘려도 됩니다)
MAX_TEST_ANGLE = 100  

PKT_MAGIC0 = 0xAA
PKT_MAGIC1 = 0x55
FLAG_ENABLE = 1 << 0 

# ==========================================
# 2. 헬퍼 함수
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
# 3. 메인 로직 (원본 동작 재현)
# ==========================================
def main():
    print(f"🔌 Opening Serial: {SERIAL_PORT} @ {BAUDRATE}")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        return

    seq = 0
    period = 1.0 / HZ

    try:
        # [초기화] 중앙 정렬 (Steer 0)
        # STM32가 연결되자마자 모터 힘을 받게 하기 위해 몇 번 보내줍니다.
        print("✅ 초기화: 바퀴를 중앙(0)으로 정렬합니다.")
        for _ in range(10):
            pkt = build_packet(seq, FLAG_ENABLE, 0, 0)
            ser.write(pkt)
            seq = (seq + 1) & 0xFF
            time.sleep(0.01)
        
        # [대기] 원본 코드의 input() 기능
        input("👉 엔터 키를 누르면 Sweep 테스트를 시작합니다...")

        print(f"🚀 테스트 시작 (범위: -{MAX_TEST_ANGLE} ~ +{MAX_TEST_ANGLE})")

        # ---------------------------------------------------------
        # [동작 1] 오른쪽 끝(-MAX)에서 왼쪽 끝(+MAX)으로 이동
        # 원본: for i in range(30, 145)
        # ---------------------------------------------------------
        print(f"➡️ Right(-{MAX_TEST_ANGLE}) -> Left(+{MAX_TEST_ANGLE})")
        
        # 시작 전 안전하게 오른쪽 끝으로 먼저 이동
        for _ in range(5):
             pkt = build_packet(seq, FLAG_ENABLE, 0, -MAX_TEST_ANGLE)
             ser.write(pkt)
             time.sleep(0.01)

        # 천천히 스윕
        for s in range(-MAX_TEST_ANGLE, MAX_TEST_ANGLE + 1, 2): 
            pkt = build_packet(seq, FLAG_ENABLE, 0, s)
            ser.write(pkt)
            seq = (seq + 1) & 0xFF
            time.sleep(0.05) # 원본 속도 (0.05s)
            print(f"Steer: {s}  ", end='\r')
        print()

        # ---------------------------------------------------------
        # [동작 2] 왼쪽 끝(+MAX)에서 오른쪽 끝(-MAX)으로 이동
        # 원본: for i in range(145, 30, -1)
        # ---------------------------------------------------------
        print(f"⬅️ Left(+{MAX_TEST_ANGLE}) -> Right(-{MAX_TEST_ANGLE})")
        
        for s in range(MAX_TEST_ANGLE, -MAX_TEST_ANGLE - 1, -2): 
            pkt = build_packet(seq, FLAG_ENABLE, 0, s)
            ser.write(pkt)
            seq = (seq + 1) & 0xFF
            time.sleep(0.05) # 원본 속도 (0.05s)
            print(f"Steer: {s}  ", end='\r')
        print()

        print("✅ Test Completed.")

    except KeyboardInterrupt:
        print("\n🚫 강제 종료됨.")

    finally:
        if ser.is_open:
            print("🛑 Stopping Motor...")
            # 종료 시 중앙으로 복귀하고 모터 끔
            for _ in range(10):
                pkt = build_packet(seq, 0, 0, 0) # Flag 0 (Disable)
                ser.write(pkt)
                time.sleep(0.01)
            ser.close()

if __name__ == "__main__":
    main()
