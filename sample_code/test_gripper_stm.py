#!/usr/bin/env python3
import struct
import time
import serial

# ==========================================
# 1. 설정 (잘 되는 코드 값 반영)
# ==========================================
SERIAL_PORT = "/dev/ttyACM0"   # 포트 확인!
BAUDRATE    = 115200
HZ          = 50.0             # [수정] 20Hz -> 50Hz (STM32 반응성 향상)

# ==========================================
# 2. 프로토콜 정의
# ==========================================
PKT_MAGIC0 = 0xAA
PKT_MAGIC1 = 0x55
PKT_LEN    = 8

FLAG_ENABLE     = 1 << 0  # 0x01
FLAG_ESTOP      = 1 << 1  # 0x02
FLAG_DOCK_START = 1 << 2  # 0x04
FLAG_DOCK_ABORT = 1 << 3  # 0x08

# ==========================================
# 3. 패킷 생성 함수
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
    # <BBBBbb 구조 (Magic0, Magic1, Seq, Flags, Speed, Steer)
    header = struct.pack("<BBBBbb", PKT_MAGIC0, PKT_MAGIC1, seq & 0xFF, flags & 0xFF, speed, steer)
    crc = crc16_ibm(header)
    return header + struct.pack("<H", crc)

# ==========================================
# 4. 명령 전송 헬퍼
# ==========================================
def send_command(ser, start_seq, flags, duration_sec):
    """
    지정된 플래그(flags)를 duration_sec 동안 50Hz로 전송
    """
    target_count = int(duration_sec * HZ)
    period = 1.0 / HZ
    seq = start_seq

    for _ in range(target_count):
        # 도킹 테스트 중이므로 Speed=0, Steer=0
        pkt = build_packet(seq, flags, 0, 0)
        ser.write(pkt)
        
        # [추가] 혹시 모를 RX 버퍼 오버플로우 방지 (데이터 읽어서 버림)
        if ser.in_waiting > 0:
            try:
                ser.read(ser.in_waiting)
            except:
                pass

        seq = (seq + 1) & 0xFF
        time.sleep(period)
    
    return seq

# ==========================================
# 5. 메인 실행 로직
# ==========================================
def main():
    print(f"🔌 Opening Serial: {SERIAL_PORT} @ {BAUDRATE}")
    try:
        # [수정] timeout 0.1 -> 0.02 (잘 되는 코드와 동일하게)
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.02)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        return

    print("✅ Serial Connected! (50Hz Mode)")
    print("🚀 도킹 시스템 테스트 (Flag 기반)")
    
    input("👉 엔터를 누르면 테스트를 시작합니다...")

    seq = 0
    
    try:
        # 1. 도킹 시작 (FLAG_DOCK_START)
        print("\n[Step 1] 🚩 DOCK_START 신호 전송! (5초)")
        print("   -> (예상) 팔 내림 -> 잡음 -> 팔 올림")
        current_flags = FLAG_ENABLE | FLAG_DOCK_START
        seq = send_command(ser, seq, current_flags, 5.0)

        # 2. 대기 (FLAG_ENABLE만 전송)
        print("\n[Step 2] 대기 (3초)...")
        seq = send_command(ser, seq, FLAG_ENABLE, 5.0)

        # 3. 도킹 해제 (FLAG_DOCK_ABORT)
        print("\n[Step 3] 🚩 DOCK_ABORT 신호 전송! (5초)")
        print("   -> (예상) 팔 내림 -> 놓음 -> 팔 올림")
        current_flags = FLAG_ENABLE | FLAG_DOCK_ABORT
        seq = send_command(ser, seq, current_flags, 5.0)

        print("\n✅ 테스트 완료.")

    except KeyboardInterrupt:
        print("\n🚫 강제 종료됨.")

    finally:
        if ser.is_open:
            print("🛑 Stopping...")
            # 종료 시 안전하게 Disable(0) 전송
            for _ in range(10):
                pkt = build_packet(seq, 0, 0, 0)
                ser.write(pkt)
                time.sleep(0.02)
            ser.close()

if __name__ == "__main__":
    main()
