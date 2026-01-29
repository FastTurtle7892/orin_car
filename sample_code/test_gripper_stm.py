#!/usr/bin/env python3
import struct
import time
import serial

# ==========================================
# 1. 설정 (환경에 맞게 수정)
# ==========================================
SERIAL_PORT = "/dev/ttyACM0"   # 포트 확인! (ls /dev/tty*)
BAUDRATE    = 115200
HZ          = 20.0             # 0.05초 간격 전송

# ==========================================
# 2. 프로토콜 정의 (send_uart_pkt.py 참고)
# ==========================================
PKT_MAGIC0 = 0xAA
PKT_MAGIC1 = 0x55
PKT_LEN    = 8

# [핵심] STM32가 동작을 미리 약속해둔 플래그들
FLAG_ENABLE     = 1 << 0  # 0x01
FLAG_ESTOP      = 1 << 1  # 0x02
FLAG_DOCK_START = 1 << 2  # 0x04 (자동 시퀀스: 내리고->잡고->올리고)
FLAG_DOCK_ABORT = 1 << 3  # 0x08 (자동 시퀀스: 내리고->놓고->올리고)

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
    # send_uart_pkt.py와 동일한 패킷 구조
    header = struct.pack("<BBBBbb", PKT_MAGIC0, PKT_MAGIC1, seq & 0xFF, flags & 0xFF, speed, steer)
    crc = crc16_ibm(header)
    return header + struct.pack("<H", crc)

# ==========================================
# 4. 명령 전송 헬퍼 (지속 전송)
# ==========================================
def send_command(ser, start_seq, flags, duration_sec):
    """
    지정된 플래그(flags)를 duration_sec 동안 계속 전송합니다.
    STM32가 명령을 놓치지 않고 수행하도록 20Hz로 계속 쏴줍니다.
    """
    target_count = int(duration_sec * HZ)
    period = 1.0 / HZ
    seq = start_seq

    for _ in range(target_count):
        # 도킹 동작 중에는 로봇이 움직이면 안 되므로 Speed=0, Steer=0
        pkt = build_packet(seq, flags, 0, 0)
        ser.write(pkt)
        
        seq = (seq + 1) & 0xFF
        time.sleep(period)
    
    return seq

# ==========================================
# 5. 메인 실행 로직
# ==========================================
def main():
    print(f"🔌 Opening Serial: {SERIAL_PORT} @ {BAUDRATE}")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    except Exception as e:
        print(f"❌ 포트 열기 실패: {e}")
        return

    print("✅ Serial Connected!")
    print("🚀 도킹 시스템 테스트 (Flag 기반)")
    print("   STM32에 '도킹 시작/해제' 신호만 보냅니다.")
    print("   (세부 동작은 STM32가 자동으로 수행합니다)")
    
    input("👉 엔터를 누르면 테스트를 시작합니다...")

    seq = 0
    
    try:
        # 1. 도킹 시작 신호 전송 (FLAG_DOCK_START)
        # 예상 동작: 팔 내림 -> 잡음 -> 팔 올림
        print("\n[Step 1] 🚩 DOCK_START 신호 전송! (잡기)")
        print("   -> STM32가 '잡기 시퀀스'를 실행합니다...")
        
        # ENABLE과 DOCK_START를 같이 보냄
        current_flags = FLAG_ENABLE | FLAG_DOCK_START
        seq = send_command(ser, seq, current_flags, 5.0)

        
        # 2. 대기 (상태 유지)
        print("\n[Step 2] 3초 대기...")
        # 동작이 끝난 후에는 ENABLE만 유지 (플래그 끔)
        seq = send_command(ser, seq, FLAG_ENABLE, 3.0)


        # 3. 도킹 해제 신호 전송 (FLAG_DOCK_ABORT)
        # 예상 동작: 팔 내림 -> 놓음 -> 팔 올림 (복귀)
        print("\n[Step 3] 🚩 DOCK_ABORT 신호 전송! (놓기)")
        print("   -> STM32가 '놓기 시퀀스'를 실행합니다...")
        
        # ENABLE과 DOCK_ABORT를 같이 보냄
        current_flags = FLAG_ENABLE | FLAG_DOCK_ABORT
        seq = send_command(ser, seq, current_flags, 5.0)

        print("\n✅ 테스트 완료.")

    except KeyboardInterrupt:
        print("\n🚫 강제 종료됨.")

    finally:
        if ser.is_open:
            print("🛑 Stopping...")
            # 종료 시 안전하게 모든 플래그 끄고(0) 종료
            for _ in range(5):
                pkt = build_packet(seq, 0, 0, 0)
                ser.write(pkt)
                time.sleep(0.01)
            ser.close()

if __name__ == "__main__":
    main()
