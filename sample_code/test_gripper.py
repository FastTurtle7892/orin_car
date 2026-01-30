import time
from adafruit_servokit import ServoKit
import board
import busio

# ==========================================
# [설정] 채널 및 각도
# ==========================================
LIFT_CHANNEL = 1      # 리프트 (팔)
GRIPPER_CHANNEL = 2   # 그리퍼 (집게)
    

def test_gripper():
    print("🔌 Connecting to Servo Driver (Address: 0x60)...")
    try:
        # I2C 초기화
        i2c_bus0 = busio.I2C(board.SCL, board.SDA)
        
        # [수정] address=0x60 추가 (이게 핵심입니다!)
        kit = ServoKit(channels=16, i2c=i2c_bus0, address=0x60)
        
        print("✅ Connected!")
    except Exception as e:
        print(f"❌ Connection Failed: {e}")
        return

    print("\n🚀 Starting Gripper Test Sequence (3 Loops)")
    
	
    # 초기 상태
    kit.servo[LIFT_CHANNEL].angle = 70
    time.sleep(3.0)
    kit.servo[GRIPPER_CHANNEL].angle = 120
    time.sleep(3.0)
    print("초기 상태")

    # 리프트 내리고 그리퍼로 잡고 리프트 올리기
    kit.servo[LIFT_CHANNEL].angle = 90
    time.sleep(1.0)
    kit.servo[GRIPPER_CHANNEL].angle = 50
    time.sleep(1.0)
    kit.servo[LIFT_CHANNEL].angle = 70
    time.sleep(1.0)
    print("grip 완료")

    # 리프트 내리고 그리퍼 풀고 리프트 올리기
    kit.servo[LIFT_CHANNEL].angle = 90
    time.sleep(1.0)
    kit.servo[GRIPPER_CHANNEL].angle = 120
    time.sleep(1.0)
    kit.servo[LIFT_CHANNEL].angle = 70
    time.sleep(1.0)
    print("놓기 헤제")

    print("Done.")

if __name__ == "__main__":
    test_gripper()
