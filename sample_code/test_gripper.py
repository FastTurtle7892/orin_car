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
    
    # 안전을 위해 처음엔 0도 등으로 튀지 않게 초기화 시도
    # kit.servo[LIFT_CHANNEL].angle = LIFT_DOWN
    # kit.servo[GRIPPER_CHANNEL].angle = GRIP_OPEN
	
    kit.servo[LIFT_CHANNEL].angle = 180
    time.sleep(3.0)

    kit.servo[GRIPPER_CHANNEL].angle = 50
    time.sleep(3.0)  

    # 종료 시 안전하게 내려놓고 벌림
    print("\n🏁 Test Finished. Resetting position...")
    kit.servo[LIFT_CHANNEL].angle = 170
    time.sleep(0.5)
    kit.servo[GRIPPER_CHANNEL].angle = 100
    time.sleep(0.5)

    kit.servo[LIFT_CHANNEL].angle = 180
    time.sleep(0.5)
    kit.servo[GRIPPER_CHANNEL].angle = 50

    print("Done.")

if __name__ == "__main__":
    test_gripper()
