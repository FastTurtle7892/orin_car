import cv2
import time
from gesture_ai import MarshallerAI
from docking_ai import DockingAI

# --- 설정 ---
CAMERA_ID = 0
STATE = "MARSHAL"

def main():
    global STATE
    
    # 1. 카메라 열기
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("❌ Camera Open Failed! Check connection.")
        return
    
    # 해상도 설정 (640x480 권장)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Loading AI Models... (Please Wait)")
    try:
        marshal_ai = MarshallerAI()
        docking_ai = DockingAI()
    except Exception as e:
        print(f"❌ Model Init Failed: {e}")
        return

    print("\n=== System Started (Headless Mode) ===")
    print("📺 No GUI window will open.")
    print("⌨️  Press Ctrl+C to Quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret: 
                print("❌ Frame read error")
                break

            # 2. 모드별 동작 수행
            if STATE == "MARSHAL":
                cmd, _ = marshal_ai.detect_gesture(frame)
                
                # 로그: 의미 있는 명령만 출력 (SKIP, IDLE 제외)
                if cmd not in ["SKIP", "IDLE", "READY", "WAITING", "STAGE_1", "STAGE_2", "STAGE_3", "STAGE_4", "MANUAL"]:
                    print(f"\r[MARSHAL] CMD: {cmd}          ", end="")
                
                # [자동 전환] 도킹 명령 수신 시
                if cmd == "DOCKING":
                    print("\n🚀 Auto-Switching to DOCKING Mode!")
                    STATE = "DOCKING"

            elif STATE == "DOCKING":
                data, _ = docking_ai.process(frame)
                
                if data["found"]:
                    # 실시간 거리 출력
                    print(f"\r[DOCKING] ID:{data['id']} Dist:{data['dist_cm']:.1f}cm   ", end="")
                    if data["dist_cm"] < 10.0:
                        print("\n✅ DOCKING COMPLETE!")
                else:
                    # 마커 찾는 중... (너무 자주 출력되면 주석 처리)
                    # print(f"\r[DOCKING] Searching...", end="")
                    pass

            elif STATE == "CAMERA":
                # 아무 연산도 안 함 (CPU 쿨링용)
                pass

            # 중요: cv2.imshow 없음 -> Qt 에러 안 남
            # 종료는 터미널에서 Ctrl+C

    except KeyboardInterrupt:
        print("\n🛑 Stopping System...")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()