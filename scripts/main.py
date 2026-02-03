import cv2
import time
from gesture_ai import MarshallerAI
from docking_ai import DockingAI

# --- 설정 ---
CAMERA_ID = 0  # Jetson 연결된 카메라
STATE = "MARSHAL" # 초기 상태: MARSHAL, DOCKING, CAMERA

def main():
    global STATE
    
    # 1. 카메라 열기
    cap = cv2.VideoCapture(CAMERA_ID)
    if not cap.isOpened():
        print("Camera Open Failed!")
        return
    
    # 해상도 설정 (ArUco 1.1cm 인식을 위해 너무 낮추지 않음)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 2. AI 모듈 초기화
    print("Loading AI Models... (Please Wait)")
    marshal_ai = MarshallerAI() # TensorRT Engine 로드
    docking_ai = DockingAI()

    print("\n=== System Started ===")
    print("Press 'm' for MARSHAL Mode (Gesture)")
    print("Press 'd' for DOCKING Mode (ArUco)")
    print("Press 'c' for CAMERA Mode (No AI)")
    print("Press 'q' to Quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed")
            break

        # 디버그용 프레임 복사 (원본 보존)
        debug_frame = frame.copy()

        # --- 상태 머신 (State Machine) ---
        if STATE == "MARSHAL":
            # [모드 1] 제스처 인식 (YOLO)
            cmd, debug_frame = marshal_ai.detect_gesture(frame)
            
            # [자동 전환] 매뉴얼 모드에서 STOP 5초 유지 시 도킹 모드로 자동 전환
            if cmd == "DOCKING":
                print("🚀 Auto-Switching to DOCKING Mode!")
                STATE = "DOCKING"

            # cmd를 ROS나 시리얼로 전송하는 로직 위치
            # ex) publisher.publish(cmd)
            
            cv2.putText(debug_frame, f"[MODE: MARSHAL] CMD: {cmd}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        elif STATE == "DOCKING":
            # [모드 2] 도킹 (ArUco)
            data, debug_frame = docking_ai.process(frame)
            
            status_text = "FOUND" if data["found"] else "SEARCHING"
            color = (0, 255, 0) if data["found"] else (0, 0, 255)
            
            # 도킹 완료 조건 예시
            if data["found"] and data["dist_cm"] < 10.0:
                 cv2.putText(debug_frame, "DOCKING COMPLETE!", (150, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

            cv2.putText(debug_frame, f"[MODE: DOCKING] {status_text}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        elif STATE == "CAMERA":
            # [모드 3] 단순 카메라 (CPU 부하 최소화)
            # 아무런 연산도 하지 않고 화면만 보여줌
            cv2.putText(debug_frame, "[MODE: CAMERA] (No AI)", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 화면 출력
        cv2.imshow("TowCar AI View", debug_frame)

        # --- 키 입력 처리 ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('m'):
            STATE = "MARSHAL"
            print("Switched to MARSHAL Mode")
        elif key == ord('d'):
            STATE = "DOCKING"
            print("Switched to DOCKING Mode")
        elif key == ord('c'):
            STATE = "CAMERA"
            print("Switched to CAMERA Mode (CPU Save)")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()