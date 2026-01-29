import cv2
import numpy as np
from ultralytics import YOLO
import os

class MarshallerAI:
    def __init__(self, model_path='yolov8n-pose.pt'):
        # 절대 경로 처리
        if not os.path.exists(model_path):
            print(f"Warning: Model not found at {model_path}. Trying local path.")
            model_path = 'yolov8n-pose.pt'
            
        self.model = YOLO(model_path) 
        self.status = "IDLE"

    def calculate_angle(self, a, b, c):
        a, b, c = np.array(a), np.array(b), np.array(c)
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        if angle > 180.0: angle = 360-angle
        return angle

    def detect_gesture(self, frame):
        h, w, _ = frame.shape
        results = self.model(frame, verbose=False, conf=0.5)
        
        current_action = "IDLE"
        
        if results[0].keypoints is None or len(results[0].keypoints.data) == 0:
            return "IDLE", frame

        kpts = results[0].keypoints.data[0].cpu().numpy()
        
        # 키포인트 정규화 좌표 함수
        def get_norm(idx): return [kpts[idx][0]/w, kpts[idx][1]/h]

        # 주요 관절 (어깨, 팔꿈치, 손목)
        l_sh, r_sh = get_norm(5), get_norm(6)
        l_el, r_el = get_norm(7), get_norm(8)
        l_wr, r_wr = get_norm(9), get_norm(10)
        
        # 신뢰도 체크 (어깨가 안 보이면 무시)
        if kpts[5][2] < 0.5 or kpts[6][2] < 0.5:
             return "IDLE", frame

        angle_l = self.calculate_angle(l_sh, l_el, l_wr)
        angle_r = self.calculate_angle(r_sh, r_el, r_wr)
        
        wrist_dist_x = abs(l_wr[0] - r_wr[0])
        
        # [1] STOP (X자 교차 or 머리 위)
        if r_wr[0] > l_wr[0] or (l_wr[1] < l_sh[1] and r_wr[1] < r_sh[1] and wrist_dist_x < 0.1):
            current_action = "STOP"

        # [2] FORWARD (팔을 접어 신호)
        elif angle_l < 125 and angle_r < 125 and l_wr[1] < l_el[1] and r_wr[1] < r_el[1]:
            current_action = "FORWARD"
            
        # [3] APPROACHING (팔을 벌림)
        elif angle_l > 130 and angle_r > 130:
            current_action = "APPROACHING"

        # [4] ENGINE_CUT (목 긋기 등 - 필요시 추가)
        
        # 시각화
        annotated_frame = results[0].plot()
        cv2.putText(annotated_frame, current_action, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return current_action, annotated_frame
