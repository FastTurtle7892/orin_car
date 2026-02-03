import cv2
import numpy as np
import os
from ultralytics import YOLO

class MarshallerAI:
    def __init__(self):
        print("Initializing MarshallerAI...")

        # [핵심] 사용자가 확인해준 절대 경로로 고정
        abs_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.engine'
        pt_path = '/home/jetson/ros_ws/src/orin_car/config/yolov8n-pose.pt'

        print(f"🔍 Looking for engine at: {abs_path}")

        try:
            # device=0 (GPU), task='pose'
            self.model = YOLO(abs_path, task='pose')
            print("✅ Loaded TensorRT Engine (Fast)")
        except Exception as e:
            print(f"⚠️ Engine load failed: {e}")
            print(f"⚠️ Loading .pt model from: {pt_path}")
            self.model = YOLO(pt_path)

        # 변수 초기화
        self.stage = 0
        self.is_finished = False
        self.trigger_counter = 0
        self.triggered_lock = False
        self.stop_timer = 0
        self.frame_count = 0
        self.SKIP_FRAMES = 2 # 3프레임 중 1번만 추론 (최적화)
        
        self.LIMIT_NORMAL = 10
        self.LIMIT_RESET = 20
        self.LIMIT_DOCKING = 50

    def calculate_angle(self, a, b, c):
        a, b, c = np.array(a), np.array(b), np.array(c)
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        if angle > 180.0: angle = 360-angle
        return angle

    # Headless라 그리기는 생략 (CPU 절약)
    def draw_custom_skeleton(self, frame, kpts): pass
    def draw_status(self, frame, action, info, color): pass
    def draw_loading_bar(self, frame, progress, text=""): pass

    def detect_gesture(self, frame):
        self.frame_count += 1
        # 프레임 스킵 로직
        if self.frame_count % (self.SKIP_FRAMES + 1) != 0:
            return "SKIP", frame

        h, w, _ = frame.shape
        # [최적화] FP16(half=True) 적용
        results = self.model(frame, verbose=False, conf=0.5, device=0, half=True)
        
        if results[0].keypoints is None or len(results[0].keypoints.data) == 0:
            return "IDLE", frame

        kpts = results[0].keypoints.data[0].cpu().numpy()
        def get_n(i): return [kpts[i][0]/w, kpts[i][1]/h]

        l_sh, r_sh = get_n(5), get_n(6)
        l_el, r_el = get_n(7), get_n(8)
        l_wr, r_wr = get_n(9), get_n(10)
        l_hip, r_hip = get_n(11), get_n(12) 
        
        wrist_dist_x = abs(l_wr[0] - r_wr[0])
        shoulder_width = abs(l_sh[0] - r_sh[0])
        angle_l = self.calculate_angle(l_sh, l_el, l_wr)
        angle_r = self.calculate_angle(r_sh, r_el, r_wr)

        action = "READY"
        trigger = False
        global_act = False

        # 1. RESET (열중쉬어)
        has_arms = (kpts[9][2] > 0.6) and (kpts[10][2] > 0.6)
        if has_arms:
            l_waist = (l_wr[1] < l_hip[1]+0.05) and (l_wr[1] > l_sh[1]+0.2)
            r_waist = (r_wr[1] < r_hip[1]+0.05) and (r_wr[1] > r_sh[1]+0.2)
            bent = (angle_l < 140) and (angle_r < 140)
            if l_waist and r_waist and bent:
                action = "RESET"; trigger = True; global_act = True

        # 2. STOP (X자)
        if not global_act:
            if (r_wr[0] > l_wr[0]) and (l_wr[1] < l_sh[1] + 0.4):
                if self.stage in [2, 10] or self.is_finished:
                    action = "STOP"
                    if self.stage == 2: trigger = True
                    global_act = True

        # 3. Stage Logic
        if not self.is_finished and not global_act:
            if self.stage == 0:
                # Manual Mode Check (Y자)
                if (l_wr[1] < l_sh[1]) and (r_wr[1] < r_sh[1]) and (wrist_dist_x > shoulder_width * 1.5):
                    trigger = True; action = "MANUAL_MODE"
                # Auto Mode Check (차렷)
                elif (l_wr[1] > l_sh[1]+0.3) and (r_wr[1] > r_sh[1]+0.3) and (wrist_dist_x < shoulder_width*1.25):
                    trigger = True; action = "READY"
            
            elif self.stage == 10: # Manual Drive
                if action == "STOP":
                    self.stop_timer += 1
                    if self.stop_timer >= self.LIMIT_DOCKING: return "DOCKING", frame
                    return "STOP", frame
                self.stop_timer = 0
                if l_wr[1] < l_sh[1] and r_wr[1] < r_sh[1]: action = "FORWARD"
                elif l_wr[1] > l_sh[1] and r_wr[1] > r_sh[1]: action = "BACKWARD"
                elif l_wr[1] < l_sh[1]: action = "LEFT"
                elif r_wr[1] < r_sh[1]: action = "RIGHT"
                else: action = "MANUAL"

            elif self.stage == 1:
                is_fast = (l_wr[1]-l_sh[1]>0.25 and r_wr[1]-r_sh[1]>0.25 and wrist_dist_x > shoulder_width*1.4)
                if is_fast: trigger=True; action="APPROACHING"
                elif l_wr[1]<l_el[1]: action="FORWARD"
                elif l_wr[1]<l_el[1]: action="TURN_LEFT"
                else: action="STAGE_1"

            elif self.stage == 2:
                 if abs(l_wr[1]-l_sh[1])<0.25: action="APPROACHING"
                 else: action="STAGE_2"
            
            elif self.stage == 3:
                is_hold = (l_wr[1]>l_sh[1] and l_wr[1]<l_hip[1] and wrist_dist_x < shoulder_width*0.6)
                if is_hold: trigger=True; action="GRIPPER_HOLD"
                else: action="STAGE_3"

            elif self.stage == 4:
                is_rel = (l_wr[1]>l_sh[1] and wrist_dist_x > shoulder_width*0.8)
                if is_rel: trigger=True; action="GRIPPER_RELEASE"
                else: action="STAGE_4"

        # Trigger Handle
        target = self.LIMIT_RESET if action == "RESET" else self.LIMIT_NORMAL
        if trigger:
            if not self.triggered_lock:
                self.trigger_counter += 1
                if self.trigger_counter >= target:
                    self.triggered_lock = True
                    if action == "RESET": self.stage = 0; self.is_finished = False
                    elif action == "READY": self.stage = 1
                    elif action == "MANUAL_MODE": self.stage = 10
                    elif action == "GRIPPER_RELEASE": self.is_finished = True
                    elif action == "STOP" and self.stage == 2: self.stage += 1
                    elif not self.is_finished: self.stage += 1
        else:
            self.trigger_counter = 0
            self.triggered_lock = False

        return action, frame