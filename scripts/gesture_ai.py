import cv2
import numpy as np
from ultralytics import YOLO

class MarshallerAI:
    def __init__(self):
        self.model = YOLO('yolov8n-pose.engine', task='pose')
        
        # [상태 관리 변수]
        self.stage = 0 
        self.is_finished = False 
        
        # [스테이지 전환용 타이머 설정]
        self.trigger_counter = 0
        self.triggered_lock = False 
        
        self.LIMIT_NORMAL = 20       # 일반 동작: 약 1.0초
        self.LIMIT_RESET  = 40       # 리셋 동작: 약 2.0초

    def calculate_angle(self, a, b, c):
        a, b, c = np.array(a), np.array(b), np.array(c)
        radians = np.arctan2(c[1]-b[1], c[0]-b[0]) - np.arctan2(a[1]-b[1], a[0]-b[0])
        angle = np.abs(radians*180.0/np.pi)
        if angle > 180.0: angle = 360-angle
        return angle

    def draw_custom_skeleton(self, frame, kpts):
        connections = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12), (11, 12)]
        for start_idx, end_idx in connections:
            if kpts[start_idx][2] > 0.5 and kpts[end_idx][2] > 0.5:
                x1, y1 = int(kpts[start_idx][0]), int(kpts[start_idx][1])
                x2, y2 = int(kpts[end_idx][0]), int(kpts[end_idx][1])
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
        for idx in [0, 5, 6, 7, 8, 9, 10, 11, 12]:
             if kpts[idx][2] > 0.5:
                cv2.circle(frame, (int(kpts[idx][0]), int(kpts[idx][1])), 6, (0, 0, 255), -1)

    def draw_status(self, frame, action, info, color):
        h, w, _ = frame.shape
        box_w, box_h = 300, 90
        x1, y1 = w - box_w, h - box_h
        cv2.rectangle(frame, (x1, y1), (w, h), color, -1)
        cv2.putText(frame, action, (x1+10, y1+35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
        if info:
            cv2.putText(frame, info, (x1+10, y1+65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"STAGE: {self.stage}", (20, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    def draw_loading_bar(self, frame, progress):
        h, w, _ = frame.shape
        bar_w, bar_h = 400, 30
        x1 = (w - bar_w) // 2
        y1 = h // 2 + 100
        
        cv2.rectangle(frame, (x1, y1), (x1 + bar_w, y1 + bar_h), (50, 50, 50), -1)
        fill_w = int(bar_w * progress)
        cv2.rectangle(frame, (x1, y1), (x1 + fill_w, y1 + bar_h), (0, 255, 0), -1)
        
        msg = "HOLDING..." if progress < 1.0 else "ACTION COMPLETE!"
        cv2.putText(frame, msg, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    def detect_gesture(self, frame):
        h, w, _ = frame.shape
        results = self.model(frame, verbose=False, conf=0.5)
        
        if results[0].keypoints is None or len(results[0].keypoints.data) == 0:
            self.draw_status(frame, "NO HUMAN", "", (100, 100, 100))
            return "IDLE", frame

        kpts_raw = results[0].keypoints.data[0].cpu().numpy()
        def get_norm(idx): return [kpts_raw[idx][0]/w, kpts_raw[idx][1]/h]

        l_sh, r_sh = get_norm(5), get_norm(6)
        l_el, r_el = get_norm(7), get_norm(8)
        l_wr, r_wr = get_norm(9), get_norm(10)
        l_hip, r_hip = get_norm(11), get_norm(12) 
        
        angle_l = self.calculate_angle(l_sh, l_el, l_wr)
        angle_r = self.calculate_angle(r_sh, r_el, r_wr)
        
        wrist_dist_x = abs(l_wr[0] - r_wr[0])
        shoulder_width = abs(l_sh[0] - r_sh[0])
        
        current_action = "READY"
        info_text = ""
        bg_color = (245, 117, 16)
        
        is_triggering = False 
        global_action_detected = False

        # -----------------------------------------------------------------
        # [GLOBAL 1] RESET (열중쉬어) - 조건 강화됨
        # -----------------------------------------------------------------
        has_arms = (kpts_raw[9][2] > 0.6) and (kpts_raw[10][2] > 0.6)
        has_hips = (kpts_raw[11][2] > 0.6) and (kpts_raw[12][2] > 0.6)
        
        if has_arms and has_hips:
            # 1. 손목 위치 수정: 허리 위쪽으로 올려야 함 (Y값이 hip보다 작거나 비슷해야 함)
            # 기존: abs(diff) < 0.2 (골반 아래 0.2까지 허용했음 -> 차렷과 겹침)
            # 수정: l_wr[1] < l_hip[1] + 0.05 (골반보다 아주 살짝 아래까지만 허용, 그보다 위여야 함)
            l_on_waist = (l_wr[1] < l_hip[1] + 0.05) and (l_wr[1] > l_sh[1] + 0.2)
            r_on_waist = (r_wr[1] < r_hip[1] + 0.05) and (r_wr[1] > r_sh[1] + 0.2)
            
            # 2. 팔꿈치 벌림 유지
            elbow_width = abs(l_el[0] - r_el[0])
            is_elbows_out = elbow_width > shoulder_width * 1.2 
            
            # 3. 팔 굽힘 각도 강화: 150도 -> 140도 미만 (더 확실히 굽혀야 함)
            # 차렷 자세는 보통 160~180도 나오므로 겹칠 일 없음
            is_bent = (angle_l < 140) and (angle_r < 140)

            if l_on_waist and r_on_waist and is_elbows_out and is_bent:
                current_action = "RESET"
                info_text = "HOLD 2s TO RESET"
                bg_color = (255, 0, 0)
                is_triggering = True
                global_action_detected = True

        # -----------------------------------------------------------------
        # [GLOBAL 2] STOP (X자)
        # -----------------------------------------------------------------
        if not global_action_detected:
            is_crossed = r_wr[0] > l_wr[0]
            if is_crossed and (l_wr[1] < l_sh[1] + 0.4):
                if self.stage == 2 or self.is_finished:
                    current_action = "STOP"
                    info_text = "EMERGENCY STOP"
                    bg_color = (0, 0, 255)
                    if self.stage == 2:
                        is_triggering = True
                        info_text = "HOLD TO STAGE 3..."
                    global_action_detected = True

        # -----------------------------------------------------------------
        # [FINISHED] BYE BYE
        # -----------------------------------------------------------------
        if self.is_finished and not global_action_detected:
            current_action = "BYE BYE"
            info_text = "MISSION COMPLETE"
            bg_color = (0, 255, 0)
            is_triggering = False

        # -----------------------------------------------------------------
        # [STAGES]
        # -----------------------------------------------------------------
        if not self.is_finished and not global_action_detected:
            # Stage 0: READY
            if self.stage == 0:
                is_arms_down = (l_wr[1] > l_sh[1] + 0.3) and (r_wr[1] > r_sh[1] + 0.3)
                is_straight = (angle_l > 150) and (angle_r > 150)
                is_narrow = wrist_dist_x < shoulder_width * 1.25
                if is_arms_down and is_narrow and is_straight:
                    is_triggering = True; current_action = "READY"; info_text = "HOLD TO START..."
                else:
                    current_action = "FACE_ME"; info_text = "STAGE 0: WAITING..."; bg_color = (100, 100, 100)

            # Stage 1: MOVE
            elif self.stage == 1:
                l_diff = l_wr[1] - l_sh[1]; r_diff = r_wr[1] - r_sh[1]
                is_fast = (l_diff > 0.25 and r_diff > 0.25) and (wrist_dist_x > shoulder_width * 1.4)
                if is_fast:
                    is_triggering = True; current_action = "APPROACHING"; info_text = "HOLD TO STAGE 2..."
                elif abs(l_el[1]-l_sh[1])<0.2 and l_wr[1]<l_el[1] and angle_l<120 and angle_r<120: current_action = "FORWARD"
                elif abs(l_el[1]-l_sh[1])<0.2 and l_wr[1]<l_el[1] and r_wr[1]>r_sh[1]+0.2: current_action = "TURN_LEFT"
                elif abs(r_el[1]-r_sh[1])<0.2 and r_wr[1]<r_el[1] and l_wr[1]>l_sh[1]+0.2: current_action = "TURN_RIGHT"
                else: current_action = "STAGE_1"; info_text = "FWD / LEFT / RIGHT"

            # Stage 2: APPROACH & STOP
            elif self.stage == 2:
                # STOP은 Global에서 처리됨
                l_diff = l_wr[1] - l_sh[1]; r_diff = r_wr[1] - r_sh[1]
                if abs(l_diff)<0.25 and abs(r_diff)<0.25: current_action = "APPROACHING"; info_text = "SPEED: NORMAL"
                elif angle_l>120 and angle_r>120 and l_wr[1]<l_sh[1]: current_action = "APPROACHING"; info_text = "SPEED: SLOW"
                elif (l_diff>0.25 and r_diff>0.25) and (wrist_dist_x>shoulder_width*1.4): current_action = "APPROACHING"; info_text = "SPEED: FAST"
                else: current_action = "STAGE_2"; info_text = "APPROACH ONLY"

            # Stage 3: GRIPPER HOLD
            elif self.stage == 3:
                in_chest = (l_wr[1] > l_sh[1]) and (l_wr[1] < l_hip[1])
                is_hold = in_chest and (wrist_dist_x < shoulder_width * 0.6)
                if is_hold: is_triggering = True; current_action = "GRIPPER_HOLD"; info_text = "HOLD TO STAGE 4..."
                else:
                    if (l_wr[1]<l_sh[1] and r_wr[1]>r_sh[1]) or (r_wr[1]<r_sh[1] and l_wr[1]>l_sh[1]): current_action = "SET_BRAKES"
                    else: current_action = "STAGE_3"; info_text = "BRAKES ONLY"

            # Stage 4: GRIPPER RELEASE -> EXIT
            elif self.stage == 4:
                in_chest = (l_wr[1] > l_sh[1]) and (l_wr[1] < l_hip[1])
                is_release = in_chest and (wrist_dist_x > shoulder_width * 0.8)
                if is_release: is_triggering = True; current_action = "GRIPPER_RELEASE"; info_text = "HOLD TO FINISH..."
                elif wrist_dist_x < shoulder_width * 0.6: current_action = "GRIPPER_HOLD"
                else: current_action = "STAGE_4"; info_text = "OPEN ARMS TO FINISH"

        # -----------------------------------------------------------------
        # [TRIGGER LOGIC]
        # -----------------------------------------------------------------
        target_limit = self.LIMIT_RESET if current_action == "RESET" else self.LIMIT_NORMAL

        if is_triggering:
            if self.triggered_lock:
                self.draw_loading_bar(frame, 1.0)
            else:
                self.trigger_counter += 1
                progress = min(self.trigger_counter / target_limit, 1.0)
                self.draw_loading_bar(frame, progress)
                
                if self.trigger_counter >= target_limit:
                    self.triggered_lock = True
                    
                    if current_action == "RESET":
                        self.stage = 0
                        self.is_finished = False
                    elif self.stage == 4 and current_action == "GRIPPER_RELEASE":
                        self.is_finished = True
                    else:
                        if not self.is_finished:
                            self.stage += 1
        else:
            self.trigger_counter = 0
            self.triggered_lock = False

        self.draw_custom_skeleton(frame, kpts_raw)
        self.draw_status(frame, current_action, info_text, bg_color)
        
        return current_action, frame