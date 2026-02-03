import cv2
import numpy as np
import math

class DockingAI:
    def __init__(self):
        # ---------------------------------------------------------
        # [1] 사용자 설정 및 상수
        # ---------------------------------------------------------
        self.MARKER_SIZE = 1.1  # 단위: cm (실측값)
        self.TARGET_ID = 0      # 목표 마커 ID
        self.target_dict = cv2.aruco.DICT_6X6_250 
        
        # [거리 보정] (10~30cm 구간 최적화 값)
        self.DIST_SCALE = 1.45   
        self.DIST_OFFSET = -1.5  

        # [최적화 1] 프레임 스킵 (CPU 부하 감소)
        self.frame_count = 0
        self.PROCESS_INTERVAL = 4  # 4프레임 중 1번만 연산 (약 7.5fps)

        # [최적화 2] ROI 설정 (상단 + 좌우 잘라내기)
        # 예: Y=0.4 (상단 40% 버림), X=0.2 (좌우 20%씩 버림 -> 중앙 60%만 봄)
        self.ROI_Y_START_RATIO = 0.4 
        self.ROI_X_MARGIN_RATIO = 0.2

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.target_dict)
        self.parameters = cv2.aruco.DetectorParameters()

        # [최적화 3] 파라미터 튜닝 (1.1cm 마커용)
        self.parameters.minMarkerPerimeterRate = 0.02 
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 
        self.parameters.adaptiveThreshWinSizeStep = 8 
        
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # ---------------------------------------------------------
        # [2] 캘리브레이션 데이터 (Camera Matrix)
        # ---------------------------------------------------------
        self.camera_matrix = np.array([
            [872.23558, 0.00000, 315.00614],
            [0.00000, 873.47815, 240.01070],
            [0.00000, 0.00000, 1.00000]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [0.14923, -1.11676, 0.00511, 0.00329, 7.40075]
        ], dtype=np.float32)

        # 3D 객체 좌표계 정의
        ms = self.MARKER_SIZE / 2
        self.obj_points = np.array([
            [-ms, ms, 0], [ms, ms, 0], [ms, -ms, 0], [-ms, -ms, 0]
        ], dtype=np.float32)

        # 상태 저장용
        self.last_data = {
            "found": False, "id": -1, 
            "dist_cm": 0.0, "x_cm": 0.0, "yaw": 0.0, 
            "roll": 0.0, "pitch": 0.0, "center": (0, 0)
        }
        self.tracking_data = {} 
        self.ALPHA = 0.3 # 스무딩 계수

    def euler_from_quaternion(self, rvec):
        """Rodrigues 벡터를 Euler 각도로 변환"""
        rmat, _ = cv2.Rodrigues(rvec)
        sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
        if sy < 1e-6:
            x = math.atan2(-rmat[1,2], rmat[1,1])
            y = math.atan2(-rmat[2,0], sy)
            z = 0
        else:
            x = math.atan2(rmat[2,1] , rmat[2,2])
            y = math.atan2(-rmat[2,0], sy)
            z = math.atan2(rmat[1,0], rmat[0,0])
        return x*180/math.pi, y*180/math.pi, z*180/math.pi

    def process(self, frame):
        h, w, _ = frame.shape
        self.frame_count += 1
        
        # [최적화 1] Interval Check
        if self.frame_count % self.PROCESS_INTERVAL != 0:
            return self._draw_ui(frame, self.last_data)

        # [최적화 2] ROI 계산 (상단 + 좌우 자르기)
        y_start = int(h * self.ROI_Y_START_RATIO)
        x_start = int(w * self.ROI_X_MARGIN_RATIO)
        x_end   = int(w * (1 - self.ROI_X_MARGIN_RATIO))
        
        # 슬라이싱 (여기서 이미지가 작아짐 -> 연산 속도 UP)
        roi_frame = frame[y_start:h, x_start:x_end]
        roi_gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(roi_gray)
        
        # [중요] 좌표 보정: ROI 기준 좌표를 원본 화면(frame) 기준으로 복구
        if ids is not None:
            for corner in corners:
                corner[:, :, 0] += x_start # X좌표 오프셋 복구
                corner[:, :, 1] += y_start # Y좌표 오프셋 복구

        # 데이터 처리 (이하 로직은 원본 좌표계 기준이므로 동일)
        data = { "found": False, "id": -1, "dist_cm": 0.0, "x_cm": 0.0, "yaw": 0.0, "roll": 0.0, "pitch": 0.0, "center": (0, 0) }

        best_marker_idx = -1
        best_rvec, best_tvec = None, None
        found_target = False
        current_visible_ids = []

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                current_visible_ids.append(marker_id)

                if marker_id == self.TARGET_ID:
                    target_corners = corners[i][0]
                    found_target = True
                    
                    # Pose Estimation
                    _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                        self.obj_points, target_corners, self.camera_matrix, self.dist_coeffs, 
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    # Ambiguity 해결
                    final_rvec = rvecs[0]
                    final_tvec = tvecs[0]
                    
                    if marker_id in self.tracking_data:
                        prev_rvec = self.tracking_data[marker_id]['rvec']
                        diff0 = abs(rvecs[0] - prev_rvec).sum()
                        diff1 = abs(rvecs[1] - prev_rvec).sum() if len(rvecs) > 1 else 9999
                        if diff1 < diff0:
                            final_rvec = rvecs[1]
                            final_tvec = tvecs[1]
                    
                    # 스무딩
                    if marker_id in self.tracking_data:
                        last_rvec = self.tracking_data[marker_id]['rvec']
                        last_tvec = self.tracking_data[marker_id]['tvec']
                        final_rvec = self.ALPHA * final_rvec + (1 - self.ALPHA) * last_rvec
                        final_tvec = self.ALPHA * final_tvec + (1 - self.ALPHA) * last_tvec

                    self.tracking_data[marker_id] = {'rvec': final_rvec, 'tvec': final_tvec}
                    best_marker_idx = i
                    best_rvec, best_tvec = final_rvec, final_tvec
                    data["id"] = int(marker_id)
                    break 

        # 트래킹 데이터 정리
        keys_to_remove = [k for k in self.tracking_data if k not in current_visible_ids]
        for k in keys_to_remove: del self.tracking_data[k]

        if found_target and best_marker_idx != -1:
            data["found"] = True
            
            # 거리 및 각도 계산
            raw_dist = math.sqrt(best_tvec[0]**2 + best_tvec[1]**2 + best_tvec[2]**2)
            data["dist_cm"] = (raw_dist * self.DIST_SCALE) + self.DIST_OFFSET
            data["x_cm"] = best_tvec[0][0] * self.DIST_SCALE
            data["roll"], data["pitch"], raw_yaw = self.euler_from_quaternion(best_rvec)
            data["yaw"] = raw_yaw 
            
            cx = int(corners[best_marker_idx][0][:, 0].mean())
            cy = int(corners[best_marker_idx][0][:, 1].mean())
            data["center"] = (cx, cy)

            # 시각화 (원본 프레임에 그림)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, best_rvec, best_tvec, self.MARKER_SIZE * 1.5)

        self.last_data = data
        return self._draw_ui(frame, data)

    def _draw_ui(self, frame, data):
        """UI 그리기 및 ROI 박스 표시"""
        h, w, _ = frame.shape
        box_width, box_height = 260, 190
        box_x, box_y = w - box_width, h - box_height

        overlay = frame.copy()
        cv2.rectangle(overlay, (box_x, box_y), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # 텍스트 색상
        dist_col = (0, 255, 0) if (7.5 <= data["dist_cm"] <= 9.5) else (0, 255, 255)
        yaw_col = (0, 255, 0) if abs(data["yaw"]) <= 0.5 else (0, 255, 255)
        x_col    = (0, 255, 0) if abs(data["x_cm"]) <= 0.1 else (0, 255, 255)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        if data["found"]:
            cv2.putText(frame, f"ID: {data['id']}", (box_x+10, box_y+30), font, 0.8, (0,255,255), 2)
            cv2.putText(frame, f"Dist : {data['dist_cm']:.1f} cm", (box_x+10, box_y+65), font, 0.7, dist_col, 2)
            cv2.putText(frame, f"Yaw  : {data['yaw']:.1f} deg", (box_x+10, box_y+100), font, 0.7, yaw_col, 2)
            cv2.putText(frame, f"X    : {data['x_cm']:.2f} cm", (box_x+10, box_y+135), font, 0.7, x_col, 2)
            cv2.putText(frame, f"P:{data['pitch']:.1f} R:{data['roll']:.1f}", (box_x+10, box_y+165), font, 0.6, (200,200,200), 1)
        else:
            cv2.putText(frame, "SEARCHING...", (box_x+10, box_y+100), font, 0.8, (0,0,255), 2)

        # [디버깅] ROI 영역 파란색 박스로 표시
        y_start = int(h * self.ROI_Y_START_RATIO)
        x_start = int(w * self.ROI_X_MARGIN_RATIO)
        x_end   = int(w * (1 - self.ROI_X_MARGIN_RATIO))
        
        cv2.rectangle(frame, (x_start, y_start), (x_end, h), (255, 0, 0), 2)
        cv2.putText(frame, "ROI AREA", (x_start + 5, y_start + 20), font, 0.5, (255, 0, 0), 1)

        return data, frame