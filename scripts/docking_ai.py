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
        
        # [거리 보정] (10~30cm 구간 정밀 보정)
        self.DIST_SCALE = 1.45   
        self.DIST_OFFSET = -1.5  

        # [핵심 최적화 1] 프레임 스킵 (CPU 부하 감소)
        self.frame_count = 0
        self.PROCESS_INTERVAL = 4  # 4프레임 중 1번만 연산 (30fps -> 7.5fps 처리)

        # [핵심 최적화 2] Super ROI 설정 (불필요한 배경 제거)
        # 상단 40% 버림 (바닥만 봄)
        self.ROI_Y_START_RATIO = 0.4 
        # 좌우 20%씩 버림 (로봇 정면만 집중, 양옆 노이즈 제거)
        self.ROI_X_MARGIN_RATIO = 0.2

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.target_dict)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 
        
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
        
        # [최적화 1] Interval Check (쉬는 타임)
        if self.frame_count % self.PROCESS_INTERVAL != 0:
            return self.last_data, self._draw_roi_box(frame)

        # [최적화 2] ROI 계산 (상단 + 좌우 자르기)
        y_start = int(h * self.ROI_Y_START_RATIO)
        x_start = int(w * self.ROI_X_MARGIN_RATIO)
        x_end   = int(w * (1 - self.ROI_X_MARGIN_RATIO))
        
        # 이미지 자르기 (연산량 약 64% 감소)
        roi_frame = frame[y_start:h, x_start:x_end]
        roi_gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(roi_gray)
        
        # [중요] 좌표 보정: 잘린 이미지 좌표 -> 원본 좌표로 복구
        if ids is not None:
            for corner in corners:
                corner[:, :, 0] += x_start # X좌표 복구
                corner[:, :, 1] += y_start # Y좌표 복구

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

                    # Ambiguity 해결 및 스무딩
                    final_rvec = rvecs[0]
                    final_tvec = tvecs[0]
                    
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
            
            # 시각화 (원본 프레임에 그림) - Headless여도 나중에 디버깅을 위해 코드 유지
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, best_rvec, best_tvec, self.MARKER_SIZE * 1.5)

        self.last_data = data
        return data, self._draw_roi_box(frame)

    def _draw_roi_box(self, frame):
        """디버깅용: 실제 연산하는 영역을 파란 박스로 표시"""
        h, w, _ = frame.shape
        y_start = int(h * self.ROI_Y_START_RATIO)
        x_start = int(w * self.ROI_X_MARGIN_RATIO)
        x_end   = int(w * (1 - self.ROI_X_MARGIN_RATIO))
        
        cv2.rectangle(frame, (x_start, y_start), (x_end, h), (255, 0, 0), 2)
        return frame