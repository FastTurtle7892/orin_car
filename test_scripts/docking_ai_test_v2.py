import cv2
import numpy as np
import math

class DockingAI:
    def __init__(self):
        # [설정]
        self.MARKER_SIZE = 1.1  # 마커 크기 (cm)
        self.TARGET_ID = 0      # 목표 ID
        self.target_dict = cv2.aruco.DICT_6X6_250 
        
        # [거리 보정]
        self.DIST_SCALE = 1.45    
        self.DIST_OFFSET = -1.5   
        
        # [최적화 설정] 
        self.PROCESS_SCALE = 0.5 

        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.target_dict)
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # [캘리브레이션]
        self.camera_matrix = np.array([
            [872.23558, 0.00000, 315.00614],
            [0.00000, 873.47815, 240.01070],
            [0.00000, 0.00000, 1.00000]
        ], dtype=np.float32)

        self.dist_coeffs = np.array([
            [0.14923, -1.11676, 0.00511, 0.00329, 7.40075]
        ], dtype=np.float32)

        ms = self.MARKER_SIZE / 2
        self.obj_points = np.array([
            [-ms, ms, 0], [ms, ms, 0], [ms, -ms, 0], [-ms, -ms, 0]
        ], dtype=np.float32)

        self.tracking_data = {} 
        self.ALPHA = 0.3 

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
        # [최적화] 이미지 크기 축소하여 마커 검출
        small_frame = cv2.resize(frame, (0, 0), fx=self.PROCESS_SCALE, fy=self.PROCESS_SCALE)
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = self.detector.detectMarkers(gray)
        corners = list(corners) # 튜플 -> 리스트 변환

        # 좌표 복구
        if corners:
            for i in range(len(corners)):
                corners[i] = corners[i] * (1.0 / self.PROCESS_SCALE)

        data = { 
            "found": False, "id": -1, 
            "dist_cm": 0.0, "x_cm": 0.0, 
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0, 
            "center": (0, 0) 
        }

        best_marker_idx = -1
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

                    # Ambiguity 해결 (두 개의 해 중 튀지 않는 값 선택)
                    final_rvec = rvecs[0]
                    final_tvec = tvecs[0]

                    if marker_id in self.tracking_data:
                        prev_rvec = self.tracking_data[marker_id]['rvec']
                        diff0 = abs(rvecs[0] - prev_rvec).sum()
                        diff1 = abs(rvecs[1] - prev_rvec).sum() if len(rvecs) > 1 else 9999
                        if diff1 < diff0:
                            final_rvec = rvecs[1]
                            final_tvec = tvecs[1]
                    
                    # 스무딩 (Low Pass Filter)
                    if marker_id in self.tracking_data:
                        last_rvec = self.tracking_data[marker_id]['rvec']
                        last_tvec = self.tracking_data[marker_id]['tvec']
                        final_rvec = self.ALPHA * final_rvec + (1 - self.ALPHA) * last_rvec
                        final_tvec = self.ALPHA * final_tvec + (1 - self.ALPHA) * last_tvec

                    self.tracking_data[marker_id] = {'rvec': final_rvec, 'tvec': final_tvec}
                    best_marker_idx = i
                    
                    # 데이터 채우기
                    data["id"] = int(marker_id)
                    data["found"] = True
                    
                    raw_dist = math.sqrt(final_tvec[0]**2 + final_tvec[1]**2 + final_tvec[2]**2)
                    data["dist_cm"] = (raw_dist * self.DIST_SCALE) + self.DIST_OFFSET
                    data["x_cm"] = final_tvec[0][0] * self.DIST_SCALE
                    
                    # 오일러 각도 변환 (Yaw 추출)
                    data["roll"], data["pitch"], raw_yaw = self.euler_from_quaternion(final_rvec)
                    data["yaw"] = raw_yaw 
                    
                    cx = int(corners[best_marker_idx][0][:, 0].mean())
                    cy = int(corners[best_marker_idx][0][:, 1].mean())
                    data["center"] = (cx, cy)
                    break 

        # 안 보이는 마커 데이터 삭제
        keys_to_remove = [k for k in self.tracking_data if k not in current_visible_ids]
        for k in keys_to_remove: del self.tracking_data[k]

        return data, frame