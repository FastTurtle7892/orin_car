import cv2
import numpy as np
import math

class DockingAI:
    def __init__(self):
        # 마커 설정
        self.MARKER_SIZE = 3.4
        self.target_dict = cv2.aruco.DICT_6X6_250 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.target_dict)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # 캘리브레이션 값 (사용자 제공)
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

        self.smooth_data = {}
        self.ALPHA = 0.3 

    def euler_from_quaternion(self, rvec):
        rmat, _ = cv2.Rodrigues(rvec)
        sy = math.sqrt(rmat[0,0] * rmat[0,0] +  rmat[1,0] * rmat[1,0])
        if sy < 1e-6:
            return 0, 0, 0 # Singular
        
        x = math.atan2(rmat[2,1] , rmat[2,2])
        y = math.atan2(-rmat[2,0], sy)
        z = math.atan2(rmat[1,0], rmat[0,0])
        return math.degrees(z), math.degrees(x), math.degrees(y) # roll, pitch, yaw

    def process(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        data = {
            "found": False, "dist_cm": 0.0, "yaw": 0.0, "center": (0, 0)
        }

        min_distance = float('inf')
        
        if ids is not None:
            for i in range(len(ids)):
                # 모든 마커 인식 (또는 특정 ID 필터링 가능)
                target_corners = corners[i][0]
                success, rvec, tvec = cv2.solvePnP(
                    self.obj_points, target_corners, 
                    self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE 
                )
                
                if success:
                    # 스무딩 처리 생략 가능하나 유지
                    dist = math.sqrt(tvec[0]**2 + tvec[1]**2 + tvec[2]**2)
                    
                    if dist < min_distance:
                        min_distance = dist
                        # 각도 계산
                        roll, pitch, yaw = self.euler_from_quaternion(rvec)
                        
                        data["found"] = True
                        data["dist_cm"] = dist
                        data["yaw"] = yaw
                        
                        # 중심점
                        cx = int(target_corners[:, 0].mean())
                        cy = int(target_corners[:, 1].mean())
                        data["center"] = (cx, cy)
                        
                        # 시각화
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.MARKER_SIZE)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        return data, frame
