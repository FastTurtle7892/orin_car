import cv2
import time
import threading
from typing import Optional

class CameraManager:
    """
    - cv2.VideoCapture는 여기서만 1번 open
    - 백그라운드 thread에서 계속 read
    - latest_frame만 덮어쓰기 (큐 안 쌓음)
    """
    def __init__(self, device_index=0, width=640, height=480, fps=20):
        self.device_index = device_index
        self.width = width
        self.height = height
        self.fps = fps

        self._cap = None
        self._thread = None
        self._stop_evt = threading.Event()

        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_ts = 0.0
        self._seq = 0

        self._sleep_s = max(0.0, 1.0 / float(fps)) if fps > 0 else 0.01

    def start(self) -> bool:
        if self._thread and self._thread.is_alive():
            return True

        self._stop_evt.clear()

        self._cap = cv2.VideoCapture(self.device_index)
        # gst_pipeline = (
        #     "v4l2src device=/dev/video0 ! "
        #     "image/jpeg, width=640, height=480, framerate=30/1 ! "
        #     "jpegdec ! videoconvert ! video/x-raw, format=BGR ! "
        #     "appsink drop=True"
        # )
        # self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self._cap.isOpened():
            try:
                self._cap.release()
            except:
                pass
            self._cap = None
            return False

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_evt.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

        if self._cap is not None:
            try:
                self._cap.release()
            except:
                pass
            self._cap = None

        with self._lock:
            self._latest_frame = None
            self._latest_ts = 0.0
            self._seq = 0

    def _loop(self):
        while not self._stop_evt.is_set():
            if self._cap is None:
                time.sleep(0.1)
                continue

            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._latest_frame = frame
                    self._latest_ts = time.time()
                    self._seq += 1
            else:
                time.sleep(0.01)

            if self._sleep_s > 0:
                # time.sleep(self._sleep_s)
                time.sleep(0.01)
    def get_latest_frame(self, copy: bool = True) -> Optional["cv2.Mat"]:
        with self._lock:
            if self._latest_frame is None:
                return None
            return self._latest_frame.copy() if copy else self._latest_frame
