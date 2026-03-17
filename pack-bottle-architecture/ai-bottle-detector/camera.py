import cv2
import threading
import time


class Camera:
    def __init__(self, rtsp_url: str):
        self.cap = cv2.VideoCapture(rtsp_url)
        self.last_frame = None
        self.lock = threading.Lock()
        self.running = True

        self.thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            with self.lock:
                self.last_frame = frame

            time.sleep(0.05)

    def get_frame(self):
        with self.lock:
            if self.last_frame is None:
                return None
            return self.last_frame.copy()

    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()
