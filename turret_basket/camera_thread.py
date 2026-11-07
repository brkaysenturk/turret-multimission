"""
================================================================================
BASKET CHALLENGE: SIFIR GECIKMELI KAMERA SURUCUSU (CAMERA THREAD)
================================================================================
Arka planda calisan, OpenCV VideoCapture tampon yigilmasini engelleyen ve daima
en taze kareyi (latest frame) hazir tutan kamera surucusu.
================================================================================
"""

import threading
import time
import cv2

from .config import CONFIG


class CameraThread:
    """Tamponsuz, en taze kareyi sunan bagimsiz kamera okuma thread'i."""

    def __init__(self, src=None, width=None, height=None, fps=None):
        self.src = CONFIG.camera.device_id if src is None else src
        if self.src is None:
            try:
                from turret_fcs.camera_thread import CameraThread as FCSCameraThread
                det_idx, _ = FCSCameraThread.detect_camera(
                    preferred_name=CONFIG.camera.camera_name_substr,
                    backend="dshow"
                )
                self.src = det_idx
            except Exception:
                self.src = 0

        self.width = CONFIG.camera.width if width is None else width
        self.height = CONFIG.camera.height if height is None else height
        self.fps = CONFIG.camera.fps if fps is None else fps

        self.cap = None
        self._init_capture()

        self.grabbed = False
        self.frame = None
        self.stopped = False
        self.lock = threading.Lock()

        # Ilk kareyi oku
        if self.cap and self.cap.isOpened():
            self.grabbed, self.frame = self.cap.read()

        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _init_capture(self):
        """Kamerayi DirectShow veya MSMF ile acar."""
        # Windows DirectShow (en hizli baslama)
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.src)

        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def _update(self):
        while not self.stopped:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.05)
                continue

            grabbed, frame = self.cap.read()
            if grabbed:
                with self.lock:
                    self.grabbed = grabbed
                    self.frame = frame
            else:
                time.sleep(0.005)

    def read(self):
        """En son taze kareyi dondurur (None veya BGR NumPy array)."""
        with self.lock:
            if self.frame is not None:
                return self.grabbed, self.frame.copy()
            return False, None

    def stop(self):
        self.stopped = True
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()

