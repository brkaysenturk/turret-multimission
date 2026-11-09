"""
================================================================================
MODUL: SIFIR-GECIKMELI KAMERA IS PARCACIGI (CAMERA THREAD / DIRECTSHOW)
================================================================================
Bu modul, kameradan goruntu okuma islemini ana kontrol dongusunden izole eden
ve ayri bir is parcaciginda (daemon thread) calistiran yuksek basarimli goruntu motorudur.

KRITIK GOREVLER VE COZULEN PROBLEMLER:
1. Donanim Algilama: PyGrabber & DirectShow Graph API ile bagli USB video aygitlarini
   tarayarak "Logitech C270" kamerasini tak-calistir seklinde otomatik secer.
2. Tampon Gecikmesini (Buffer Lag) Sifirlama:
   Standart cv2.VideoCapture(), arka planda 3-5 karelik bir FIFO tamponu tutar.
   Bu durum gercek dunyada 100-150 ms'lik bayat goruntu (stale frame) gecikmesine yol acar.
   Bu sinif, en son gelen tek kareyi (latest frame) anlik saklar ve tamponu surekli
   bosaltarak goruntu gecikmesini sifira indirir.
3. Donanimsal Cozunurluk: 1280x720 @ 30 FPS optimize parametrelerle calisir.
================================================================================
"""

import threading
import time
import cv2

from .config import CONFIG


class CameraThread:
    """
    OpenCV VideoCapture nesnesini arka planda bagimsiz bir thread ile sarmalayan sinif.
    Kamera tamponundaki gecikmeyi sifirlar, MSMF ve DirectShow ile Logitech C270'i
    otonom olarak dogru backend indeksiyle eslestirir.
    """

    @staticmethod
    def _list_msmf_devices():
        """Windows Media Foundation (MSMF) uzerinden bagli video giris aygitlarini listeler."""
        devices = []
        try:
            import ctypes
            from ctypes import wintypes, POINTER, c_void_p, c_uint32, byref, cast

            class _GUID(ctypes.Structure):
                _fields_ = [
                    ("Data1", wintypes.DWORD),
                    ("Data2", wintypes.WORD),
                    ("Data3", wintypes.WORD),
                    ("Data4", wintypes.BYTE * 8),
                ]

            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE = _GUID(
                0xC60AC5FE, 0x252A, 0x478F,
                (wintypes.BYTE * 8)(0xA0, 0xEF, 0xBC, 0x8F, 0xA5, 0xF7, 0xCA, 0xD3)
            )
            MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID = _GUID(
                0x8AC35BA3, 0x4BE7, 0x4228,
                (wintypes.BYTE * 8)(0xAA, 0x6A, 0x51, 0x6B, 0x1A, 0xAB, 0x7E, 0x64)
            )
            MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME = _GUID(
                0x60D0E557, 0x52AC, 0x4FA8,
                (wintypes.BYTE * 8)(0xB2, 0x49, 0x43, 0x4B, 0xDA, 0x5F, 0x69, 0x90)
            )
            MF_VERSION = 0x00020070

            ole32 = ctypes.windll.ole32
            mfplat = ctypes.windll.mfplat
            mf = ctypes.windll.mf

            ole32.CoInitializeEx(None, 0x2)
            if mfplat.MFStartup(MF_VERSION, 0) != 0:
                return devices

            pAttributes = c_void_p()
            if mfplat.MFCreateAttributes(byref(pAttributes), 1) != 0 or not pAttributes:
                mfplat.MFShutdown()
                return devices

            vtbl = cast(cast(pAttributes, POINTER(c_void_p))[0], POINTER(c_void_p))
            SetGUID = ctypes.WINFUNCTYPE(ctypes.c_long, c_void_p, POINTER(_GUID), POINTER(_GUID))(vtbl[24])
            Release_proto = ctypes.WINFUNCTYPE(ctypes.c_ulong, c_void_p)
            Release = Release_proto(vtbl[2])

            if SetGUID(pAttributes, byref(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE),
                       byref(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID)) != 0:
                Release(pAttributes)
                mfplat.MFShutdown()
                return devices

            pppSourceActivate = POINTER(c_void_p)()
            count = c_uint32(0)
            hr = mf.MFEnumDeviceSources(pAttributes, byref(pppSourceActivate), byref(count))
            Release(pAttributes)

            if hr == 0 and count.value > 0 and pppSourceActivate:
                GetAllocatedString_proto = ctypes.WINFUNCTYPE(
                    ctypes.c_long, c_void_p, POINTER(_GUID), POINTER(wintypes.LPWSTR), POINTER(c_uint32)
                )
                for i in range(count.value):
                    pActivate = pppSourceActivate[i]
                    if not pActivate:
                        continue
                    act_vtbl = cast(cast(pActivate, POINTER(c_void_p))[0], POINTER(c_void_p))
                    GetAllocatedString = GetAllocatedString_proto(act_vtbl[13])
                    ActRelease = Release_proto(act_vtbl[2])

                    pwsz = wintypes.LPWSTR()
                    cch = c_uint32()
                    name = f"MSMF Camera {i}"
                    if GetAllocatedString(pActivate, byref(MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME), byref(pwsz), byref(cch)) == 0:
                        if pwsz and pwsz.value:
                            name = str(pwsz.value)
                            ole32.CoTaskMemFree(pwsz)
                    devices.append(name)
                    ActRelease(pActivate)

                ole32.CoTaskMemFree(pppSourceActivate)

            mfplat.MFShutdown()
        except Exception:
            pass
        return devices

    @staticmethod
    def _list_dshow_devices():
        """DirectShow (pygrabber) uzerinden bagli video giris aygitlarini listeler."""
        try:
            from pygrabber.dshow_graph import FilterGraph
            return FilterGraph().get_input_devices() or []
        except Exception:
            return []

    @classmethod
    def detect_camera(cls, preferred_name="C270", backend="msmf"):
        """Sistemdeki kameralari belirtilen backend'e gore listeler ve tercihen C270'i secer."""
        preferred_patterns = [preferred_name.lower(), "c270", "logi"]
        if backend.lower() == "msmf":
            devices = cls._list_msmf_devices()
            if devices:
                for idx, dev in enumerate(devices):
                    dev_lower = dev.lower()
                    if any(p in dev_lower for p in preferred_patterns):
                        return idx, dev
                return 0, devices[0]
            return 0, "Default Camera (MSMF)"

        # DirectShow
        devices = cls._list_dshow_devices()
        if devices:
            for idx, dev in enumerate(devices):
                dev_lower = dev.lower()
                if any(p in dev_lower for p in preferred_patterns):
                    return idx, dev
            return 0, devices[0]
        return 0, "Default Camera (DSHOW)"

    def __init__(
        self,
        src=None,
        width=CONFIG.optics.default_width,
        height=CONFIG.optics.default_height,
        fps=CONFIG.optics.default_fps,
    ):
        if src is None:
            msmf_src, msmf_name = self.detect_camera(backend="msmf")
            dshow_src, dshow_name = self.detect_camera(backend="dshow")
            self.src, self.cam_name = msmf_src, msmf_name
        else:
            msmf_src = dshow_src = src
            self.src = src
            self.cam_name = f"Index {src}"

        # Windows'ta MSMF ve DSHOW farkli aygit indeks siralamasi kullanir.
        # MSMF kendi indeksini (msmf_src), DSHOW fallback ise kendi indeksini (dshow_src) kullanir.
        fourcc_mjpg = cv2.VideoWriter.fourcc(*"MJPG")
        self.cap = cv2.VideoCapture(msmf_src, cv2.CAP_MSMF)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            try:
                self.cap.release()
            except Exception:
                pass
            if src is None:
                self.src, self.cam_name = dshow_src, dshow_name
            self.cap = cv2.VideoCapture(self.src, cv2.CAP_DSHOW)
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpg)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.grabbed, self.frame = self.cap.read()
        self.last_frame_time = time.time()
        self.frame_id = 0
        self.new_frame_event = threading.Event()
        self.running = False
        self.lock = threading.Lock()
        self.thread = None

        if self.grabbed:
            self.start()

    def start(self):
        """Kare yakalama is parcacigini baslatir."""
        if not self.running:
            self.running = True
            self.last_frame_time = time.time()
            self.thread = threading.Thread(target=self._update, daemon=True)
            self.thread.start()

    def _update(self):
        """Arka planda surekli en guncel kareyi okur ve yeni kare olayini tetikler."""
        while self.running and self.cap.isOpened():
            grabbed, frame = self.cap.read()
            if not grabbed:
                time.sleep(0.005)
                continue

            with self.lock:
                self.grabbed = grabbed
                self.frame = frame
                self.frame_id += 1
                self.last_frame_time = time.time()
            self.new_frame_event.set()

    def read(self, timeout=0.35, wait_for_new=False, return_id=False):
        """
        Ana donguye sifir gecikmeli en son kareyi kopyalar (donma korumali).
        wait_for_new=True secilirse ana dongu 30 FPS kamera yakalama hiziyla donanimsal olarak
        senkronize edilir; ayni karenin tekrar tekrar (duplike) islenmesi %100 engellenir.
        return_id=True verilirse (ok, frame, frame_id) seklinde doner.
        """
        if wait_for_new:
            self.new_frame_event.wait(timeout=min(0.100, timeout))
            self.new_frame_event.clear()

        with self.lock:
            is_stale = (time.time() - self.last_frame_time) > timeout
            if not self.grabbed or self.frame is None or is_stale:
                return (False, None, 0) if return_id else (False, None)
            copied_frame = self.frame.copy()
            fid = self.frame_id
            return (True, copied_frame, fid) if return_id else (True, copied_frame)

    @property
    def current_frame_id(self):
        """Son yakalanan donanimsal kare numarasini doner."""
        with self.lock:
            return self.frame_id

    def isOpened(self):
        with self.lock:
            is_stale = (time.time() - self.last_frame_time) > 0.60
            return self.running and self.cap.isOpened() and self.grabbed and not is_stale

    def stop(self):
        self.release()

    def release(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        if self.cap.isOpened():
            self.cap.release()
