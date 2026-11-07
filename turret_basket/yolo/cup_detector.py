"""
================================================================================
BASKET CHALLENGE: YOLOV8 KUPA / BARDAK TESPIT MODULU (CUP DETECTOR)
================================================================================
YOLOv8 sinif 41 ('cup') agirliklarini kullanarak sahnede kulplu kupayi tespit eden,
kutu koordinatlarindaki titresimi EMA ile pürüzsüzleştiren ve taret nişan hatalarını
hesaplayan optik tespit motoru.
================================================================================
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from turret_basket.config import CONFIG


@dataclass
class CupDetection:
    """Tek bir kupa tespit sonucu."""
    detected: bool = False
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # (x1, y1, x2, y2)
    center_px: Tuple[int, int] = (0, 0)             # (cx, cy)
    confidence: float = 0.0
    err_x_norm: float = 0.0     # [-1.0, 1.0] Ekran yatay merkez sapması
    err_y_norm: float = 0.0     # [-1.0, 1.0] Ekran dikey merkez sapması
    err_pan_deg: float = 0.0    # Açısal yatay nişan hatası (derece)
    err_tilt_deg: float = 0.0   # Açısal dikey nişan hatası (derece)
    class_name: str = "cup"


class CupDetector:
    """YOLOv8 tabanli hassas kupa tespit ve filtreleme sinifi."""

    def __init__(self, model_path: Optional[str] = None):
        path = model_path or CONFIG.cup.yolo_model_path
        if YOLO is not None:
            self.model = YOLO(path, verbose=False)
        else:
            self.model = None

        self.target_classes = CONFIG.cup.target_classes
        self.conf_thresh = CONFIG.cup.confidence_threshold
        self.alpha = CONFIG.cup.ema_alpha

        # EMA Kutu Filtresi (Titreme onleyici)
        self.smooth_box = None
        self.lost_frames = 0
        self.MAX_LOST_FRAMES = 8

        # Kupa agzina kilitlenme orani (Standart: Kutu tepesinin %10 alti = tam agiz elipsi)
        params_file = Path(__file__).resolve().parent.parent / "rl" / "models" / "cup_tracker_params.json"
        self.aim_mouth_ratio = CONFIG.cup.aim_mouth_ratio
        if params_file.exists():
            try:
                with open(params_file, "r", encoding="utf-8") as f:
                    p = json.load(f)
                    if "aim_mouth_ratio" in p:
                        val = float(p.get("aim_mouth_ratio", CONFIG.cup.aim_mouth_ratio))
                        self.aim_mouth_ratio = val if (0.04 <= val <= 0.16) else CONFIG.cup.aim_mouth_ratio
            except Exception:
                pass

    def reset(self):
        """Filtre belleğini sıfırlar."""
        self.smooth_box = None
        self.lost_frames = 0

    def detect(self, frame: np.ndarray, tilt_deg: Optional[float] = None) -> CupDetection:
        """Kare uzerinde kupa tespiti yapar, kutuyu EMA ile filtreler ve nisan hatalarini hesaplar.
        
        Args:
            frame: BGR goruntu matrisi
            tilt_deg: Taret anlik dikey egim acisi (derece). Verilirse kamera baxis acisina gore
                      agiz elipsi merkezini perspektif olarak dinamik standardize eder.
        """
        if self.model is None or frame is None or frame.size == 0:
            return CupDetection()

        h, w = frame.shape[:2]
        cx_screen, cy_screen = w / 2.0, h / 2.0

        # YOLOv8 çıkarımı
        results = self.model.predict(
            source=frame,
            conf=self.conf_thresh,
            classes=self.target_classes,
            verbose=False,
            imgsz=CONFIG.camera.width
        )

        best_box = None
        best_conf = 0.0

        if len(results) > 0 and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            for i in range(len(boxes)):
                conf = float(boxes.conf[i].cpu().item())
                if conf > best_conf:
                    best_conf = conf
                    xyxy = boxes.xyxy[i].cpu().numpy().tolist()
                    best_box = xyxy

        # Tespit varsa EMA ile pürüzsüzleştir
        if best_box is not None:
            self.lost_frames = 0
            curr_box = np.array(best_box, dtype=np.float32)
            if self.smooth_box is None:
                self.smooth_box = curr_box
            else:
                self.smooth_box = self.alpha * curr_box + (1.0 - self.alpha) * self.smooth_box

            x1, y1, x2, y2 = self.smooth_box.tolist()
            conf = best_conf
            detected = True
        else:
            # Geçici kaçışlarda önceki kutuyu bir miktar koru (hysteresis)
            self.lost_frames += 1
            if self.smooth_box is not None and self.lost_frames <= self.MAX_LOST_FRAMES:
                x1, y1, x2, y2 = self.smooth_box.tolist()
                conf = self.conf_thresh
                detected = True
            else:
                self.smooth_box = None
                return CupDetection(detected=False)

        # ---------------------------------------------------------------------
        # Kupa Agzi Elipsinin Merkez Koordinati (Standart Kararli Oran)
        # ---------------------------------------------------------------------
        # Kulplu kupanın ağzı kutunun en üstünden (y1) standart oranda (aim_mouth_ratio = %10) aşağıdadır.
        # Sabit oran kullanılarak taret eğimi ile hedefleme arasında geri besleme döngüsü engellenir.
        effective_ratio = self.aim_mouth_ratio

        cup_w = x2 - x1
        cup_h = y2 - y1
        aim_x = x1 + (cup_w / 2.0)
        aim_y = y1 + (cup_h * effective_ratio)

        # Ekran merkezine göre normalize hata [-1.0, 1.0]
        err_x_norm = (aim_x - cx_screen) / cx_screen
        err_y_norm = (aim_y - cy_screen) / cy_screen

        # Açısal hata (pinhole kamera modeli: theta = atan(err_px / f))
        fx = CONFIG.camera.fx
        fy = CONFIG.camera.fy
        err_pan_deg = math.degrees(math.atan((aim_x - cx_screen) / fx))
        err_tilt_deg = math.degrees(math.atan((aim_y - cy_screen) / fy))

        return CupDetection(
            detected=detected,
            bbox=(int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))),
            center_px=(int(round(aim_x)), int(round(aim_y))),
            confidence=conf,
            err_x_norm=float(err_x_norm),
            err_y_norm=float(err_y_norm),
            err_pan_deg=float(err_pan_deg),
            err_tilt_deg=float(err_tilt_deg),
            class_name="cup"
        )
