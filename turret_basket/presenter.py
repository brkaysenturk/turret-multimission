"""
================================================================================
MODUL: BAS-USTU GOSTERGE SUNUCUSU (BASKET HUD PRESENTER)
================================================================================
Bu modul, Turret Basket ve Atis kontrol durumunu OpenCV gorsel katmanina (`BasketHUD`)
aktaran yuksek seviyeli sunum motorudur (Presenter). `main.py` ve `basket_hud.py`
arasindaki tum gorsel render siralamasini ve renk sozlesmesini tek yerde toplar.
================================================================================
"""

import cv2
import numpy as np

from turret_basket.basket_manager import BasketStatus
from turret_basket.config import CONFIG


class BasketPresenter:
    """Basket HUD katmanlarini sivil robotik hiyerarsisine gore kare uzerine cizen sunucu."""

    def __init__(self, hud):
        self.hud = hud

    def render(
        self,
        frame: np.ndarray,
        status: BasketStatus,
        fps: float = 30.0,
    ) -> np.ndarray:
        """Kare uzerine dengeli, sivil robotik tasarim dilinde HUD cizer."""
        out = frame.copy()
        h, w = out.shape[:2]
        cx, cy = w // 2, h // 2

        # 1. Sivillestirilmis Durum Metni ve Rengi
        if status.is_estop:
            status_text = "SAFETY STOP"
            status_color = self.hud.C_RED
        elif status.firing:
            status_text = "THROWING DART"
            status_color = self.hud.C_AMBER
        elif status.is_locked:
            status_text = "CUP ALIGNED"
            status_color = self.hud.C_GREEN
        elif status.cup_detected:
            status_text = "ALIGNING CUP"
            status_color = self.hud.C_AMBER
        else:
            status_text = "SCANNING"
            status_color = (140, 150, 160)

        # 2. Kupa Hedef Cercevesi ve Takip Cizgisi
        if status.cup_detected and status.bbox and not status.is_estop:
            x1, y1, x2, y2 = status.bbox
            if getattr(status, "center_px", None) and status.center_px[0] > 0:
                bx, by = status.center_px
            else:
                bx = (x1 + x2) // 2
                by = int(round(y1 + (y2 - y1) * CONFIG.cup.aim_mouth_ratio))

            box_color = self.hud.C_GREEN if status.is_locked else self.hud.C_AMBER
            label = "CUP"
            self.hud.draw_corner_box(out, x1, y1, x2, y2, box_color, length=18, thickness=2, label=label)

            # Kilitlenene kadar cismin merkez noktasi ve takip cizgisi gozukur; kilitlenince kaybolur
            if not status.is_locked and not status.firing:
                cv2.line(out, (cx, cy), (bx, by), (90, 100, 110), 1, cv2.LINE_AA)
                cv2.circle(out, (bx, by), 3, box_color, -1)

        # 3. Merkez Nisangah ve Hizalanma Dolum Bari (Zarif, sivil ince vizor)
        reticle_color = (
            self.hud.C_RED
            if status.is_estop
            else (self.hud.C_GREEN if status.is_locked else (self.hud.C_AMBER if status.cup_detected else (240, 215, 120)))
        )
        self.hud.draw_reticle(out, cx, cy, reticle_color)
        if status.cup_detected and not status.is_estop:
            self.hud.draw_lock_bar(out, cx, cy, status.lock_ratio, status.is_locked)

        # 4. Sol Ust, Sag Ust ve Ust Banner (Kilitliyken veya atistayken gizlenir, sadece merkez panel kalir)
        if not (status.is_locked or status.firing):
            # Ust Orta Banner: [RL CONTROLLER]
            self.hud.draw_mode_banner(out, is_locked=status.is_locked, firing=status.firing)

            # Sol Ust Telemetri Paneli (280px Genislik)
            self.hud.draw_telemetry(
                out,
                status_text=status_text,
                status_color=status_color,
                dist_m=status.distance_m,
                err_pan=status.err_pan_deg,
                err_tilt=status.err_tilt_deg,
                target_detected=status.cup_detected and not status.is_estop,
            )

            # Sag Ust FPS
            fps_text = f"FPS: {int(round(fps))}"
            (fps_w, _), _ = cv2.getTextSize(fps_text, self.hud.font, 0.50, 2)
            fps_color = (0, 255, 0) if fps >= 25.0 else (0, 165, 255)
            cv2.putText(out, fps_text, (w - fps_w - 18, 26), self.hud.font, 0.50, fps_color, 2, cv2.LINE_AA)

            # Sol Alt Olay Gunlugu
            self.hud.draw_event_log(out, max_lines=4, is_locked=status.is_locked, firing=status.firing)

        # 5. Hedefe Kilitlendiginde Acilan Merkez Atis Paneli (SADECE bu panel odakli kalir)
        if (status.is_locked or status.firing) and not status.is_estop:
            self.hud.draw_combat_panel(
                out,
                cx=cx,
                cy=cy,
                dist_m=status.distance_m,
                speed_mps=status.speed_mps,
                tilt_offset_deg=status.tilt_offset_deg,
                firing=status.firing,
            )

        # 6. Alt Operator Tus Bilgisi
        self.hud.draw_footer(out, is_locked=status.is_locked, firing=status.firing, is_estop=status.is_estop)

        return out
