"""
================================================================================
BASKET CHALLENGE: MODERN BAS-USTU GOSTERGESI (BASKET HUD) - KOMPAKT TASARIM
================================================================================
turret_fcs standartlarina tam uyumlu, 640x480 cozunurluk icin optimize edilmis,
zarif, minimalist ve kompakt gorsel yerlesim:
- Paneller kucultulmus ve ince cercevelerle ferahlatilmistir (190-200px genislik).
- Font olcekleri ve cizgi kalinliklari goruntuyu bogmayacak sekilde ayarlanmistir.
- Sol Ust: STATUS ve mesafe / ofset paneli
- Ust Orta: Minimalist [RL CONTROLLER] banneri
- Sag Ust: FPS gostergesi ve kompakt BASKET SCOREBOARD
- Merkez: Zarif minimalist vizor, ince dolum bari ve atis paneli
- Sol Alt: Kompakt EVENT LOG
- Alt Bar: Silik operator tus kontrolleri
================================================================================
"""

from typing import Optional, Tuple
import cv2
import numpy as np


class BasketHUD:
    """turret_fcs ile tam dengeli, kompakt sivil robotik tasarim dilinde Basket HUD motoru."""

    def __init__(self):
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        # Renk Sabitleri (BGR)
        self.C_WHITE = (255, 255, 255)
        self.C_GRAY = (170, 175, 180)
        self.C_DARK_BORDER = (70, 80, 90)
        self.C_AMBER = (0, 191, 255)       # Aligning / Active / Amber
        self.C_GREEN = (0, 255, 128)       # Target Aligned / Success / Emerald Green
        self.C_CYAN = (255, 215, 0)        # Telemetry / Cyan
        self.C_RED = (0, 60, 255)          # Emergency / Safety / Red

        # Basket Istatistikleri
        self.total_shots = 0
        self.total_baskets = 0
        self.last_shot_result = "STANDBY"
        self.last_shot_color = self.C_GRAY

        # Olay Gunlugu
        self.event_logs = []

    def log_event(self, msg: str, color: Tuple[int, int, int] = (200, 210, 220)):
        """HUD olay gunlugune yeni bir sivil robotik olay ekler."""
        clean_msg = msg.strip().replace("\n", " ")
        self.event_logs.append((clean_msg, color))
        if len(self.event_logs) > 8:
            self.event_logs.pop(0)

    def record_shot(self, is_basket: bool):
        """Atis sonucunu istatistige kaydeder."""
        self.total_shots += 1
        if is_basket:
            self.total_baskets += 1
            self.last_shot_result = "BASKET! (+100)"
            self.last_shot_color = self.C_GREEN
        else:
            self.last_shot_result = "MISSED"
            self.last_shot_color = self.C_RED

    def draw_corner_box(
        self,
        img: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        color: Tuple[int, int, int],
        length: int = 12,
        thickness: int = 1,
        label: Optional[str] = None,
    ):
        """Hedef cercevesi (kompakt ve ince kose cizgileri)."""
        x1, y1, x2, y2 = int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))
        cv2.line(img, (x1, y1), (x1 + length, y1), color, thickness)
        cv2.line(img, (x1, y1), (x1, y1 + length), color, thickness)
        cv2.line(img, (x2, y1), (x2 - length, y1), color, thickness)
        cv2.line(img, (x2, y1), (x2, y1 + length), color, thickness)
        cv2.line(img, (x1, y2), (x1 + length, y2), color, thickness)
        cv2.line(img, (x1, y2), (x1, y2 - length), color, thickness)
        cv2.line(img, (x2, y2), (x2 - length, y2), color, thickness)
        cv2.line(img, (x2, y2), (x2, y2 - length), color, thickness)

        if label:
            cv2.putText(img, label, (x1, max(16, y1 - 6)), self.font, 0.40, color, 1, cv2.LINE_AA)

    def draw_reticle(self, frame: np.ndarray, cx: int, cy: int, color: Tuple[int, int, int]):
        """Merkez nisangah vizoru - Zarif, minimalist kucuk halka ve merkez nokta."""
        cv2.circle(frame, (cx, cy), 10, color, 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 1, color, -1, cv2.LINE_AA)

    def draw_lock_bar(self, frame: np.ndarray, cx: int, cy: int, lock_ratio: float, is_locked: bool):
        """Hizalanma dolum bari (zarif, ince ve kompakt)."""
        bar_w, bar_h = 70, 5
        bar_x = cx - (bar_w // 2)
        bar_y = cy + 18

        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (90, 100, 110), 1)
        fill_w = int((bar_w - 2) * np.clip(lock_ratio, 0.0, 1.0))
        if fill_w > 0:
            fill_color = self.C_GREEN if is_locked else self.C_AMBER
            cv2.rectangle(frame, (bar_x + 1, bar_y + 1), (bar_x + 1 + fill_w, bar_y + bar_h - 1), fill_color, -1)

    def draw_mode_banner(self, frame: np.ndarray, is_locked: bool = False, firing: bool = False):
        """Ust orta kontrol modu baslik banneri: [RL CONTROLLER]."""
        if is_locked or firing:
            return

        h, w, _ = frame.shape
        cx = w // 2
        banner_text = "[RL CONTROLLER]"
        banner_color = self.C_GREEN
        banner_border = (70, 80, 90)

        (b_w, _), _ = cv2.getTextSize(banner_text, self.font, 0.56, 2)
        banner_x = max(10, cx - (b_w // 2))

        pad_x, pad_y = 12, 5
        bx1, by1 = max(0, banner_x - pad_x), max(0, 26 - 16 - pad_y)
        bx2, by2 = min(w, banner_x + b_w + pad_x), min(h, 26 + pad_y)
        roi_b = frame[by1:by2, bx1:bx2]
        if roi_b.size > 0:
            black_b = np.zeros_like(roi_b)
            frame[by1:by2, bx1:bx2] = cv2.addWeighted(roi_b, 0.30, black_b, 0.70, 0)
        cv2.rectangle(frame, (bx1, by1), (bx2, by2), banner_border, 1)

        cv2.putText(frame, banner_text, (banner_x, 26), self.font, 0.56, banner_color, 2, cv2.LINE_AA)

    def draw_combat_panel(
        self,
        frame: np.ndarray,
        cx: int,
        cy: int,
        dist_m: float,
        speed_mps: float,
        tilt_offset_deg: float,
        firing: bool = False,
    ):
        """Hedefe kitlendikten sonra acilan sivil Atis Izni ve Balistik Parametreleri paneli (Kompakt)."""
        h, w, _ = frame.shape
        panel_color = self.C_GREEN if not firing else self.C_AMBER

        dist_cm = int(round(dist_m * 100.0)) if dist_m > 0.05 else 0
        dist_str = f"{dist_cm} cm" if dist_cm > 0 else "-- cm"
        speed_str = f"{speed_mps:.1f} m/s"
        tilt_str = f"+{tilt_offset_deg:.1f} deg ELEVATION"

        if firing:
            lines = [
                ("THROW IN PROGRESS...", 0.46, self.C_AMBER, 1),
                (f"SPEED: {speed_str}   |   DIST: {dist_str}", 0.38, self.C_WHITE, 1),
                (f"PITCH: {tilt_str}", 0.38, self.C_CYAN, 1),
            ]
        else:
            lines = [
                ("CUP ALIGNED - READY TO THROW", 0.46, self.C_GREEN, 1),
                (f"DIST: {dist_str}   |   SPEED: {speed_str}", 0.38, self.C_WHITE, 1),
                (f"PITCH: {tilt_str}", 0.38, self.C_CYAN, 1),
                ("[Y] THROW DART   |   [N] CANCEL", 0.36, self.C_AMBER, 1),
            ]

        max_w = 0
        for text, scale, _, thick in lines:
            (tw, _), _ = cv2.getTextSize(text, self.font, scale, thick)
            if tw > max_w:
                max_w = tw

        padding_x = 16
        panel_w = max_w + 2 * padding_x
        line_step = 20
        panel_h = 16 + len(lines) * line_step

        px1 = cx - (panel_w // 2)
        py1 = cy + 65
        px2 = px1 + panel_w
        py2 = py1 + panel_h

        px1, py1 = max(0, px1), max(0, py1)
        px2, py2 = min(w, px2), min(h, py2)

        roi = frame[py1:py2, px1:px2]
        if roi.size > 0:
            black_layer = np.zeros_like(roi)
            frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.35, black_layer, 0.65, 0)

        cv2.rectangle(frame, (px1, py1), (px2, py2), panel_color, 1)

        curr_y = py1 + 18
        for text, scale, color, thick in lines:
            (tw, _), _ = cv2.getTextSize(text, self.font, scale, thick)
            tx = cx - (tw // 2)
            cv2.putText(frame, text, (tx, curr_y), self.font, scale, color, thick, cv2.LINE_AA)
            curr_y += line_step

    def draw_telemetry(
        self,
        frame: np.ndarray,
        status_text: str,
        status_color: Tuple[int, int, int],
        dist_m: float,
        err_pan: float,
        err_tilt: float,
        target_detected: bool = False,
    ):
        """Sol ust sivil durum ve telemetri paneli (Kompakt ~205px genislik, ince 1px cerceve)."""
        h, w, _ = frame.shape

        # 1. Sivillestirilmis Durum Metni ve Rengi
        status_line = f"STATUS: {status_text}"
        dist_str = f"{dist_m:.2f} m" if dist_m > 0.05 else "--"
        sig_str = "STRONG" if dist_m > 0.05 else "NO LOCK"
        telemetry_line = f"RANGE: {dist_str}  |  SIGNAL: {sig_str}"
        angle_err_text = f"ANGULAR ERR: [{err_pan:+.1f}, {err_tilt:+.1f}] deg"

        title_scale = 0.55
        sub_scale = 0.50
        thick = 2

        (w_status, _), _ = cv2.getTextSize(status_line, self.font, title_scale, thick)
        (w_telemetry, _), _ = cv2.getTextSize(telemetry_line, self.font, sub_scale, 1)
        (w_angle, _), _ = cv2.getTextSize(angle_err_text, self.font, sub_scale, 1)

        max_text_w = max(w_status, w_telemetry, w_angle)

        px1, py1 = 15, 46
        px2 = px1 + max_text_w + 24
        py2 = py1 + 86

        px2 = min(px2, w)
        py2 = min(py2, h)

        roi = frame[py1:py2, px1:px2]
        if roi.size > 0:
            black_layer = np.zeros_like(roi)
            frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.35, black_layer, 0.65, 0)

        box_border_color = (0, 191, 255) if target_detected else (90, 90, 90)
        angle_color = (0, 255, 0) if abs(err_pan) < 1.0 and abs(err_tilt) < 1.0 else (200, 210, 220)
        text_color = (255, 255, 255) if target_detected else (170, 175, 180)

        cv2.rectangle(frame, (px1, py1), (px2, py2), box_border_color, 2)

        cv2.putText(frame, status_line, (26, py1 + 24), self.font, title_scale, status_color, thick, cv2.LINE_AA)
        cv2.putText(frame, telemetry_line, (26, py1 + 50), self.font, sub_scale, text_color, 1, cv2.LINE_AA)
        cv2.putText(frame, angle_err_text, (26, py1 + 72), self.font, sub_scale, angle_color, 1, cv2.LINE_AA)

    def draw_event_log(self, frame: np.ndarray, max_lines: int = 4, is_locked: bool = False, firing: bool = False):
        """Sol alt kosede terminaldeki sistem ve operator mesajlarini gosteren EVENT LOG kutusu."""
        if is_locked or firing or not self.event_logs:
            return

        display_logs = self.event_logs[-max_lines:]
        h, w, _ = frame.shape
        line_height = 18
        header_height = 20
        box_h = header_height + (len(display_logs) * line_height) + 8

        px1 = 15
        py2 = h - 30
        py1 = py2 - box_h

        max_txt_w = 220
        for item in display_logs:
            txt = str(item[0]) if isinstance(item, (list, tuple)) else str(item)
            (tw, _), _ = cv2.getTextSize(txt, self.font, 0.38, 1)
            if tw > max_txt_w:
                max_txt_w = tw

        px2 = min(w - 20, px1 + max_txt_w + 24)
        px1, py1 = max(0, px1), max(0, py1)
        px2, py2 = min(w, px2), min(h, py2)

        roi = frame[py1:py2, px1:px2]
        if roi.size > 0:
            black_layer = np.zeros_like(roi)
            frame[py1:py2, px1:px2] = cv2.addWeighted(roi, 0.25, black_layer, 0.75, 0)

        cv2.rectangle(frame, (px1, py1), (px2, py2), (75, 85, 95), 1)

        header_text = "EVENT LOG"
        cv2.putText(frame, header_text, (px1 + 10, py1 + 14), self.font, 0.36, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.line(frame, (px1 + 8, py1 + 18), (px2 - 8, py1 + 18), (55, 65, 75), 1)

        curr_y = py1 + 18 + line_height
        for item in display_logs:
            if isinstance(item, (list, tuple)):
                full_text = str(item[0])
                color = item[1] if len(item) > 1 else (200, 210, 220)
            else:
                color = (200, 210, 220)
                full_text = str(item)

            cv2.putText(frame, full_text, (px1 + 10, curr_y - 3), self.font, 0.38, color, 1, cv2.LINE_AA)
            curr_y += line_height

    def draw_footer(self, frame: np.ndarray, is_locked: bool = False, firing: bool = False, is_estop: bool = False):
        """Alt operator kontrol lejandi (Kompakt ve silik)."""
        if firing:
            return

        h, w, _ = frame.shape
        cx = w // 2

        if is_estop:
            footer_text = "[SPACE] RESUME SYSTEM  |  [Q / ESC] QUIT"
            footer_color = self.C_RED
        elif is_locked:
            footer_text = "[Y] Throw Dart  |  [N] Cancel  |  [SPACE] Safety Stop  |  [Q] Quit"
            footer_color = (150, 160, 170)
        else:
            footer_text = "[Y] Throw Dart  |  [SPACE] Safety Stop  |  [Q / ESC] Quit"
            footer_color = (160, 170, 180)

        (f_w, _), _ = cv2.getTextSize(footer_text, self.font, 0.42, 1)
        cv2.putText(
            frame,
            footer_text,
            (cx - f_w // 2, h - 14),
            self.font,
            0.42,
            footer_color,
            1,
            cv2.LINE_AA,
        )

    def render(
        self,
        frame: np.ndarray,
        cup_detected: bool = False,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        confidence: float = 0.0,
        distance_m: float = 0.0,
        err_pan_deg: float = 0.0,
        err_tilt_deg: float = 0.0,
        fps: float = 30.0,
        is_locked: bool = False,
        firing: bool = False,
        is_estop: bool = False,
        lock_ratio: float = 0.0,
        speed_mps: float = 10.0,
        tilt_offset_deg: float = 6.0,
        center_px: Optional[Tuple[int, int]] = None,
    ) -> np.ndarray:
        """Kare uzerine BasketPresenter uzerinden kompakt sivil robotik tasarim dilinde HUD cizer."""
        from turret_basket.basket_manager import BasketStatus
        from turret_basket.presenter import BasketPresenter

        status = BasketStatus(
            cup_detected=cup_detected,
            bbox=bbox,
            confidence=confidence,
            distance_m=distance_m,
            err_pan_deg=err_pan_deg,
            err_tilt_deg=err_tilt_deg,
            is_locked=is_locked,
            firing=firing,
            is_estop=is_estop,
            lock_ratio=lock_ratio,
            speed_mps=speed_mps,
            tilt_offset_deg=tilt_offset_deg,
            center_px=center_px,
        )
        return BasketPresenter(self).render(frame=frame, status=status, fps=fps)
