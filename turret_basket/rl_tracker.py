"""
================================================================================
BASKET CHALLENGE: 1. MODEL - KUPA KILITLENME VE TAKIP MOTORU (RL TRACKER)
================================================================================
turret-fcs projesinde egitilen ve Webots donaniminda dogrulanmis RLController
(PPO 9-64-64-2 NumPy MLP) mimarisini birebir kullanan 1. Asama Gorsel Takip Motoru.
- Bardagi optik eksenin tam merkezine (err_x -> 0, err_y -> 0) kilitler.
- In-flight komut sonumleme, soft deadband taper ve ivme sinirlayici icerir.
================================================================================
"""

from pathlib import Path
import json
from typing import Optional, Tuple

from turret_fcs.rl_controller import RLController


class RLTracker:
    """1. Model: Kupaya gorsel servo ile kilitlenen RL takip kontrolcusu."""

    def __init__(self, model_path: Optional[str] = None, params_path: Optional[str] = None):
        if model_path is None:
            model_path = str(Path(__file__).resolve().parent / "rl" / "models" / "cup_tracker_policy.npz")

        # Optuna tarafindan statik kupa icin optimize edilmis hiperparametreler
        if params_path is None:
            params_file = Path(__file__).resolve().parent / "rl" / "models" / "cup_tracker_params.json"
        else:
            params_file = Path(params_path)

        params = {}
        if params_file.exists():
            try:
                with open(params_file, "r", encoding="utf-8") as f:
                    params = json.load(f)
            except Exception:
                pass

        self.deadband_pan_deg = float(params.get("deadband_pan_deg", 0.1948))
        self.deadband_tilt_deg = float(params.get("deadband_tilt_deg", 0.1522))
        self.taper_pan_deg = float(params.get("taper_pan_deg", 0.1844))
        self.taper_tilt_deg = float(params.get("taper_tilt_deg", 0.1222))
        self.in_flight_damping = float(params.get("in_flight_damping", 0.3907))
        self.reversal_damping = float(params.get("reversal_damping", 0.3351))
        self.aim_mouth_ratio = float(params.get("aim_mouth_ratio", 0.3518))

        # turret_fcs'in optimize edilmis RLController sinifi
        self.rl = RLController(
            model_path=model_path,
            deadband_pan_deg=self.deadband_pan_deg,
            deadband_tilt_deg=self.deadband_tilt_deg,
            taper_pan_deg=self.taper_pan_deg,
            taper_tilt_deg=self.taper_tilt_deg,
            in_flight_damping=self.in_flight_damping,
            reversal_damping=self.reversal_damping,
            soft_deadband_enabled=True,
        )

    def reset(self):
        """Hedef kayboldugunda veya yeni hedefte gecmisi temizle."""
        self.rl.reset()

    def on_target_lost(self):
        """Hedef kaybinda gecmisi temizle."""
        self.rl.on_target_lost()

    def compute(
        self,
        cx: float,
        cy: float,
        center_x: float,
        center_y: float,
        screen_w: float,
        screen_h: float,
        distance_m: float = 1.45,
        dt: float = 0.033,
    ) -> Tuple[float, float, bool]:
        """
        Kupa merkez koordinatina gore taret Pan ve Tilt motor adimlarini hesaplar.
        Donus: (delta_pan_deg, delta_tilt_deg, is_locked)
        """
        d_pan, d_tilt = self.rl.predict_angles(
            bx=float(cx),
            by=float(cy),
            center_x=float(center_x),
            center_y=float(center_y),
            screen_w=float(screen_w),
            screen_h=float(screen_h),
            vx=0.0,
            vy=0.0,
            dist_m=float(distance_m),
            dt=float(dt),
        )

        # Hedefe kilitlenme kriteri: Uzak mesafede kupa agzinin acisal capi kuculdugu icin
        # menzil uzadikca kilit penceresi orantili daraltilir (sensor gerektirmez, salt kamera + LiDAR).
        dist_scale = min(1.0, max(0.55, 1.45 / max(1.20, float(distance_m))))
        err_x_ratio = abs(cx - center_x) / (screen_w / 2.0)
        err_y_ratio = abs(cy - center_y) / (screen_h / 2.0)
        is_locked = (err_x_ratio < (0.045 * dist_scale) and err_y_ratio < (0.065 * dist_scale))

        return float(d_pan), float(d_tilt), is_locked
