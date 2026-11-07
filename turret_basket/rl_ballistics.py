"""
================================================================================
BASKET CHALLENGE: 2. MODEL - BALISTIK EGIM VE HIZ MOTORU (RL BALLISTICS)
================================================================================
ToF LiDAR menziline gore gereken ek super-elevasyon acisini (+21.8 deg - +22.6 deg)
ve volan motoru hizini (4.4 m/s) anlik kestiren 10.000 yorungeli fizik gudumlu
RL Balistik Politika motoru.
================================================================================
"""

from pathlib import Path
from typing import Optional, NamedTuple
import numpy as np

from turret_basket.config import CONFIG


class LaunchParameters(NamedTuple):
    tilt_offset_deg: float
    speed_mps: float


class RLBallistics:
    """2. Model: Kupa mesafesine gore atis egimini ve volan hizini kestiren balistik AI motoru."""

    def __init__(self, models_dir: Optional[Path] = None, model_name: str = "cup_ballistic_policy.npz"):
        if models_dir is None:
            models_dir = Path(__file__).resolve().parent / "rl" / "models"
        self.models_dir = Path(models_dir)
        self.model_name = model_name
        self.predict_fn = self._load_model()

        self.tilt_min = CONFIG.rl.tilt_offset_min_deg
        self.tilt_max = CONFIG.rl.tilt_offset_max_deg
        self.speed_min = CONFIG.rl.speed_min_mps
        self.speed_max = CONFIG.rl.speed_max_mps
        self.default_parallax_margin_deg = CONFIG.ballistics.parallax_pitch_margin_deg

    def _load_model(self):
        """2. Model: Hizli ve bagimsiz NumPy MLP Balistik Politika yukleyicisi."""
        model_npz = self.models_dir / self.model_name
        model_zip = self.models_dir / "cup_ballistic_policy.zip"

        if model_npz.exists():
            try:
                data = np.load(str(model_npz))
                w1, b1, w2, b2, w3, b3 = (
                    data["w1"],
                    data["b1"],
                    data["w2"],
                    data["b2"],
                    data["w3"],
                    data["b3"],
                )

                def predict_npz(obs: np.ndarray) -> np.ndarray:
                    h1 = np.tanh(np.dot(w1, obs) + b1)
                    h2 = np.tanh(np.dot(w2, h1) + b2)
                    return np.clip(np.dot(w3, h2) + b3, -1.0, 1.0)

                return predict_npz
            except Exception:
                pass

        if model_zip.exists():
            try:
                from stable_baselines3 import PPO

                sb3_model = PPO.load(str(model_zip), device="cpu")
                return lambda obs: sb3_model.predict(obs, deterministic=True)[0]
            except Exception:
                pass

        # Fallback: Nominal analitik yaklasim
        return lambda obs: np.array([0.75, -0.65], dtype=np.float32)

    def compute_launch_parameters(self, dist_m: float, delta_z_m: float = -0.2287) -> LaunchParameters:
        """Menzile (metre) ve kupa dikey kot farkina (delta_z) gore gereken balistik egimi ve volan hizini hesaplar."""
        # Ultra-genisletilmis normalizasyon: [1.0m .. 3.1m] ve [-0.85m .. +0.35m]
        norm_dist = np.clip((dist_m - 2.05) / 1.05, -1.0, 1.0)
        norm_h = np.clip((delta_z_m - (-0.25)) / 0.60, -1.0, 1.0)
        obs = np.array([norm_dist, norm_h], dtype=np.float32)
        act = self.predict_fn(obs)

        # Aci: [5.0° .. 30.0°] -> center 17.5, span 12.5
        # Hiz: [2.0 m/s .. 12.0 m/s] -> center 7.0, span 5.0
        target_tilt_offset = float(np.clip(17.5 + act[0] * 12.5, self.tilt_min, self.tilt_max))
        target_speed_mps = float(np.clip(7.0 + act[1] * 5.0, self.speed_min, self.speed_max))
        return LaunchParameters(target_tilt_offset, target_speed_mps)

    def compute_super_elevation(
        self,
        tilt_offset_deg: float,
        parallax_margin_deg: Optional[float] = None,
    ) -> float:
        """Model 2'nin hesapladigi mutlak atis egim acisini mekanik sinirlarla dondurur."""
        margin = self.default_parallax_margin_deg if parallax_margin_deg is None else parallax_margin_deg
        return float(np.clip(tilt_offset_deg + margin, self.tilt_min, self.tilt_max))

