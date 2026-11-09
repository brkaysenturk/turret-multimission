"""Uretim RL kontrolcusu konsolide testleri."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager
from turret_fcs.rl_controller import RLController, rate_scaled_step_limits


class _FakeModel:
    def __init__(self, action):
        self.action = np.asarray(action, dtype=np.float32)
        self.last_obs = None

    def predict(self, obs, deterministic=True):
        self.last_obs = np.asarray(obs, dtype=np.float32)
        return self.action, None


def test_rl_policy_loading_and_manager_wiring():
    """FCSManager entegrasyonu, NumPy politika yuklenmesi ve guvenlik limitlerini dogrular."""
    # 1. FCSManager wiring
    manager = FCSManager("RL")
    assert isinstance(manager.rl_controller, RLController)

    # 2. Varsayilan NumPy politika modeli
    rl = RLController()
    assert rl.is_loaded, rl.load_error
    assert rl.policy_format == "numpy"
    assert (rl.model.obs_dim, rl.model.act_dim) == (9, 2)
    action, _ = rl.model.predict(np.zeros(9, dtype=np.float32))
    assert action.shape == (2,)
    assert np.all(np.isfinite(action))
    assert np.all(np.abs(action) <= 1.0)

    # 3. Yuklenmemis denetleyici guvenligi ve P-blend yoklugu
    unloaded = RLController(auto_load=False)
    assert unloaded.is_loaded is False
    assert unloaded.predict_angles(700, 300, 640, 360, 1280, 720, 0, 0, 2.0, 0, 0, 0.033) == (0.0, 0.0)
    assert not hasattr(unloaded, "blend_with_p")
    assert not hasattr(unloaded, "blend_commands")


def test_rl_observation_and_action_history():
    """9-Boyutlu durum gozlem vektoru ve 2-adimli eylem gecmisi akisini dogrular."""
    fake = _FakeModel([0.5, -0.25])
    rl = RLController(
        auto_load=False, soft_deadband_enabled=False,
        cmd_accel_pan_deg_s2=2000.0, cmd_accel_tilt_deg_s2=2000.0,
    )
    rl.model = fake
    rl.is_loaded = True

    d_pan, d_tilt = rl.predict_angles(
        700.0, 360.0, 640.0, 360.0, 1280.0, 720.0,
        0.0, 0.0, 2.0, 0.0, 0.0, 0.03333,
    )
    assert fake.last_obs.shape == (9,)
    assert fake.last_obs[0] == pytest.approx(60.0 / 640.0)
    assert d_pan == pytest.approx(0.5 * CONFIG.actuator.max_step_pan_deg)
    assert d_tilt == pytest.approx(-0.25 * CONFIG.actuator.max_step_tilt_deg)
    assert rl.act_hist_1 == pytest.approx([0.5, -0.25], abs=1e-6)


def test_rl_deadband_taper_and_reversal_damping():
    """Olu bolge, yumusak taper, ivme limiti ve terslenme sonumunu dogrular."""
    fake = _FakeModel([1.0, 1.0])
    rl = RLController(
        auto_load=False,
        soft_deadband_enabled=True,
        reversal_damping=0.5,
        deadband_pan_deg=0.85,
        deadband_tilt_deg=0.95,
        taper_pan_deg=1.00,
        cmd_accel_pan_deg_s2=5000.0,
        cmd_accel_tilt_deg_s2=5000.0,
    )
    rl.model = fake
    rl.is_loaded = True

    # 1. Olu bolge icinde (0.50 deg): sifir komut (tam kilit)
    d_pan, d_tilt = rl.predict_angles(642.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 0.50, 0.50, 0.033)
    assert d_pan == 0.0
    assert d_tilt == 0.0
    assert rl._in_deadband is True

    # 2. Taper bolgesinde (1.35 deg = db + 0.50): komut yumusatilir
    d_pan_taper, _ = rl.predict_angles(700.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 1.35, 1.35, 0.033)
    assert 0.0 < d_pan_taper < CONFIG.actuator.max_step_pan_deg

    # 3. Buyuk hata, ayni yon: tam aksiyon
    d_pan, d_tilt = rl.predict_angles(800.0, 200.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 5.0, 5.0, 0.033)
    assert d_pan == pytest.approx(CONFIG.actuator.max_step_pan_deg, abs=0.02)
    assert d_tilt == pytest.approx(CONFIG.actuator.max_step_tilt_deg, abs=0.02)

    # 4. Hizli yon degisimi: aksiyon yariya indirilir, rezonans sonumlenir
    fake.action = np.array([-1.0, -1.0], dtype=np.float32)
    d_pan_rev, d_tilt_rev = rl.predict_angles(800.0, 200.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 5.0, 5.0, 0.033)
    assert d_pan_rev == pytest.approx(-0.5 * CONFIG.actuator.max_step_pan_deg, abs=0.02)
    assert d_tilt_rev == pytest.approx(-0.5 * CONFIG.actuator.max_step_tilt_deg, abs=0.02)

    # 5. Ivme sinirlama testi (accel * dt^2)
    rl_accel = RLController(auto_load=False, cmd_accel_pan_deg_s2=10.0)
    rl_accel.model = fake
    rl_accel.is_loaded = True
    d1, _ = rl_accel.predict_angles(800.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 5.0, 0.0, 0.03333)
    assert d1 == pytest.approx(-10.0 * (0.03333 ** 2), abs=1e-5)


def test_rl_reset_and_rate_scaling():
    """Hedef kaybi, durum sifirlama ve kare hizina gore adim limitlerini dogrular."""
    fake = _FakeModel([0.4, 0.2])
    rl = RLController(auto_load=False)
    rl.model = fake
    rl.is_loaded = True

    # Eylem gecmisi olustur
    rl.predict_angles(700.0, 300.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.03333)
    assert np.any(rl.act_hist_1 != 0.0)

    # Hedef kaybinda gecmis temizlenmeli
    rl.on_target_lost()
    assert np.all(rl.act_hist_1 == 0.0)
    assert rl.has_prev_target is False

    # Reset testi
    rl.predict_angles(700.0, 300.0, 640.0, 360.0, 1280.0, 720.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.03333)
    rl.reset()
    assert np.all(rl.act_hist_1 == 0.0)
    assert rl.last_action.tolist() == [0.0, 0.0]

    # Kare hizi olcek semantigi
    assert rate_scaled_step_limits(0.80, 0.50, 0.03333)[:2] == pytest.approx((0.80, 0.50), abs=1e-3)
    fast = rate_scaled_step_limits(0.80, 0.50, 0.01667)
    assert fast[:2] == pytest.approx((0.40, 0.25), abs=1e-3)
    slow = rate_scaled_step_limits(0.80, 0.50, 0.06666)
    assert slow[0] <= CONFIG.control.max_output_deg
    assert slow[1] <= CONFIG.control.max_output_deg



def test_rl_asymmetric_braking_eliminates_overshoot():
    """Oneri 1: RL kontrolcu tam hizla giderken hedef olu bolgeye girdiginde aninda 0.0 urettigini dogrular."""
    fake = _FakeModel([1.0, 1.0])
    rl = RLController(auto_load=False, soft_deadband_enabled=True, asymmetric_braking=True)
    rl.model = fake
    rl.is_loaded = True

    # Tam hiza cikana kadar 5 kare ilerle
    for _ in range(5):
        d_pan, _ = rl.predict_angles(800.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 10.0, 0.0, 0.03333)
    assert d_pan == pytest.approx(CONFIG.actuator.max_step_pan_deg, abs=1e-4)

    # Hedef merkeze (0.40 deg <= 0.85 deg deadband) girdigi anda aninda durmali (sifir overshoot)
    d_stop, _ = rl.predict_angles(642.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 0.40, 0.0, 0.03333)
    assert d_stop == 0.0
    assert rl.command_speed_pan_deg_s == 0.0

    # Smith Predictor (_compensate_in_flight) cikisi sonumlerken gozlemdeki hata turevini (d_err) bozmamali:
    rl_smith = RLController(auto_load=False, soft_deadband_enabled=True, in_flight_damping=0.6875)
    rl_smith.model = fake
    rl_smith.is_loaded = True
    rl_smith.predict_angles(680.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 1.60, 0.0, 0.03333)
    # Kameradaki hedef sabitken (bx=680.0) in-flight gecmisi degisse bile ham d_err_x 0.0 kalmali!
    rl_smith.cmd_hist_pan_1 = 0.0
    rl_smith.cmd_hist_pan_2 = 0.0
    rl_smith.cmd_hist_pan_3 = 0.0
    rl_smith.predict_angles(680.0, 360.0, 640.0, 360.0, 1280.0, 720.0, 0, 0, 2.0, 1.60, 0.0, 0.03333)
    assert fake.last_obs[2] == pytest.approx(0.0, abs=1e-6)
