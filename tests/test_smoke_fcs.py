"""FCSManager duman ve entegrasyon testleri."""
import numpy as np
import pytest

from turret_fcs.config import CONFIG
from turret_fcs.fcs_manager import FCSManager


def test_fcs_lifecycle_tracking_and_lock_recovery():
    fcs = FCSManager(control_mode="P")
    assert fcs.control_mode in ("P", "RL")
    assert fcs.PAN_LIMIT_MIN < fcs.PAN_LIMIT_MAX
    assert (fcs.target_velocity[0], fcs.target_velocity[1]) == (0.0, 0.0)

    # Taret açı senkronizasyonu
    fcs.sync_turret_pose(15.5, -5.2)
    assert (fcs.turret_pan_deg, fcs.turret_tilt_deg) == pytest.approx((15.5, -5.2))

    # Tekil hedef güncelleme ve komut sınırlandırma
    pan_cmd, tilt_cmd = fcs.update(
        [620, 340, 660, 380], 640, 360, 1100.0, 1100.0, 0.033,
        is_firing=False, dist_cm=200,
    )
    assert abs(pan_cmd) <= CONFIG.control.max_output_deg
    assert abs(tilt_cmd) <= CONFIG.control.max_output_deg

    # Kararlı takip ile kilitlenme (lock hysteresis)
    box = [630, 350, 650, 370]
    for _ in range(15):
        fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, is_firing=False, dist_cm=250)
    assert fcs.is_locked is True

    # Atış sırasında kilit korunumu
    fcs.update(box, 640, 360, 1100.0, 1100.0, 0.033, is_firing=True, dist_cm=250)
    assert fcs.is_locked is True

    # Hedef kaybı ve kilidin düşmesi
    fcs.update(None, 640, 360, 1100.0, 1100.0, 0.033)
    assert fcs.frames_since_lost > 0

    for _ in range(fcs.max_lost_frames + 2):
        fcs.update(None, 640, 360, 1100.0, 1100.0, 0.033)
    assert fcs.is_locked is False


def test_fcs_ballistic_slew_and_lead():
    fcs = FCSManager(control_mode="P")
    fx = fy = 1100.0
    box = [620, 340, 660, 380]

    # Balistik süper-elevasyon ve slew factor etkisi
    fcs.update(box, 640, 360, fx, fy, 0.033, dist_cm=300, slew_factor=0.0)
    assert fcs.prev_applied_lead_tilt == 0.0
    _, tilt_cmd = fcs.update(box, 640, 360, fx, fy, 0.033, is_firing=True, dist_cm=300, slew_factor=0.3)
    assert fcs.prev_applied_lead_tilt > 0.0
    assert tilt_cmd > 0.0

    # Hareketli hedef için Kalman lead kestirimi
    fcs = FCSManager(control_mode="P")
    for i in range(30):
        bx = 640.0 + 100.0 * 0.033 * i
        target_box = [bx - 20, 340, bx + 20, 380]
        fcs.update(target_box, 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)

    lead_time = fcs.lead_time
    measured_x = (target_box[0] + target_box[2]) // 2
    measured_y = (target_box[1] + target_box[3]) // 2
    assert fcs.pred_center == pytest.approx(
        [measured_x + fcs.target_velocity[0] * lead_time,
         measured_y + fcs.target_velocity[1] * lead_time],
        abs=1e-6,
    )

    # Kisa sureli (3 kare) hedef kaybinda coasting (predict_only) sayesinde yeniden yakalamada hiz sicramaz
    vel_before_dropout = float(fcs.target_velocity[0])
    for j in range(1, 4):
        fcs.update(None, 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)
    bx_reacquired = 640.0 + 100.0 * 0.033 * 33
    fcs.update([bx_reacquired - 20, 340, bx_reacquired + 20, 380], 640, 360, 1411.0, 1411.0, 0.033, dist_cm=200)
    assert fcs.target_velocity[0] == pytest.approx(vel_before_dropout, rel=0.08)


def test_dual_state_pose_architecture():
    """Cift Durum Poz Mimarisi (Commanded Pose vs Sensed HW Pose) dogrulamasi."""
    fcs = FCSManager(control_mode="P")

    # 1. Ilk acilista senkronizasyon (Initial Sync)
    fcs.sync_turret_pose(10.0, 5.0)
    assert fcs.turret_pan_deg == pytest.approx(10.0)
    assert fcs.turret_tilt_deg == pytest.approx(5.0)
    assert fcs.hw_pan_deg == pytest.approx(10.0)
    assert fcs.hw_tilt_deg == pytest.approx(5.0)
    assert not fcs.is_stalled

    # 2. Komut uygulandiginda komut pozisyonu ilerler
    fcs._integrate_command(1.5, 0.5)
    assert fcs.turret_pan_deg == pytest.approx(11.5)
    assert fcs.turret_tilt_deg == pytest.approx(5.5)

    # 3. Gecikmeli donanim telemetrisi komut pozisyonunu ezmez
    fcs.sync_turret_pose(10.2, 5.1)
    assert fcs.hw_pan_deg == pytest.approx(10.2)
    assert fcs.hw_tilt_deg == pytest.approx(5.1)
    # turret_pan_deg komut edilen 11.5'te kalir (ezilmez!)
    assert fcs.turret_pan_deg == pytest.approx(11.5)
    assert fcs.tracking_lag_deg == pytest.approx(np.hypot(1.3, 0.4), abs=1e-3)
    assert not fcs.is_stalled

    # 4. Asiri farkta (> 8.0 deg) Stall korumasi devreye girer
    fcs.sync_turret_pose(0.0, 0.0)
    assert fcs.tracking_lag_deg > 8.0
    assert fcs.is_stalled

    # 5. force=True ile zorunlu donanim senkronizasyonu
    fcs.sync_turret_pose(0.0, 0.0, force=True)
    assert fcs.turret_pan_deg == pytest.approx(0.0)
    assert fcs.turret_tilt_deg == pytest.approx(0.0)
    assert not fcs.is_stalled


def test_patrol_continuous_return_dt_slew_and_transient_immunity():
    """Devriye modunda kesintisiz merkeze donus, dt-olcekli slew ve sahte tespit bagisikligini dogrular."""
    fcs = FCSManager(control_mode="P", patrol_enabled=True)
    fcs.sync_turret_pose(12.0, -4.0, force=True)

    # 1. 1.5s HOLDING sonrasi RETURNING asamasinda her karede 0.0 olmadan kesintisiz adim uretilmeli
    for _ in range(46):
        fcs.update(None, 640, 360, 1100.0, 1100.0, 1.0 / 30.0)
    assert fcs.patrol.state == "RETURNING"

    steps_pan = []
    for _ in range(6):
        dp, _ = fcs.update(None, 640, 360, 1100.0, 1100.0, 1.0 / 30.0)
        steps_pan.append(dp)
    assert all(dp < -0.5 for dp in steps_pan), "RETURNING adimlari dur-kalk (0.0) yapmadan kesintisiz olmali"

    # 2. SCANNING asamasinda kisa dt (8 ms) geldiginde adim STM32 hiz siniriyla (45 deg/s * dt) olceklenmeli
    fcs.patrol.state = "SCANNING"
    fcs.patrol.scan_time = 2.0
    dp_short, dt_short, _ = fcs.patrol.update(0.008, current_pan=0.0, current_tilt=0.0)
    expected_limit = 1.5 * (0.008 / CONFIG.rl.nominal_dt_s)
    assert abs(dp_short) == pytest.approx(expected_limit, rel=1e-3)
    assert abs(dt_short) <= expected_limit + 1e-6

    # 3. SCANNING sirasinda 1-2 karelik gecici (sahte) tespit taramayi WAITING (1.5s donma) durumuna dusurmemeli
    saved_scan_time = fcs.patrol.scan_time
    box = [630, 350, 650, 370]
    fcs.update(box, 640, 360, 1100.0, 1100.0, 1.0 / 30.0, dist_cm=200)
    fcs.update(box, 640, 360, 1100.0, 1100.0, 1.0 / 30.0, dist_cm=200)
    assert fcs.patrol.state == "SCANNING"
    assert fcs.patrol.scan_time == pytest.approx(saved_scan_time)

    # Sahte tespit kayboldugunda 1.5s HOLDING'e girmeden dogrudan FIGURE-8 SCAN devam etmeli
    fcs.update(None, 640, 360, 1100.0, 1100.0, 1.0 / 30.0)
    assert fcs.patrol.state == "SCANNING"
    assert "FIGURE-8 SCAN" in fcs.patrol_status

    # 4. Gercek hedef (3+ ardisik kare) yakalandiginda ise devriye WAITING durumuna sifirlanmali
    for _ in range(3):
        fcs.update(box, 640, 360, 1100.0, 1100.0, 1.0 / 30.0, dist_cm=200)
    assert fcs.patrol.state == "WAITING"


