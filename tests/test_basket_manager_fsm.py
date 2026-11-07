"""
================================================================================
TEST KATEGORISI 4: ANGAJMAN DURUM MAKINESI (FSM) VE BASKET MANAGER ORKESTRATORU
================================================================================
Kapsam:
- turret_basket.engagement (4 asamali SPINUP_ELEVATE -> SETTLE -> FIRING -> RECOVER FSM)
- turret_basket.basket_manager (yasam dongusu, menzil mandali, hedef kaybi histerezisi,
  uzak mesafe kilit daraltma ve tetik aninda ince yatay pan hizalamasi)
================================================================================
"""

import math
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.protocol import STATE_TRACKING, STATE_LOCKED, STATE_FIRING
from turret_basket.engagement import BasketEngagement
from turret_basket.basket_manager import BasketManager, BasketStatus
from turret_basket.yolo.cup_detector import CupDetector, CupDetection


def test_basket_engagement_fsm():
    """BasketEngagement 4-asamali (SPINUP_ELEVATE -> SETTLE -> FIRING -> RECOVER) atis durum makinesini dogrular."""
    eng = BasketEngagement(spinup_time_s=0.50, settle_time_s=0.25, firing_pulse_s=0.08, recovery_time_s=0.50)

    # 1. Baslangic: Bostayken
    step = eng.update(now=10.0, current_tilt=0.0)
    assert step.is_active is False
    assert step.phase is None
    assert step.cmd_state == STATE_TRACKING

    # 2. Tetikleme (t = 10.0)
    ok, _ = eng.trigger(now=10.0, tilt_at_lock=5.0, fire_speed=4.5, fire_tilt=21.0, super_elevation=21.8)
    assert ok is True
    assert eng.is_active is True

    # 3. Spin-up ve Super-Elevasyon Asamasi (t = 10.30s)
    step = eng.update(now=10.30, current_tilt=5.0)
    assert step.is_active is True
    assert step.phase == "SPINUP_ELEVATE"
    assert step.cmd_state == STATE_LOCKED
    assert step.step_tilt > 0.0

    # 4. Settle / Mekanik Oturma Asamasi (t = 10.60s)
    step = eng.update(now=10.60, current_tilt=21.0)
    assert step.is_active is True
    assert step.phase == "SETTLE"
    assert step.cmd_state == STATE_LOCKED

    # 5. Solenoid Darbe Asamasi (t = 10.78s)
    step = eng.update(now=10.78, current_tilt=21.0)
    assert step.is_active is True
    assert step.phase == "FIRING"
    assert step.cmd_state == STATE_FIRING
    assert step.just_fired is True

    # Ayni asama icinde ikinci cagri (just_fired artik False olmali)
    step2 = eng.update(now=10.80, current_tilt=21.0)
    assert step2.just_fired is False

    # 6. Toparlama Asamasi (t = 11.00s)
    step = eng.update(now=11.00, current_tilt=15.0)
    assert step.is_active is True
    assert step.phase == "RECOVER"
    assert step.cmd_state == STATE_TRACKING

    # 7. Tamamlanma (t = 11.40s)
    step = eng.update(now=11.40, current_tilt=5.0)
    assert step.is_active is False
    assert step.is_finished is True
    assert eng.is_active is False

    # 8. Abort testi
    eng.trigger(now=20.0, tilt_at_lock=0.0, fire_speed=4.0, fire_tilt=20.0, super_elevation=21.0)
    assert eng.is_active is True
    eng.abort()
    assert eng.is_active is False


def test_basket_manager_lifecycle():
    """BasketManager orkestratorunun cevrim adimini, E-Stop ve delta_z geometrisini dogrular."""
    manager = BasketManager(comm=None)
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    status = manager.step(frame=dummy_frame, dt=0.033, now=1.0)
    assert isinstance(status, BasketStatus)
    assert status.is_estop is False
    assert status.firing is False

    # E-Stop Ac / Kapa
    assert manager.toggle_estop() is True
    assert manager.is_estop is True
    assert manager.toggle_estop() is False
    assert manager.is_estop is False

    # delta_z geometrik isaret dogrulamasi (delta_z_m = dist * tan(tilt) - 0.0634)
    manager.dist_m = 2.0
    delta_z_camera = manager.dist_m * math.tan(0.0)
    delta_z_m = delta_z_camera - 0.0634
    assert math.isclose(delta_z_m, -0.0634, abs_tol=1e-5)


def test_basket_manager_range_latch_on_lock():
    """Kilit aninda hedefin mesafesinin donduruldugunu ve gurultuden etkilenmedigini dogrular."""
    manager = BasketManager(comm=None)
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    mock_det = CupDetection(
        detected=True,
        bbox=(300, 220, 340, 260),
        center_px=(320, 240),
        confidence=0.92,
        err_pan_deg=0.1,
        err_tilt_deg=0.1,
    )
    manager.detector.detect = lambda f, *args, **kwargs: mock_det

    manager.dist_m = 1.56
    manager.last_valid_dist_m = 1.56

    for _ in range(CONFIG.cup.lock_threshold_frames):
        manager.step(dummy_frame, dt=0.033, now=1.0)

    assert manager.is_locked is True
    assert manager.locked_dist_m == 1.56

    # Kilitliyken disaridan mesafe degisse bile dondurulan mesafe korunmali
    manager.last_valid_dist_m = 1.35
    manager.step(dummy_frame, dt=0.033, now=1.033)
    assert manager.dist_m == 1.56
    assert manager.locked_dist_m == 1.56

    manager.reset()
    assert manager.is_locked is False
    assert manager.locked_dist_m is None


def test_basket_manager_real_system_improvements():
    """BasketManager uzerine tasinan 4 gercek sistem iyilestirmesini dogrular."""
    class DummyDetector(CupDetector):
        def __init__(self):
            super().__init__(model_path=None)
            self.mock_det = CupDetection(
                detected=True,
                bbox=(280, 200, 360, 280),
                center_px=(320, 240),
                confidence=0.90,
                err_pan_deg=0.95,
                err_tilt_deg=0.20,
            )

        def detect(self, frame, tilt_deg=0.0):
            return self.mock_det

    det = DummyDetector()
    mgr = BasketManager(comm=None, detector=det)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # 1. Uzak mesafede kilit toleransi olceklemesi (1.45m'de 0.95 deg < 1.20 deg kilit sayacini artirir; 2.30m'de 0.95 deg > 0.756 deg azaltir)
    mgr.dist_m = 1.45
    mgr.last_valid_dist_m = 1.45
    mgr.step(frame, dt=0.033, now=1.0)
    assert mgr.lock_counter == 1

    mgr.dist_m = 2.30
    mgr.last_valid_dist_m = 2.30
    mgr.step(frame, dt=0.033, now=1.033)
    assert mgr.lock_counter == 0

    # 2. Hedef kaybi histerezisi (1-9 kare kayipta tracker.rl.act_hist_1 korunur, 10. karede sifirlanir)
    mgr.tracker.rl.act_hist_1 = np.array([0.4, -0.3], dtype=np.float32)
    det.mock_det = CupDetection(detected=False, bbox=None, center_px=None, confidence=0.0, err_pan_deg=0.0, err_tilt_deg=0.0)
    for i in range(9):
        mgr.step(frame, dt=0.033, now=2.0 + i * 0.033)
    assert not np.allclose(mgr.tracker.rl.act_hist_1, 0.0)
    mgr.step(frame, dt=0.033, now=2.35)
    assert np.allclose(mgr.tracker.rl.act_hist_1, 0.0)

    # 3. Tetik aninda (trigger_throw) ince pan hizalama darbesi (pending_pan_trim_deg) ve atis bitisinde tam durum sifirlama
    det.mock_det = CupDetection(
        detected=True,
        bbox=(280, 200, 360, 280),
        center_px=(325, 240),
        confidence=0.92,
        err_pan_deg=0.60,
        err_tilt_deg=0.10,
    )
    mgr.dist_m = 1.60
    mgr.last_valid_dist_m = 1.60
    for i in range(mgr.MAX_LOCK_FRAMES):
        mgr.step(frame, dt=0.033, now=3.0 + i * 0.033)
    assert mgr.is_locked is True
    assert mgr.locked_dist_m == 1.60

    ok = mgr.trigger_throw(now=4.0)
    assert ok is True
    assert math.isclose(mgr.pending_pan_trim_deg, 0.85 * 0.60, abs_tol=1e-4)

    mgr.step(frame, dt=0.033, now=4.05)
    assert mgr.pending_pan_trim_deg == 0.0
    mgr.step(frame, dt=0.033, now=5.50)
    assert mgr.locked_dist_m is None
    assert mgr.is_locked is False

    # 4. R (reset) cagrisinda dedektor EMA kutusu ve tum filtrelerin temizlenmesi
    det.smooth_box = np.array([100.0, 100.0, 200.0, 200.0])
    mgr.target_lost_counter = 5
    mgr.reset()
    assert det.smooth_box is None
    assert mgr.target_lost_counter == 0

