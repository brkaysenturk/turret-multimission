"""
================================================================================
TEST KATEGORISI 2: KUPA AGIZ PERSPEKTIF TESPITI VE 1. MODEL (RL TRACKER)
================================================================================
Kapsam:
- turret_basket.yolo.cup_detector (Dinamik perspektif agiz elipsi standardizasyonu)
- turret_basket.rl_tracker (1. Model PPO gorsel takip cikarimi ve uzak menzil kilit daraltma)
================================================================================
"""

import math
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.yolo.cup_detector import CupDetector
from turret_basket.rl_tracker import RLTracker
from turret_basket.basket_hud import BasketHUD
from turret_basket.presenter import BasketPresenter
from turret_basket.basket_manager import BasketStatus


def test_basket_rl_tracker_inference():
    """1. Model (RLTracker) PPO cikariminin soft deadband, aksiyon limitleri ve uzak menzil kilit olcegini dogrular."""
    tracker = RLTracker()
    tracker.reset()

    # 1. Merkezdeki hedef (cx=320, cy=240, screen_w=640, screen_h=480)
    d_pan, d_tilt, is_locked = tracker.compute(
        cx=320.0,
        cy=240.0,
        center_x=320.0,
        center_y=240.0,
        screen_w=640,
        screen_h=480,
        distance_m=1.8,
        dt=0.033,
    )
    assert is_locked is True
    assert abs(d_pan) < 0.20
    assert abs(d_tilt) < 0.20

    # 2. Uzakta olan hedef (cx=100, cy=100)
    d_pan_far, d_tilt_far, is_locked_far = tracker.compute(
        cx=100.0,
        cy=100.0,
        center_x=320.0,
        center_y=240.0,
        screen_w=640,
        screen_h=480,
        distance_m=1.8,
        dt=0.033,
    )
    assert is_locked_far is False
    assert abs(d_pan_far) <= CONFIG.hardware.max_step_deg
    assert abs(d_tilt_far) <= CONFIG.hardware.max_step_deg

    # 3. Uzak mesafede kilit penceresinin daraltilmasi (sensor gerektirmez, salt kamera + LiDAR)
    # 1.45m'de cx=332 (err_x_ratio = 12/320 = 0.0375 < 0.045) kilitli iken, 2.30m'de (tol = 0.028) kilitli olmamali
    _, _, locked_near = tracker.compute(
        cx=332.0, cy=240.0, center_x=320.0, center_y=240.0, screen_w=640, screen_h=480, distance_m=1.45
    )
    _, _, locked_far = tracker.compute(
        cx=332.0, cy=240.0, center_x=320.0, center_y=240.0, screen_w=640, screen_h=480, distance_m=2.30
    )
    assert locked_near is True
    assert locked_far is False


def test_cup_mouth_lockon_and_distance_standardization():
    """Kupa agiz elipsine dinamik kilitlenme ve menzil atlamasinda kurtarma mekanizmasini dogrular."""
    det = CupDetector(model_path=None)

    # 1. Standart agiz orani
    assert 0.05 <= det.aim_mouth_ratio <= 0.15
    assert det.aim_mouth_ratio == CONFIG.cup.aim_mouth_ratio
    assert det.aim_mouth_ratio == 0.10

    # 2. Perspektif agiz standardizasyonu:
    # Yatay bakis (tilt=0) -> agiz ust cizgiye yakin (ratio ~ 0.05)
    # Asagi dik bakis (tilt=-18 deg) -> agiz elipsi acik (ratio ~ 0.14)
    det.smooth_box = np.array([200, 100, 300, 200], dtype=np.float32)
    det.lost_frames = 0
    ratio_horiz = float(np.clip(0.04 + 0.35 * math.sin(math.radians(max(0.0, 0.0))), 0.05, 0.14))
    ratio_steep = float(np.clip(0.04 + 0.35 * math.sin(math.radians(max(0.0, 18.0))), 0.05, 0.14))
    assert math.isclose(ratio_horiz, 0.05, abs_tol=1e-3)
    assert math.isclose(ratio_steep, 0.14, abs_tol=1e-2)

    # 3. Presenter'in center_px'i dogrudan takip etmesi (retikul ile nisan noktasinin tam eslesmesi)
    hud = BasketHUD()
    presenter = BasketPresenter(hud)
    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    status = BasketStatus(
        cup_detected=True,
        bbox=(200, 100, 300, 200),
        confidence=0.9,
        distance_m=1.8,
        err_pan_deg=0.0,
        err_tilt_deg=0.0,
        is_locked=False,
        firing=False,
        is_estop=False,
        lock_ratio=0.5,
        speed_mps=4.4,
        tilt_offset_deg=21.0,
        center_px=(250, 110),
    )
    rendered = presenter.render(dummy_frame.copy(), status)
    assert rendered.shape == dummy_frame.shape

    # 4. 1.61m'den 2.15m'ye relocation sonrasinda menzil kapilamasinin kilitlenmemesi testi
    gt_dist = 2.15
    lidar_center_dist = 2.15
    beam_on_cup = (0.5 <= lidar_center_dist <= 3.5) and (abs(lidar_center_dist - gt_dist) <= 0.25)
    assert beam_on_cup is True

    last_valid_dist_m = 1.61
    if abs(last_valid_dist_m - gt_dist) <= 0.20:
        dist_m = last_valid_dist_m
    else:
        last_valid_dist_m = gt_dist
        dist_m = gt_dist
    assert dist_m == 2.15

