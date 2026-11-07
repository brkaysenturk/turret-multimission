"""
================================================================================
TEST KATEGORISI 5: BASKET HUD, PRESENTER VE KLAVYE OLAY YONETIMI (KEYMAP)
================================================================================
Kapsam:
- turret_basket.basket_hud (Skor tablosu, kilit bari ve olay gunlugu)
- turret_basket.presenter (OpenCV HUD katmani cizimi)
- turret_basket.keymap (Y, N, R, SPACE, Q klavye tuslarinin BasketManager entegrasyonu)
================================================================================
"""

import numpy as np

from turret_basket.basket_manager import BasketManager, BasketStatus
from turret_basket.basket_hud import BasketHUD
from turret_basket.presenter import BasketPresenter
from turret_basket.keymap import process_key_event


def test_basket_hud_and_presenter():
    """BasketHUD ve BasketPresenter render katmanlarinin calismasini dogrular."""
    hud = BasketHUD()
    presenter = BasketPresenter(hud)

    dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # 1. Scanning durumu
    status_scan = BasketStatus(
        cup_detected=False,
        bbox=None,
        confidence=0.0,
        distance_m=1.45,
        err_pan_deg=0.0,
        err_tilt_deg=0.0,
        is_locked=False,
        firing=False,
        is_estop=False,
        lock_ratio=0.0,
        speed_mps=4.4,
        tilt_offset_deg=21.8,
    )
    out1 = presenter.render(dummy_frame.copy(), status=status_scan, fps=30.0)
    assert out1.shape == dummy_frame.shape

    # 2. Locked durumu
    status_locked = BasketStatus(
        cup_detected=True,
        bbox=(200, 150, 440, 350),
        confidence=0.95,
        distance_m=1.80,
        err_pan_deg=0.2,
        err_tilt_deg=0.1,
        is_locked=True,
        firing=False,
        is_estop=False,
        lock_ratio=1.0,
        speed_mps=4.5,
        tilt_offset_deg=22.0,
    )
    out2 = presenter.render(dummy_frame.copy(), status=status_locked, fps=30.0)
    assert out2.shape == dummy_frame.shape

    # 3. Skor tablosu
    hud.record_shot(is_basket=True)
    assert hud.total_shots == 1
    assert hud.total_baskets == 1


def test_basket_keymap_events():
    """Basket keymap tus isleyicisinin dogrudan BasketManager ile entegrasyonunu dogrular."""
    manager = BasketManager(comm=None)
    hud = BasketHUD()

    # 1. Tus basilmadi (-1)
    exit_req = process_key_event(key_code=-1, manager=manager, hud=hud)
    assert exit_req is False

    # 2. Cikis (Q)
    exit_req = process_key_event(key_code=ord("q"), manager=manager, hud=hud)
    assert exit_req is True

    # 3. Y (Kilitli degilken atis reddedilmeli)
    manager.is_locked = False
    process_key_event(key_code=ord("y"), manager=manager, hud=hud)
    assert manager.engagement.is_active is False
    assert any("CANNOT FIRE" in msg for msg, _ in hud.event_logs)

    # 4. Y (Kilitliyken atis baslatilmali)
    manager.is_locked = True
    process_key_event(key_code=ord("y"), manager=manager, hud=hud)
    assert manager.engagement.is_active is True

    # 5. N (Atisi iptal et)
    process_key_event(key_code=ord("n"), manager=manager, hud=hud)
    assert manager.engagement.is_active is False

    # 6. SPACE (E-Stop mandallama)
    process_key_event(key_code=32, manager=manager, hud=hud)
    assert manager.is_estop is True
    process_key_event(key_code=32, manager=manager, hud=hud)
    assert manager.is_estop is False

    # 7. R (Reset)
    manager.lock_counter = 10
    process_key_event(key_code=ord("r"), manager=manager, hud=hud)
    assert manager.lock_counter == 0

