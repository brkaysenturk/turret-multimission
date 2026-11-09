"""LiDAR menzil kapılama: sıçrama reddi, ateş dondurma, geometri ve yeniden yakalama."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from turret_fcs.fcs_manager import FCSManager


def _stable():
    fcs = FCSManager(control_mode="P")
    for _ in range(3):
        fcs._update_lidar_range(200, False, None, 1000.0, 1000.0)
    assert fcs.range_valid is True
    assert fcs.range_stable is True
    return fcs


def test_lidar_range_gating_and_freeze():
    # 1. Kararlı menzil başlatma
    fcs = _stable()

    # 2. Ani duvar sıçraması reddi
    fcs._update_lidar_range(408, False, None, 1000.0, 1000.0)
    assert fcs.range_jump is True
    assert fcs.range_valid is False
    assert fcs.last_valid_target_dist_m == 2.0

    # 3. Geometrik tutarsızlık reddi (kamera kutusu ile LiDAR menzili uyuşmazlığı)
    fcs_geom = FCSManager(control_mode="P")
    for _ in range(3):
        fcs_geom._update_lidar_range(200, False, (211, 211), 1410.0, 1412.0)
    fcs_geom._update_lidar_range(408, False, (211, 211), 1410.0, 1412.0)
    assert fcs_geom.range_geom_mismatch is True
    assert fcs_geom.range_valid is False
    assert fcs_geom.last_valid_target_dist_m == 2.0

    # 4. Atış esnasında LiDAR dondurma (flaş/duman gürültüsünden korunma)
    fcs_fire = _stable()
    fcs_fire._update_lidar_range(408, True, None, 1000.0, 1000.0)
    assert fcs_fire.last_valid_target_dist_m == 2.0
    assert fcs_fire.range_valid is True

    # 5. Atış bittikten sonra yeni mesafeyi kararlı yakalama
    for _ in range(5):
        fcs_fire._update_lidar_range(300, False, None, 1000.0, 1000.0)
    assert fcs_fire.last_valid_target_dist_m == 3.0
    assert fcs_fire.range_valid is True

    # 6. İlk hedef yakalamada (henüz menzil kilitlenmemişken) farklı boyuttaki balonun
    #    varsayılan 2.0m'den >0.5m uzakta olması durumunda geometri kilidine takılmadan kabulü
    fcs_init = FCSManager(control_mode="P")
    fcs_init._update_lidar_range(115, False, (60, 60), 1410.0, 1412.0)
    assert fcs_init.range_valid is True
    assert fcs_init.last_valid_target_dist_m == 1.15



def test_lidar_off_target_and_geometric_fallback():
    """Nisangah hedef kutusunun disindayken (off_target) duvar okumasinin reddedildigini ve geometrik mesafeye dusuldugunu dogrular."""
    fcs = FCSManager(control_mode="P")
    # Hedef kenarda [900, 300, 1000, 400] (merkezi 950, 350), nisangah (640, 360) kutunun disinda.
    # LiDAR arka duvar olan 450 cm (4.5 m) okuyor.
    fcs.update([900, 300, 1000, 400], 640, 360, 1411.0, 1411.0, 0.033, is_firing=False, dist_cm=450)
    assert fcs.range_status == "off_target"
    assert fcs.range_valid is False
    # 4.5 m duvar mesafesi kabul edilmemeli!
    assert fcs.last_valid_target_dist_m == 2.0
    assert fcs.range_geom_m is not None and fcs.range_geom_m > 3.5

    # CAD v10 3.30 cm dikey paralaks ve %60 elips cekirdek kapisi testi:
    # 1.5 m'deki balon (280x280 px kutu) kamera merkezinin (640, 360) 65 px yukarisindayken (y=295):
    # Kamera merkezi (360) kutunun alt yarisinin icindedir (295 + 140 = 435), ANCAK 3.30 cm asagidaki
    # LiDAR izi (y ~ 391) %60 elips cekirdeginin (295 + 84 = 379) disina kayip duvara (420 cm) vurmustur!
    fcs_parallax = FCSManager(control_mode="P")
    fcs_parallax._update_lidar_range(
        150, False, (280, 280), 1411.0, 1411.0,
        box_center=(640.0, 380.0), frame_center=(640.0, 360.0),
    )
    assert fcs_parallax.range_valid is True
    assert fcs_parallax.last_valid_target_dist_m == 1.50

    fcs_parallax._update_lidar_range(
        420, False, (280, 280), 1411.0, 1411.0,
        box_center=(640.0, 295.0), frame_center=(640.0, 360.0),
    )
    assert fcs_parallax.range_status == "off_target"
    assert fcs_parallax.range_valid is False
    assert fcs_parallax.last_valid_target_dist_m == 1.50
