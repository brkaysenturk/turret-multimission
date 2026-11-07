"""
================================================================================
TEST KATEGORISI 3: 2. MODEL (RL BALLISTICS), RK4 FIZIK MOTORU VE WEBOTS 3B CCD
================================================================================
Kapsam:
- turret_basket.rl_ballistics (Donanim balistik RL politikasi cup_ballistic_policy.npz)
- turret_basket.rl.cup_env (RK4 aerodinamik yorunge cozucu ve BallisticBasketEnv)
- Webots 3B namlu ucu (spawn_dart) balistik politikasi (webots_ballistic_policy.npz) ve 3B CCD
================================================================================
"""

import math
from pathlib import Path
import numpy as np

from turret_basket.config import CONFIG
from turret_basket.rl_ballistics import RLBallistics, LaunchParameters
from turret_basket.rl.cup_env import BallisticBasketEnv, solve_rk4_trajectory


def test_basket_rl_ballistics_inference():
    """2. Model (RLBallistics) politika cikarimini, super-elevasyon sinirlarini ve RK4 isabetini dogrular."""
    ballistics = RLBallistics()

    # 1. 1.80m mesafede firlatma parametreleri
    params = ballistics.compute_launch_parameters(dist_m=1.80)
    assert isinstance(params, LaunchParameters)
    assert CONFIG.rl.tilt_offset_min_deg <= params.tilt_offset_deg <= CONFIG.rl.tilt_offset_max_deg
    assert CONFIG.rl.speed_min_mps <= params.speed_mps <= CONFIG.rl.speed_max_mps

    # 2. Super-elevasyon acisi ve 30 derece ust tavan
    super_elev = ballistics.compute_super_elevation(params.tilt_offset_deg)
    assert CONFIG.rl.tilt_offset_min_deg <= super_elev <= CONFIG.rl.tilt_offset_max_deg
    assert abs(super_elev - (params.tilt_offset_deg + ballistics.default_parallax_margin_deg)) < 1e-4
    assert ballistics.compute_super_elevation(28.0) == 28.0

    # 3. Gercek donanim modeliyle 1.45m mesafede RK4 yorungesinin on dudaga carpmadan kupaya girmesi
    params_145 = ballistics.compute_launch_parameters(dist_m=1.45, delta_z_m=-0.2322)
    final_y, _, _, _, _, hit_table, hit_lip = solve_rk4_trajectory(
        1.45, params_145.speed_mps, math.radians(params_145.tilt_offset_deg), rim_rel_y=-0.2322, bottom_rel_y=-0.3237
    )
    assert not hit_table
    assert not hit_lip
    assert -0.3237 <= final_y <= -0.2222


def test_basket_ballistics_rk4_trajectory():
    """2. Model Runge-Kutta 4. Derece (RK4) aerodinamik yorunge cozumunu dogrular."""
    res = solve_rk4_trajectory(
        dist_m=2.0,
        v0=10.0,
        theta_launch_rad=math.radians(10.0),
    )
    assert len(res) == 7
    y, t_flight, v_impact, vy, apex_y, hit_table, hit_front_lip = res
    assert t_flight > 0.05
    assert v_impact > 0.0
    assert isinstance(hit_table, bool)
    assert isinstance(hit_front_lip, bool)


def test_basket_ballistics_gym_environment():
    """BallisticBasketEnv Gymnasium ortaminda reset, step ve odul sozlesmelerini dogrular."""
    env = BallisticBasketEnv(domain_randomization=False)
    obs, info = env.reset(seed=42)

    assert obs.shape == (2,)
    assert -1.0 <= obs[0] <= 1.0

    action = np.array([0.0, 0.0], dtype=np.float32)
    next_obs, reward, terminated, truncated, step_info = env.step(action)

    assert next_obs.shape == (2,)
    assert isinstance(reward, (float, np.floating))
    assert isinstance(terminated, bool)
    assert "basket" in step_info
    assert "miss_dist_m" in step_info
    assert "speed_mps" in step_info
    assert "tilt_deg" in step_info


def test_webots_3d_muzzle_ballistics_and_ccd():
    """Webots'a ozel 3B namlu ucu balistigi (webots_ballistic_policy.npz) ve 20 alt adimli CCD dogrulamasi."""
    # 1. Webots'a ozel balistik model dosyasinin varligi ve 3B namlu ucu isabeti
    webots_npz = Path(__file__).resolve().parents[1] / "turret_basket" / "rl" / "models" / "webots_ballistic_policy.npz"
    assert webots_npz.exists()
    data = np.load(str(webots_npz))
    w1, b1, w2, b2, w3, b3 = data["w1"], data["b1"], data["w2"], data["b2"], data["w3"], data["b3"]

    b = 0.05
    g = 9.81
    for dist_m in [1.35, 1.75, 2.25]:
        for table_dz in [-0.20, 0.0, 0.15]:
            cup_z = 0.71 + table_dz
            cup_rim_z = cup_z + 0.1065
            delta_z_m = cup_rim_z - 1.0558
            norm_dist = np.clip((dist_m - 2.05) / 1.05, -1.0, 1.0)
            norm_h = np.clip((delta_z_m - (-0.25)) / 0.60, -1.0, 1.0)
            obs = np.array([norm_dist, norm_h], dtype=np.float32)
            h1 = np.tanh(np.dot(w1, obs) + b1)
            h2 = np.tanh(np.dot(w2, h1) + b2)
            act = np.clip(np.dot(w3, h2) + b3, -1.0, 1.0)
            th_deg = float(17.5 + act[0] * 12.5)
            v0 = float(7.0 + act[1] * 5.0)
            th = math.radians(th_deg)
            r_spawn = 0.075 * math.cos(th) - 0.0256 * math.sin(th)
            sz = 1.0131 + 0.075 * math.sin(th) + 0.0256 * math.cos(th)
            d_flight = dist_m - r_spawn
            vx0, vz0 = v0 * math.cos(th), v0 * math.sin(th)
            t_c = -(1.0 / b) * math.log(1.0 - b * d_flight / vx0)
            z_c = sz + ((vz0 + g / b) / b) * (1.0 - math.exp(-b * t_c)) - (g / b) * t_c
            assert (cup_z + 0.02) <= z_c <= cup_rim_z

    # 2. 20 alt adimli CCD (Surekli Carpisma Tespiti) dogrulamasi:
    # Kare basinda kupanin ustunde (z=0.85m), kare sonunda kupanin altinda (z=0.70m) olan hizli dart yakalanmali
    cup_pos = [0.0, 2.00, 0.71]
    prev_pos = [0.0, 1.96, 0.85]
    dart_pos = [0.0, 2.05, 0.70]
    caught = False
    for step_i in range(1, 21):
        a = step_i / 20.0
        sx_i = prev_pos[0] + a * (dart_pos[0] - prev_pos[0])
        sy_i = prev_pos[1] + a * (dart_pos[1] - prev_pos[1])
        sz_i = prev_pos[2] + a * (dart_pos[2] - prev_pos[2])
        if math.hypot(sx_i - cup_pos[0], sy_i - cup_pos[1]) <= 0.043 and (0.015 <= (sz_i - cup_pos[2]) <= 0.115):
            caught = True
            break
    assert caught is True

