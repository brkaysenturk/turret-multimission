"""
================================================================================
BASKET CHALLENGE: WEBOTS TARET KONTROLCUSU (HIYERARSIK 2-ASAMALI MIMARI)
================================================================================
1. Asama: Model 1 (CupTracker - PPO) ile kupayi optik eksenin tam ortasina kilitler.
2. Asama: Hedef kilitlendiginde LiDAR mesafesine gore:
          - Dinamik Tilt Asirtma Acisi (Süper-elevasyon)
          - Dinamik Volan Sürücü Hizi (Muzzle Velocity)
          ile kupanin agzindan iceriye basket atar.
HUD: turret_fcs tasarimiyla tam uyumlu, Ingilizce, [RL CONTROLLER] banneri,
     Y: Fire, N: Abort, SPACE: E-Stop, ESC/Q: Quit kontrolleri.
================================================================================
"""

import sys
import os
import math
import time
from pathlib import Path
import numpy as np
import cv2
from controller import Supervisor

# Proje koku ve basket modullerini sys.path'e ekle
project_root = Path(__file__).resolve().parents[3]
basket_dir = project_root / "turret_basket"
for p in [str(project_root), str(basket_dir)]:
    if p not in sys.path:
        sys.path.insert(0, p)

from turret_basket.config import CONFIG
from turret_basket.yolo.cup_detector import CupDetector
from turret_basket.cup_tracker import CupTracker
from turret_basket.basket_hud import BasketHUD


def rot_matrix_to_axis_angle(R):
    """3x3 donus matrisini Webots eksen-aci [x, y, z, angle] formatina cevirir."""
    trace = R[0][0] + R[1][1] + R[2][2]
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if abs(theta) < 1e-4:
        return "0 0 1 0"
    sin_theta = math.sin(theta)
    x = (R[2][1] - R[1][2]) / (2.0 * sin_theta)
    y = (R[0][2] - R[2][0]) / (2.0 * sin_theta)
    z = (R[1][0] - R[0][1]) / (2.0 * sin_theta)
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-6:
        return "0 0 1 0"
    return f"{x/norm:.4f} {y/norm:.4f} {z/norm:.4f} {theta:.4f}"


def main():
    robot = Supervisor()
    # 30 FPS kamera dongusu ile uyumlu 32 ms timestep (8 x 4 ms basicTimeStep)
    basic_step = int(robot.getBasicTimeStep())
    if basic_step > 0:
        steps_32ms = max(1, round(32.0 / basic_step))
        timestep = steps_32ms * basic_step
    else:
        timestep = 32
    dt = timestep / 1000.0

    print("=" * 65)
    print(f"[WEBOTS] CUP BASKET CHALLENGE (2-STAGE RL TURRET FCS) | Timestep: {timestep}ms ({1.0/dt:.1f} FPS)")
    print("=" * 65)

    # 1. Motor ve Sensor Baglantilari
    pan_motor = robot.getDevice('pan_motor')
    tilt_motor = robot.getDevice('tilt_motor')
    pan_sensor = robot.getDevice('pan_sensor')
    tilt_sensor = robot.getDevice('tilt_sensor')

    if pan_motor:
        pan_motor.setPosition(0.0)
        pan_motor.setVelocity(1.5)
    if tilt_motor:
        tilt_motor.setPosition(0.0)
        tilt_motor.setVelocity(1.5)

    if pan_sensor:
        pan_sensor.enable(timestep)
    if tilt_sensor:
        tilt_sensor.enable(timestep)

    # Kamera C270
    camera = robot.getDevice('c270')
    if camera:
        camera.enable(timestep)
        cam_w = camera.getWidth()
        cam_h = camera.getHeight()
    else:
        cam_w, cam_h = 640, 480

    # TF-Luna LiDAR
    lidar = robot.getDevice('tf_luna')
    if lidar:
        lidar.enable(timestep)

    # Klavye
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    # Volan Motorlari
    flywheel_left = robot.getDevice('flywheel_left')
    flywheel_right = robot.getDevice('flywheel_right')
    if flywheel_left and flywheel_right:
        flywheel_left.setPosition(float('inf'))
        flywheel_right.setPosition(float('inf'))
        flywheel_left.setVelocity(0.0)
        flywheel_right.setVelocity(0.0)

    # Dugum Baglantilari
    cup_node = robot.getFromDef("CUP")
    desk_node = robot.getFromDef("CUP_DESK")
    kafa_node = robot.getFromDef("KAFA_BLOGU")
    root_children = robot.getRoot().getField("children")

    # Ortam Degiskenleri (Optuna Webots Tuning ve Headless Destegi)
    headless = os.environ.get("CUP_HEADLESS", "0").strip() == "1"
    max_sim_time_env = os.environ.get("CUP_MAX_SIM_TIME", "").strip()
    max_sim_time = float(max_sim_time_env) if max_sim_time_env else None
    tuning_json = os.environ.get("CUP_TUNING_JSON", "").strip() or None
    telemetry_path = os.environ.get("CUP_TELEMETRY_CSV", "").strip() or None
    target_x_env = os.environ.get("CUP_TARGET_X", "").strip()
    target_y_env = os.environ.get("CUP_TARGET_Y", "").strip()

    if target_x_env and target_y_env:
        tx = float(target_x_env)
        ty = float(target_y_env)
        if desk_node:
            trans_desk = desk_node.getField("translation")
            if trans_desk:
                trans_desk.setSFVec3f([0.0, ty, 0.0])
        if cup_node:
            trans_field = cup_node.getField("translation")
            if trans_field:
                trans_field.setSFVec3f([tx, ty, 0.71])
                print(f"[INIT] Cup repositioned by ENV: X={tx:.2f}m, Y={ty:.2f}m (Desk Y={ty:.2f}m)")

    # 2. Moduller: Tespitci, Model 1 (Tracker) ve Modern HUD
    detector = CupDetector()
    tracker = CupTracker(params_path=tuning_json)  # 1. MODEL: turret-fcs PPO Agirliklari
    hud = BasketHUD()
    hud.log_event("[SYS] BASKET CHALLENGE DIGITAL TWIN READY", (0, 255, 128))
    hud.log_event("[SYS] RL CENTERING AGENT ONLINE", (0, 215, 255))

    # Model 2 (Webots 3B Namlu Ucu Balistik Ajani - Hizli ve Bagimsiz NumPy MLP)
    webots_model2_npz = project_root / "turret_basket" / "rl" / "models" / "webots_ballistic_policy.npz"
    fallback_model2_npz = project_root / "turret_basket" / "rl" / "models" / "cup_ballistic_policy.npz"
    model2_npz = webots_model2_npz if webots_model2_npz.exists() else fallback_model2_npz
    model2_zip = project_root / "turret_basket" / "rl" / "models" / "cup_ballistic_policy.zip"
    model2 = None

    if model2_npz.exists():
        try:
            data = np.load(str(model2_npz))
            w1, b1, w2, b2, w3, b3 = data["w1"], data["b1"], data["w2"], data["b2"], data["w3"], data["b3"]
            def predict_model2(obs):
                h1 = np.tanh(np.dot(w1, obs) + b1)
                h2 = np.tanh(np.dot(w2, h1) + b2)
                return np.clip(np.dot(w3, h2) + b3, -1.0, 1.0)
            model2 = predict_model2
            print(f"[INFO] Model 2 ({model2_npz.name}) loaded successfully!")
        except Exception as e:
            print(f"[WARN] NumPy Model 2 load failed: {e}")

    if model2 is None and model2_zip.exists():
        try:
            from stable_baselines3 import PPO
            _sb3_m = PPO.load(str(model2_zip), device="cpu")
            model2 = lambda obs: _sb3_m.predict(obs, deterministic=True)[0]
            print("[INFO] Model 2 (SB3 PPO Policy) loaded successfully!")
        except Exception as e:
            print(f"[WARN] Model 2 zip could not be loaded: {e}")
            model2 = None

    # Taret Aci, Kilit ve E-Stop Durumlari
    pan_cmd_deg = 0.0
    tilt_cmd_deg = 0.0
    is_spinning = False
    shot_in_progress = False
    dart_spawned = False
    shot_time = 0.0
    total_darts = 0
    is_estop = False
    active_fire_speed = 10.0
    active_fire_tilt = 4.0
    applied_tilt_deg = 0.0
    last_valid_dist_m = 1.50
    locked_dist_m = None
    target_tilt_offset = 4.0
    target_speed_mps = 10.0
    is_locked = False

    lock_counter = 0
    LOCK_THRESHOLD_FRAMES = 12
    prev_locked = False
    target_lost_counter = 0

    WINDOW_NAME = "Turret Basket - Webots Digital Twin"
    if not headless:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 540)

    active_darts = []
    cleanup_darts = []
    telemetry_rows = []

    def get_firing_parameters(dist_m: float, delta_z_m: float = -0.2287) -> tuple[float, float]:
        """
        Webots'a ozel 3B namlu ucu (spawn_dart: +0.075 m ileri, +0.0256 m yukari) kinematigi ile
        ODE kati cisim dart balistigini kapali formda tam 0.1 mm hassasiyetle cozer.
        """
        b = 0.05
        g = 9.81
        cup_rim_z = 1.0558 + delta_z_m  # Dinamik kupa agzi hedef dunya yuksekligi
        target_z = cup_rim_z - 0.012    # Kupa merkezinde agzin 1.2 cm icine tam dalis

        # Mesafeye ve yukseklige gore yumusak asirtma acisi (19.5° .. 24.0°)
        th_deg = float(np.clip(22.2 - (dist_m - 1.2) * 1.4 + max(0.0, delta_z_m + 0.15) * 4.0, 19.5, 24.0))
        th = math.radians(th_deg)
        cos_th = math.cos(th)
        sin_th = math.sin(th)

        # Webots spawn_dart 3B namlu ucu konumu
        r_spawn = 0.075 * cos_th - 0.0256 * sin_th
        sz_launch = 1.0131 + 0.075 * sin_th + 0.0256 * cos_th
        d_flight = max(0.50, dist_m - r_spawn)

        # İkili arama (Binary Search) ile hedef kotu tam vuran v0 hızını analitik çöz
        v_low = 2.0
        v_high = 10.0
        for _ in range(35):
            v_mid = 0.5 * (v_low + v_high)
            vx0 = v_mid * cos_th
            vz0 = v_mid * sin_th
            arg = 1.0 - b * d_flight / vx0
            if arg <= 0.0:
                v_low = v_mid
                continue
            t = -(1.0 / b) * math.log(arg)
            zT = sz_launch + ((vz0 + g / b) / b) * (1.0 - math.exp(-b * t)) - (g / b) * t
            if zT < target_z:
                v_low = v_mid
            else:
                v_high = v_mid

        best_v = float(0.5 * (v_low + v_high))
        return float(th_deg), best_v

    def spawn_dart(v0_mps: float, launch_pitch_deg: float = 21.5, target_dist_m: float = 1.60):
        nonlocal total_darts
        total_darts += 1
        head_pos = kafa_node.getPosition() if kafa_node else [0.0, 0.0, 1.0131]
        head_rot = kafa_node.getOrientation() if kafa_node else [1, 0, 0, 0, 1, 0, 0, 0, 1]

        rx, ry, rz = head_rot[0], head_rot[3], head_rot[6]
        fx, fy, fz = head_rot[1], head_rot[4], head_rot[7]

        # Namlu ucundan firlat (CAD v10: Namlu ucu +0.075 m onde, kafa Z ekseninde +0.0256 m yukarida)
        sx = head_pos[0] + head_rot[1] * 0.075 + head_rot[2] * 0.0256
        sy = head_pos[1] + head_rot[4] * 0.075 + head_rot[5] * 0.0256
        sz = head_pos[2] + head_rot[7] * 0.075 + head_rot[8] * 0.0256

        vx = fx * v0_mps
        vy = fy * v0_mps
        vz = fz * v0_mps

        # Gercek agir uclu sunger dart yunuslama (weathercocking) acisal hizi:
        # Cikis aninda +launch_pitch_deg olan dart burnu, ucus suresi sonunda inis hiz vektorune (-22 deg) hizalanir
        th_rad = math.radians(launch_pitch_deg)
        r_sp = 0.075 * math.cos(th_rad) - 0.0256 * math.sin(th_rad)
        d_fl = max(0.50, target_dist_m - r_sp)
        vx0_h = max(1.0, v0_mps * math.cos(th_rad))
        vz0_v = v0_mps * math.sin(th_rad)
        b_lin = 0.05
        b_ang = 0.10
        g_acc = 9.81
        arg_fl = max(0.05, 1.0 - b_lin * d_fl / vx0_h)
        t_fl = -(1.0 / b_lin) * math.log(arg_fl)
        vx_imp = vx0_h * math.exp(-b_lin * t_fl)
        vz_imp = (vz0_v + g_acc / b_lin) * math.exp(-b_lin * t_fl) - (g_acc / b_lin)
        impact_pitch_rad = math.atan2(vz_imp, vx_imp)
        delta_pitch_rad = impact_pitch_rad - th_rad
        omega_pitch = delta_pitch_rad * b_ang / max(1e-4, 1.0 - math.exp(-b_ang * t_fl))
        wx = rx * omega_pitch
        wy = ry * omega_pitch
        wz = rz * omega_pitch

        # Silindir govdesini namlu eksenine hizala (VRML Cylinder Z eksenindedir, namlu Y eksenindedir)
        R_pitch90 = [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0]
        ]
        R_comb = [
            [sum(head_rot[3*r + k] * R_pitch90[k][c] for k in range(3)) for c in range(3)]
            for r in range(3)
        ]
        rot_str = rot_matrix_to_axis_angle(R_comb)

        dart_vrml = f"""
        DEF DART_{total_darts} Solid {{
          translation {sx:.4f} {sy:.4f} {sz:.4f}
          rotation {rot_str}
          children [
            DEF DART_SHAPE Pose {{
              children [
                Shape {{
                  appearance PBRAppearance {{
                    baseColor 0.1 0.45 0.95
                    roughness 0.4
                    metalness 0.1
                  }}
                  geometry Cylinder {{
                    height 0.072
                    radius 0.0065
                  }}
                }}
              ]
            }}
          ]
          name "dart_{total_darts}"
          contactMaterial "dart_foam"
          boundingObject USE DART_SHAPE
          physics Physics {{
            density -1
            mass 0.012
            centerOfMass [
              0 0 0
            ]
            inertiaMatrix [
              1e-05 1e-05 1e-05
              0 0 0
            ]
            damping Damping {{
              linear 0.05
              angular 0.10
            }}
          }}
        }}
        """
        if root_children:
            root_children.importMFNodeFromString(-1, dart_vrml)
            new_dart = robot.getFromDef(f"DART_{total_darts}")
            if not new_dart:
                new_dart = root_children.getMFNode(-1)

            if new_dart:
                new_dart.setVelocity([vx, vy, vz, wx, wy, wz])
                active_darts.append([new_dart, robot.getTime(), [sx, sy, sz]])
                print(f"[FIRE] Dart #{total_darts} released! Launch Velocity: {v0_mps:.1f} m/s")
            else:
                print(f"[ERROR] Could not reference imported dart node #{total_darts}!")
        else:
            print("[ERROR] root_children is None! Supervisor permissions required.")

    # 3. Ana Kontrol Dongusu
    prev_wall_t = time.perf_counter()
    live_fps = 31.0

    while robot.step(timestep) != -1:
        step_start_wall = time.perf_counter()
        sim_t = robot.getTime()

        # Gercek dunya FPS olcumu (Simulasyon hizi gostergesi)
        now_wall = time.perf_counter()
        dt_wall = now_wall - prev_wall_t
        prev_wall_t = now_wall
        if dt_wall > 0.001:
            inst_fps = 1.0 / dt_wall
            live_fps = 0.90 * live_fps + 0.10 * inst_fps

        # Sensör Verilerini Oku (Pan Webots koordinatinda ters yonde doner: pan_invert = True)
        # Sensor okunarak donanim geribildirimi alinir
        if pan_sensor:
            _ = -math.degrees(pan_sensor.getValue())
        tilt_sensor_deg = math.degrees(tilt_sensor.getValue()) if tilt_sensor else tilt_cmd_deg

        # Ground Truth Mesafe (Taret ekseninden kupa merkezine olan gercek yatay mesafe)
        gt_dist = 1.50
        if cup_node and kafa_node:
            cp = cup_node.getPosition()
            hp = kafa_node.getPosition()
            gt_dist = float(math.hypot(cp[0] - hp[0], cp[1] - hp[1]))

        # TF-Luna kafa merkezinden 0.055m onde, kupa on ceperi ise kupa merkezinden 0.045m ondedir.
        # LiDAR kupanin on yuzeyini olctugunden kupa merkezine olan gercek mesafe: raw_lidar + 0.10m
        CUP_CENTER_OFFSET_M = 0.10
        raw_lidar = lidar.getValue() if lidar else (gt_dist - CUP_CENTER_OFFSET_M)
        lidar_center_dist = raw_lidar + CUP_CENTER_OFFSET_M

        # LiDAR isini kupaya isabet ediyorsa olcumu kabul et (0.5m .. 3.5m arasi ve gt_dist ile tutarli)
        beam_on_cup = (0.5 <= lidar_center_dist <= 3.5) and (abs(lidar_center_dist - gt_dist) <= 0.25)
        if beam_on_cup:
            last_valid_dist_m = lidar_center_dist
            dist_m = lidar_center_dist
        else:
            # Lidar isini bardağı kacirdiysa (duvara/tablaya carptiysa) son gecerli mesafeyi dondur.
            # Eger bardak yer degistirdiyse (last_valid gercek mesafeden koptuysa) gt_dist'e hizalan.
            if abs(last_valid_dist_m - gt_dist) <= 0.20:
                dist_m = last_valid_dist_m
            else:
                last_valid_dist_m = gt_dist
                dist_m = gt_dist

        # Kilitli veya atis halinde mesafeyi kilit anindaki degerde dondur (Menzil Kilit Mandal / Range Latch)
        if (is_locked or shot_in_progress) and locked_dist_m is not None:
            dist_m = locked_dist_m

        # Kameradan Goruntu Al
        frame_bgr = None
        if camera:
            cam_data = camera.getImage()
            if cam_data:
                img_np = np.frombuffer(cam_data, np.uint8).reshape((cam_h, cam_w, 4))
                frame_bgr = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)

        if frame_bgr is None:
            frame_bgr = np.zeros((cam_h, cam_w, 3), dtype=np.uint8)

        # Kupa Tespiti (Anlik namlu egimine gore perspektif agiz standardizasyonu ile)
        detection = detector.detect(frame_bgr, tilt_deg=tilt_sensor_deg)

        # 1. ASAMA: Model 1 (CupTracker / RLController) ile Merkezleme & Kilitlenme
        is_locked = False
        if not is_estop:
            if detection.detected and not shot_in_progress:
                target_lost_counter = 0
                cx, cy = detection.center_px
                d_pan, d_tilt, locked_cand = tracker.compute(
                    cx=cx,
                    cy=cy,
                    center_x=cam_w // 2,
                    center_y=cam_h // 2,
                    screen_w=cam_w,
                    screen_h=cam_h,
                    distance_m=dist_m,
                    dt=dt
                )
                pan_cmd_deg = float(np.clip(pan_cmd_deg + d_pan, -75.0, 75.0))
                tilt_cmd_deg = float(np.clip(tilt_cmd_deg + d_tilt, -15.0, 30.0))

                # WEBOTS MOTOR SURUSU (turret_fcs SSOT: pan_invert = True)
                if pan_motor:
                    pan_motor.setPosition(math.radians(-pan_cmd_deg))
                if tilt_motor:
                    tilt_motor.setPosition(math.radians(tilt_cmd_deg))

                if locked_cand:
                    lock_counter = min(LOCK_THRESHOLD_FRAMES, lock_counter + 1)
                else:
                    lock_counter = max(0, lock_counter - 1)

                is_locked = (lock_counter >= LOCK_THRESHOLD_FRAMES)
                if is_locked:
                    if locked_dist_m is None:
                        # Kilit anindaki mesafeyi dondur (LiDAR zemin/oda sapmalarini onler)
                        locked_dist_m = dist_m
                        hud.log_event(f"[LOCK] RANGE FROZEN: {locked_dist_m:.2f}m", (0, 255, 200))
                    dist_m = locked_dist_m
                    if not prev_locked:
                        hud.log_event(f"[AIM] CUP ALIGNED (+{target_tilt_offset:.1f} deg)", (0, 255, 128))
                elif not shot_in_progress:
                    locked_dist_m = None

                prev_locked = is_locked
            elif not detection.detected and not shot_in_progress:
                target_lost_counter += 1
                lock_counter = max(0, lock_counter - 1)
                is_locked = False
                prev_locked = False
                locked_dist_m = None

                # Yalnizca hedef en az 10 kare (320 ms) boyunca gercekten kayipsa merkeze don
                # 1-2 karelik anlik tespit kayiplarinda tareti geri cekmeyi onler!
                if target_lost_counter >= 10:
                    tracker.on_target_lost()
                    pan_cmd_deg *= 0.96
                    tilt_cmd_deg *= 0.96
                    if abs(pan_cmd_deg) < 0.05:
                        pan_cmd_deg = 0.0
                    if abs(tilt_cmd_deg) < 0.05:
                        tilt_cmd_deg = 0.0

                    if pan_motor:
                        pan_motor.setPosition(math.radians(-pan_cmd_deg))
                    if tilt_motor:
                        tilt_motor.setPosition(math.radians(tilt_cmd_deg))
        else:
            # E-Stop aninda tareti dondurme
            lock_counter = 0
            is_locked = False
            locked_dist_m = None
            pan_cmd_deg = 0.0
            tilt_cmd_deg = 0.0

        if telemetry_path:
            telemetry_rows.append({
                "sim_time_sec": f"{sim_t:.3f}",
                "target_detected": 1 if detection.detected else 0,
                "err_pan": f"{detection.err_pan_deg:.4f}",
                "err_tilt": f"{detection.err_tilt_deg:.4f}",
                "is_locked": 1 if is_locked else 0,
                "cmd_pan_deg": f"{pan_cmd_deg:.4f}",
                "cmd_tilt_deg": f"{tilt_cmd_deg:.4f}",
            })

        # 2. ASAMA: Dinamik Balistik Parametreleri (2. Model: PPO RL Balistik Ajan Cikarimi)
        # Sadece atış sekansı devam etmiyorken hesapla (atış esnasında LiDAR duvara çarpabilir)
        if not shot_in_progress:
            # delta_z: Taret namlu seviyesinden kupa agzina olan dikey kot farki (yatay menzil uzerinden tanjant ile)
            # Gercek sistemde harici pozisyon sensoru olmadigindan yalnizca yazilimsal komut acisi (tilt_cmd_deg) kullanilir:
            delta_z_camera = dist_m * math.tan(math.radians(tilt_cmd_deg))
            delta_z_m = delta_z_camera - 0.0634  # Namlu eksenine gore gercek dikey kot farki
            if model2 is not None:
                # 2. Model PPO RL Girdisi: [norm_dist, norm_height]
                # Ultra-genisletilmis calisma zarfi: [1.0m .. 3.1m] ve [-0.85m .. +0.35m]
                norm_dist = np.clip((dist_m - 2.05) / 1.05, -1.0, 1.0)
                norm_h = np.clip((delta_z_m - (-0.25)) / 0.60, -1.0, 1.0)
                obs = np.array([norm_dist, norm_h], dtype=np.float32)
                act = model2(obs)
                tilt_min = CONFIG.rl.tilt_offset_min_deg
                tilt_max = CONFIG.rl.tilt_offset_max_deg
                speed_min = CONFIG.rl.speed_min_mps
                speed_max = CONFIG.rl.speed_max_mps
                target_tilt_offset = float(np.clip(17.5 + act[0] * 12.5, tilt_min, tilt_max))
                target_speed_mps = float(np.clip(7.0 + act[1] * 5.0, speed_min, speed_max))
            else:
                target_tilt_offset, target_speed_mps = get_firing_parameters(dist_m, delta_z_m=delta_z_m)

        # Atis Firlatma ve Toparlanma Sekansi (Spinup 0.5s -> Settle 0.25s -> Fire -> Recover 0.5s)
        if shot_in_progress:
            time_since_trigger = sim_t - shot_time
            # 1. Aşama: 0.50s spinup + 0.25s settle sonrasinda (t >= 0.75s) namlu oturmasi denetlenir ve ateslenir
            actual_tilt = math.degrees(tilt_sensor.getValue()) if tilt_sensor else applied_tilt_deg
            tilt_settled = abs(actual_tilt - applied_tilt_deg) < 0.25

            if ((tilt_settled and time_since_trigger >= 0.75) or (time_since_trigger >= 1.20)) and not dart_spawned:
                dart_spawned = True
                actual_th = math.degrees(tilt_sensor.getValue()) if tilt_sensor else applied_tilt_deg
                spawn_dart(v0_mps=active_fire_speed, launch_pitch_deg=actual_th, target_dist_m=dist_m)
                hud.log_event(f"[THROW] DART #{total_darts} RELEASED ({active_fire_speed:.1f} m/s @ +{actual_th:.1f} deg)", (0, 255, 128))
                print(f"[FIRE] Dart #{total_darts} released! Launch Velocity: {active_fire_speed:.1f} m/s, Actual Pitch: +{actual_th:.1f} deg")

            # 2. Aşama: Firlatma sonrasi toparlanma (0.75s + 0.08s firing + 0.50s recover = ~1.33s toplam)
            if time_since_trigger > 1.33:
                shot_in_progress = False
                dart_spawned = False
                locked_dist_m = None
                if is_spinning and flywheel_left and flywheel_right:
                    flywheel_left.setVelocity(0.0)
                    flywheel_right.setVelocity(0.0)
                    is_spinning = False
                if tilt_motor:
                    tilt_motor.setPosition(math.radians(tilt_cmd_deg))

        # Basket Kontrolu (3B Dogru Parcasi Boyunca Surekli Carpisma Tespiti - CCD)
        if cup_node and active_darts:
            cup_pos = cup_node.getPosition()

            for entry in list(active_darts):
                dart_node, spawn_t, prev_pos = entry
                try:
                    dart_pos = dart_node.getPosition()
                    dx = dart_pos[0] - cup_pos[0]
                    dy = dart_pos[1] - cup_pos[1]
                    dz = dart_pos[2] - cup_pos[2]
                    dist_to_cup = math.hypot(dx, dy)

                    # 32 ms kareler arasindaki 3B yorunge dogru parcasini 20 alt adimda (1.6 ms / ~7 mm cozunurlukle) tara
                    is_inside_cup = False
                    best_radial_off = dist_to_cup
                    for step_i in range(1, 21):
                        alpha_s = step_i / 20.0
                        sx_i = prev_pos[0] + alpha_s * (dart_pos[0] - prev_pos[0])
                        sy_i = prev_pos[1] + alpha_s * (dart_pos[1] - prev_pos[1])
                        sz_i = prev_pos[2] + alpha_s * (dart_pos[2] - prev_pos[2])
                        r_i = math.hypot(sx_i - cup_pos[0], sy_i - cup_pos[1])
                        dz_i = sz_i - cup_pos[2]
                        if (r_i <= 0.043) and (0.015 <= dz_i <= 0.115):
                            is_inside_cup = True
                            if r_i < best_radial_off:
                                best_radial_off = r_i

                    entry[2] = [dart_pos[0], dart_pos[1], dart_pos[2]]

                    if is_inside_cup:
                        hud.record_shot(is_basket=True)
                        hud.log_event("[SCORE] BASKET SCORED! (+100 PTS)", (0, 255, 128))
                        print(f"[GOAL!] BASKET SCORED! Dart inside cup opening (radial off: {best_radial_off*100:.1f} cm)")
                        active_darts.remove(entry)
                        cleanup_darts.append((dart_node, sim_t))
                        break
                    elif (sim_t - spawn_t) > 2.0:
                        hud.record_shot(is_basket=False)
                        hud.log_event("[RESULT] SHOT MISSED CUP", (0, 60, 255))
                        if dist_to_cup < 0.060 and dz < 0.125:
                            print(f"[MISS] Dart hit cup exterior wall/lip (dist: {dist_to_cup*100:.1f} cm, dz: {dz*100:.1f} cm)")
                        else:
                            print(f"[MISS] Dart missed cup. Remaining distance: {dist_to_cup*100:.1f} cm")
                        active_darts.remove(entry)
                        cleanup_darts.append((dart_node, sim_t))
                except Exception:
                    pass

        # Atisi tamamlanmis dartlari 1.5 saniye sonra sahneden temizle
        for d_node, finish_t in list(cleanup_darts):
            if (sim_t - finish_t) > 1.5:
                try:
                    d_node.remove()
                except Exception:
                    pass
                cleanup_darts.remove((d_node, finish_t))

        # Zaman asimina ugrayan eski aktif dartlari temizle
        if active_darts and (sim_t - active_darts[0][1]) > 3.0:
            old_entry = active_darts.pop(0)
            try:
                old_entry[0].remove()
            except Exception:
                pass

        # Modern Bilgilendirici HUD Render (Headless modda render/imshow yukunu atla)
        if not headless:
            lock_ratio = float(lock_counter / LOCK_THRESHOLD_FRAMES)
            disp_speed = active_fire_speed if shot_in_progress else target_speed_mps
            disp_tilt = active_fire_tilt if shot_in_progress else target_tilt_offset
            rendered = hud.render(
                frame=frame_bgr,
                cup_detected=detection.detected,
                bbox=detection.bbox,
                confidence=detection.confidence,
                distance_m=dist_m,
                err_pan_deg=detection.err_pan_deg,
                err_tilt_deg=detection.err_tilt_deg,
                fps=live_fps,
                is_locked=is_locked,
                firing=shot_in_progress,
                is_estop=is_estop,
                lock_ratio=lock_ratio,
                speed_mps=disp_speed,
                tilt_offset_deg=disp_tilt,
                center_px=detection.center_px
            )
            cv2.imshow(WINDOW_NAME, rendered)

            # Gercek zamanli 1.0x oynatma hizi sabitlemesi (Pacing):
            # 32 ms adimda (31.25 FPS) hesaplama erken biterse kalan sure kadar beklenerek normal insan gozu hizina kilitlenir.
            elapsed_step = time.perf_counter() - step_start_wall
            rem_ms = int(round((dt - elapsed_step) * 1000.0))
            wait_ms = max(1, min(33, rem_ms)) if rem_ms > 0 else 1
            cv_k = cv2.waitKey(wait_ms) & 0xFF
        else:
            elapsed_step = time.perf_counter() - step_start_wall
            rem_s = dt - elapsed_step
            if rem_s > 0.001:
                time.sleep(rem_s)
            cv_k = 255

        # Klavye Olaylari: HEM Webots 3B Gorunumu HEM DE OpenCV HUD Penceresinden Tuslari Topla!
        keys_pressed = []
        wb_k = keyboard.getKey()
        while wb_k > 0:
            keys_pressed.append(wb_k)
            wb_k = keyboard.getKey()

        if cv_k not in (255, 0):
            keys_pressed.append(cv_k)

        should_break = False
        for k in keys_pressed:
            if k == 32:  # SPACE: E-Stop (Acil Durdurma Mandali)
                is_estop = not is_estop
                if is_estop:
                    print("[SAFETY] Safety stop engaged! Motors halted.")
                    hud.log_event("[SYS] SAFETY STOP ENGAGED", (0, 60, 255))
                    if flywheel_left and flywheel_right:
                        flywheel_left.setVelocity(0.0)
                        flywheel_right.setVelocity(0.0)
                        is_spinning = False
                    if pan_motor:
                        pan_motor.setPosition(0.0)
                    if tilt_motor:
                        tilt_motor.setPosition(0.0)
                else:
                    print("[SAFETY] System resumed.")
                    hud.log_event("[SYS] SYSTEM RESUMED", (0, 255, 128))

            elif k in (ord('Y'), ord('y')):  # Y: Atisi Baslat / Authorize Fire
                if not is_estop and not shot_in_progress:
                    if is_locked:
                        # Tetik aninda kameradaki kalan ince yatay (pan) hatasini spin-up suresinde sifirla (sensor gerektirmez)
                        if detection.detected and abs(detection.err_pan_deg) < 2.0:
                            pan_cmd_deg = float(np.clip(pan_cmd_deg + 0.85 * detection.err_pan_deg, -75.0, 75.0))
                            if pan_motor:
                                pan_motor.setPosition(math.radians(-pan_cmd_deg))

                        # Kilitlenen balistik hiz ve aciyi dondur (Trigger Latch)
                        active_fire_speed = target_speed_mps
                        active_fire_tilt = target_tilt_offset
                        # Model 2 ve get_firing_parameters dogrudan namlu yuksekliginden (barrel_z=1.0387m)
                        # cozdugu icin target_tilt_offset zaten tam mutlak atis acisidir.
                        applied_tilt_deg = float(np.clip(active_fire_tilt, -15.0, 30.0))
                        print(f"[INPUT] FIRE command received (Key: Y)! Range: {dist_m:.2f}m (LOCKED) -> Speed: {active_fire_speed:.1f} m/s, Elevating to +{applied_tilt_deg:.1f} deg & spin-up...")
                        shot_in_progress = True
                        dart_spawned = False
                        shot_time = sim_t
                        hud.log_event(f"[FIRE] SPEED: {active_fire_speed:.1f} m/s | +{applied_tilt_deg:.1f} deg", (0, 191, 255))

                        # Volan Suruculerini Dondurulan Hiza Cikar
                        wheel_rad_s = active_fire_speed * 246.0
                        if flywheel_left and flywheel_right:
                            flywheel_left.setVelocity(wheel_rad_s)
                            flywheel_right.setVelocity(-wheel_rad_s)
                            is_spinning = True

                        if tilt_motor:
                            tilt_motor.setPosition(math.radians(applied_tilt_deg))
                    else:
                        print("[WARN] Cannot fire: Target cup not locked! Wait for lock bar to fill.")
                        hud.log_event("[WARN] CANNOT FIRE - CUP NOT LOCKED", (0, 0, 255))

            elif k in (ord('N'), ord('n')):  # N: Kilidi Birak / Abort Lock
                lock_counter = 0
                locked_dist_m = None

            elif k in (ord('R'), ord('r')):  # R: Kupayi Yeni Konuma Tasi / Relocate
                if cup_node:
                    # Mesafe: 1.20m - 2.40m, Yatay Sapma: -0.35m ile +0.35m
                    rand_y = float(np.random.uniform(1.20, 2.40))
                    rand_x = float(np.random.uniform(-0.35, 0.35))
                    # Masa ve Kupa Yuksekligi: Masa tablasini -0.30m (alçak sehpa) ile +0.20m (yüksek masa) arasi rastgele tasi
                    # Standart masa z=0.0m iken tabla 0.71m'dir.
                    rand_table_dz = float(np.random.uniform(-0.30, 0.20))
                    desk_z = rand_table_dz
                    cup_z = 0.71 + rand_table_dz

                    if desk_node:
                        trans_desk = desk_node.getField("translation")
                        if trans_desk:
                            trans_desk.setSFVec3f([0.0, rand_y, desk_z])
                            try:
                                desk_node.resetPhysics()
                            except Exception:
                                pass

                    trans_field = cup_node.getField("translation")
                    if trans_field:
                        trans_field.setSFVec3f([rand_x, rand_y, cup_z])
                        try:
                            cup_node.resetPhysics()
                            cup_node.setVelocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        except Exception:
                            pass

                    # Onceki tum atis dartlarini dunya agacindan temizle
                    for item in list(active_darts) + list(cleanup_darts):
                        try:
                            item[0].remove()
                        except Exception:
                            pass
                    active_darts.clear()
                    cleanup_darts.clear()

                    # Atis sekansi devam ediyorsa guvenli sekilde sifirla
                    shot_in_progress = False
                    dart_spawned = False
                    if is_spinning and flywheel_left and flywheel_right:
                        flywheel_left.setVelocity(0.0)
                        flywheel_right.setVelocity(0.0)
                        is_spinning = False

                    lock_counter = 0
                    locked_dist_m = None
                    new_dist = float(math.hypot(rand_x, rand_y))
                    last_valid_dist_m = new_dist
                    dist_m = new_dist
                    target_lost_counter = 0
                    is_locked = False
                    prev_locked = False
                    tracker.reset()
                    detector.reset()
                    delta_z_display = cup_z + 0.1065 - 1.0387  # Namluya gore kupa agzi kot farki (10.65cm kupa)
                    print(f"[WORLD] Cup relocated: Range={rand_y:.2f}m, X={rand_x:.2f}m, Table_Z={cup_z:.2f}m (Delta_Z={delta_z_display:+.2f}m)")
                    hud.log_event(f"[CUP] REPOSITIONED ({rand_y:.2f}m | dZ:{delta_z_display:+.2f}m)", (200, 210, 220))

            elif k in (ord('Q'), ord('q'), 27):  # Q veya ESC: Cikis
                should_break = True
                break

        if should_break:
            break

        if max_sim_time is not None and sim_t >= max_sim_time:
            break

    if telemetry_path and telemetry_rows:
        import csv
        with open(telemetry_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(telemetry_rows[0].keys()))
            writer.writeheader()
            writer.writerows(telemetry_rows)
        print(f"[TELEMETRY] Logged {len(telemetry_rows)} frames to {telemetry_path}")

    if not headless:
        cv2.destroyAllWindows()

    if max_sim_time is not None:
        try:
            robot.simulationQuit(0)
        except Exception:
            pass


if __name__ == "__main__":
    main()
