"""
================================================================================
MODUL: ATIS KONTROL SISTEMI UST YONETICISI (FCS MANAGER)
================================================================================
Bu modul, otonom taretin en ust karar verici ve durum yoneticisidir (State Machine).
Tum alt kontrolculeri, filtreleri ve guvenlik mekanizmalarini koordine eder.

GOREVLER VE ALGORITMALAR:
1. Hedef Onceliklendirme: Ego-hareket telafili uzamsal kapi ve IoU handoff ile hedef secimi.
2. Durum Kestirimi (CWNA Kalman Filter): 3B pinhole ego-motion ve disli boslugu (backlash)
   kovaryans enjeksiyonu ile dunya referansli hedef hiz vektoru (vx, vy) kestirimi.
3. Balistik Onleme ve Yercekimi Telafisi: Ucus suresi (Time-of-Flight) boyunca hedefin
   alacagi yolu hesaplayip atis onleme acisi (Lead Angle) ve irtifa dususunu kompanse etme.
4. Dinamik Kilit Yonetimi (Lock Manager): Histerezisli kilit penceresi ve analitik
   Gauss isabet olasiligi hesabi ile atis izni kontrolu.
5. Mod Yonetimi ve Kontrolculer: Fiziksel hiz-ivme sinirli Tip-1 P Kontrolcu (Hassas Kilit)
   veya 9-Boyutlu PPO RL Kontrolcu (Cevik Takip).
6. Sakin Figure-8 (Lissajous) Devriye: Hedef kayboldugunda 1.5 sn bekleme, merkeze donus ve 2 eksenli sekiz motifi tarama.
================================================================================
"""

from collections import deque
import numpy as np

from .target_selector import TargetSelector
from .kalman_filter import KalmanFilter, ServoMotionEstimator, rotated_pixel_shift
from .lock_manager import LockManager
from .p_controller import PController
from .patrol_generator import PatrolGenerator
from .config import CONFIG
from .ballistics import compute_rk4_super_elevation
from .dob import DisturbanceObserver


class FCSManager:
    """
    Hedef secimi, Kalman durum takibi, dinamik kilit yonetimi, balistik ve tarete 
    atesleme koordinasyonunu yoneten ust seviye yonetici sinifi.
    """

    def __init__(self, control_mode="P", max_lost_frames=CONFIG.selector.max_lost_frames, patrol_enabled=False):
        self.selector = TargetSelector()
        self.tracker = KalmanFilter()
        self.ego_model = ServoMotionEstimator()
        self.lock_mgr = LockManager()
        self.p_controller = PController()
        self.patrol = PatrolGenerator()
        self.control_mode = "RL" if control_mode == "RL" else "P"  # "P" (Precision) veya "RL" (Agile)
        self.rl_controller = None
        if self.control_mode == "RL":
            self._init_rl()

        self.patrol_enabled = bool(patrol_enabled)
        self.patrol_status = "STANDBY"

        self.max_lost_frames = max_lost_frames
        self.frames_since_lost = 0
        self.consecutive_target_frames = 0

        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.last_pan_err_deg = 0.0
        self.last_tilt_err_deg = 0.0

        # Son karede tarete gonderilen motor komutlari (Ego-motion kompanzasyonu icin)
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0

        # Taret Acisal Konum Kestirimi (Cift Durum Mimarisi)
        # 1. Komut Edilen Ic Pozisyon (Commanded Pose / Dead-Reckoning):
        # Devriye ve yazilimsal limitler bu temiz referansa gore calisir.
        self.turret_pan_deg = 0.0
        self.turret_tilt_deg = 0.0
        # 2. Olculen Donanimsal Pozisyon (Sensed Pose / Telemetry Feedback):
        # STM32 paketlerinden beslenir; HUD ve stall kontrolu icindir.
        self.hw_pan_deg = 0.0
        self.hw_tilt_deg = 0.0
        self._has_initial_hw_sync = False
        self.tracking_lag_deg = 0.0
        self.is_stalled = False
        self.PAN_LIMIT_MIN = CONFIG.mount.pan_limit_min
        self.PAN_LIMIT_MAX = CONFIG.mount.pan_limit_max
        self.TILT_LIMIT_MIN = CONFIG.mount.tilt_limit_min
        self.TILT_LIMIT_MAX = CONFIG.mount.tilt_limit_max

        # Ego-motion: 1. derece servo yaniti, disli boslugu (backlash) ve kovaryans modeli
        self.ego_tau_s = CONFIG.control.ego_tau_s
        self.ego_step_pan_deg = 0.0
        self.ego_step_tilt_deg = 0.0
        self.ego_var_x = 0.0
        self.ego_var_y = 0.0
        self._frame_ego = (0.0, 0.0)
        self._ego_prepared = False
        self._image_center = (CONFIG.optics.default_width / 2.0, CONFIG.optics.default_height / 2.0)

        # Menzil Kapilama (Range Gating) icin dogrulanmis hedef mesafesi hafizasi
        self.last_valid_target_dist_m = float(CONFIG.ballistics.default_distance_m)
        self.range_valid = False
        self.range_stable = False
        self.range_jump = False
        self.range_geom_m = None
        self.range_geom_mismatch = False
        self.range_status = "unavailable"
        self._range_has_reference = False
        self._range_history = deque(maxlen=max(3, int(CONFIG.lidar.stability_window)))
        self._range_candidate = None
        self._range_candidate_count = 0

        # Balistik Slew ve Nisan Noktasi Ileri Beslemesi (Aim-Point Feedforward)
        self.prev_applied_lead_pan = 0.0
        self.prev_applied_lead_tilt = 0.0
        self.lead_time = 0.0
        self.dob = DisturbanceObserver(
            enabled=CONFIG.control.dob_enabled,
            gain=CONFIG.control.dob_gain,
            tau_s=CONFIG.control.dob_tau_s,
            max_speed_deg_s=CONFIG.control.max_dob_speed_deg_s,
        )

    def _integrate_command(self, u_pan, u_tilt):
        """Hesaplanan inkremental adimlari limitler dahilinde tarete ve ego modeline uygular."""
        new_pan = float(np.clip(self.turret_pan_deg + u_pan, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
        new_tilt = float(np.clip(self.turret_tilt_deg + u_tilt, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))
        pan_applied = new_pan - self.turret_pan_deg
        tilt_applied = new_tilt - self.turret_tilt_deg
        self.turret_pan_deg = new_pan
        self.turret_tilt_deg = new_tilt
        self.ego_model.command(pan_applied, tilt_applied)
        self.last_cmd_pan = u_pan
        self.last_cmd_tilt = u_tilt
        return u_pan, u_tilt

    def _compute_ego_pixel_shift(self, dt, fx, fy):
        """Servo yanitini (backlash dahil) gunceller ve 3B pinhole ego piksel kaymasini hesaplar."""
        point = np.zeros(2, dtype=float)
        if self.tracker.initialized:
            point = np.array([
                self.tracker.x[0, 0] - self._image_center[0],
                self.tracker.x[1, 0] - self._image_center[1],
            ], dtype=float)
        step, _, variance = self.ego_model.advance(dt, fx, fy, tau=self.ego_tau_s)
        shift = rotated_pixel_shift(point, step[0], step[1], fx, fy)
        self.ego_step_pan_deg = float(step[0])
        self.ego_step_tilt_deg = float(step[1])
        self.ego_var_x = float(variance[0])
        self.ego_var_y = float(variance[1])
        return float(shift[0]), float(shift[1])

    def _estimate_geom_distance(self, box_dim, fx, fy):
        """Hedef kutusunun acisal boyutundan mesafe tahmini (0.15 m yaricap varsayimi)."""
        if not box_dim or fx is None or fy is None:
            return None
        bw, bh = float(box_dim[0]), float(box_dim[1])
        if bw <= 0.0 or bh <= 0.0 or fx <= 1.0 or fy <= 1.0:
            return None
        theta_pan = np.arctan((bw / 2.0) / fx)
        theta_tilt = np.arctan((bh / 2.0) / fy)
        theta = 0.5 * (theta_pan + theta_tilt)
        if theta <= 1e-4:
            return None
        d_geom = float(CONFIG.lock.target_radius_m / np.tan(theta))
        return d_geom if np.isfinite(d_geom) and d_geom > CONFIG.lidar.min_distance_m else None

    def _update_lidar_range(self, dist_cm, is_firing, box_dim, fx, fy, box_center=None, frame_center=None):
        """
        TF-Luna menzil kapilama:
        - Ates sirasinda mesafeyi dondur (spin-up'ta duvar okumasini engelle).
        - Nisangah hedef kutusunun disindaysa (off_target) arka plan okumasini reddet.
        - Son gecerli mesafeden 50 cm'den fazla sicramayi reddet.
        - Ust uste reacquire_frames kare boyunca tutarli yeni mesafeyi gercek degisim olarak kabul et.
        - Kutu geometrisiyle celisen okumayi (duvar) reddet.
        - Ates icin 3 kare / 10 cm kararlilik iste.
        """
        L = CONFIG.lidar
        self.range_geom_m = self._estimate_geom_distance(box_dim, fx, fy)
        self.range_geom_mismatch = False

        if dist_cm is None or float(dist_cm) <= 0.0:
            self.range_valid = False
            self.range_stable = False
            self.range_jump = False
            self.range_status = "invalid"
            return

        d_new = float(dist_cm) / 100.0
        if not (L.min_distance_m <= d_new <= L.max_distance_m):
            self.range_valid = False
            self.range_stable = False
            self.range_jump = False
            self.range_status = "out_of_range"
            return

        # Ates sirasinda LiDAR'in arka plani okumasina izin verme.
        if is_firing:
            self.range_jump = False
            return

        # Nisangah merkezi yerine 3.30 cm asagidaki LiDAR lazer izinin (lidar_spot_y)
        # hedef kutusunun %60'lik elips cekirdegi icinde olup olmadigini denetle.
        if box_center is not None and frame_center is not None and box_dim is not None:
            bw, bh = float(box_dim[0]), float(box_dim[1])
            if bw > 0.0 and bh > 0.0:
                d_ref = float(self.last_valid_target_dist_m)
                if self.range_geom_m is not None:
                    d_ref = max(d_ref, float(self.range_geom_m))
                d_ref = max(L.min_distance_m, d_ref)
                fy_safe = float(fy) if (fy is not None and float(fy) > 1.0) else 1100.0
                # CAD v10: camera_to_lidar_z_m = -0.0330 m -> Goruntude (asagi +Y) +dy_lidar_px kayma
                dy_lidar_px = -fy_safe * (L.camera_to_lidar_z_m / d_ref)
                lidar_spot_x = float(frame_center[0])
                lidar_spot_y = float(frame_center[1]) + dy_lidar_px

                core_rx = max(1.0, 0.5 * bw * L.core_ellipse_ratio)
                core_ry = max(1.0, 0.5 * bh * L.core_ellipse_ratio)
                norm_dx = (lidar_spot_x - float(box_center[0])) / core_rx
                norm_dy = (lidar_spot_y - float(box_center[1])) / core_ry
                if (norm_dx * norm_dx + norm_dy * norm_dy) > 1.0:
                    self.range_valid = False
                    self.range_stable = False
                    self.range_jump = False
                    self.range_status = "off_target"
                    return

        geom_bad = False
        if self.range_geom_m is not None and self.range_geom_m > 0.3:
            ratio = abs(d_new - self.range_geom_m) / self.range_geom_m
            geom_bad = ratio > L.geometry_tolerance_ratio
        self.range_geom_mismatch = geom_bad

        last = float(self.last_valid_target_dist_m)
        jump = bool(self._range_history) and (abs(d_new - last) > L.jump_threshold_m)
        self.range_jump = jump

        if jump:
            self.range_valid = False
            self.range_stable = False
            self.range_status = "jump_rejected"
            # Duvar/arka plan ise adayi hic kabul etme.
            if geom_bad:
                self._range_candidate = None
                self._range_candidate_count = 0
                return
            # Gercek hedef mesafesi degismis olabilir: reacquire_frames kare tutarliysa kabul et.
            if self._range_candidate is None or abs(d_new - self._range_candidate) > L.stability_tolerance_m:
                self._range_candidate = d_new
                self._range_candidate_count = 1
            else:
                self._range_candidate_count += 1
            if self._range_candidate_count >= L.reacquire_frames:
                self.last_valid_target_dist_m = d_new
                self._range_has_reference = True
                self._range_history.clear()
                self._range_history.append(d_new)
                self._range_candidate = None
                self._range_candidate_count = 0
                self.range_jump = False
                self.range_valid = True
                self.range_stable = False
                self.range_status = "valid"
            return

        # Normal, sicramasiz okuma.
        self._range_candidate = None
        self._range_candidate_count = 0
        self.last_valid_target_dist_m = d_new
        self._range_has_reference = True
        self._range_history.append(d_new)
        self.range_valid = True
        if len(self._range_history) >= L.stability_window:
            self.range_stable = (max(self._range_history) - min(self._range_history)) <= L.stability_tolerance_m
        else:
            self.range_stable = False
        self.range_status = "stable" if self.range_stable else "valid"

    def _reset_lidar_range(self):
        """Hedef kaybinda filtre gecmisini sifirla; son gecerli mesafeyi koru."""
        self.range_valid = False
        self.range_stable = False
        self.range_jump = False
        self.range_geom_mismatch = False
        self.range_status = "unavailable"
        self._range_has_reference = False
        self._range_history.clear()
        self._range_candidate = None
        self._range_candidate_count = 0

    def _init_rl(self):
        """PPO politika agirligini yalnizca RL modu secildiginde tembel (lazy) yukler."""
        if self.rl_controller is None:
            from .rl_controller import RLController
            self.rl_controller = RLController()
        return self.rl_controller

    @property
    def is_locked(self):
        return self.lock_mgr.is_locked

    @property
    def lock_counter(self):
        return self.lock_mgr.stability_counter

    @property
    def LOCK_THRESHOLD_FRAMES(self):
        return self.lock_mgr.lock_threshold_frames

    @property
    def hit_probability(self):
        return self.lock_mgr.hit_probability

    @property
    def kalman(self):
        """Kalman filtresi referansi (telemetri ve harici analitikler icin)."""
        return self.tracker

    @property
    def lock_quality(self):
        return self.lock_mgr.lock_quality

    @property
    def actual_pan(self):
        return self.p_controller.actual_pan

    @property
    def actual_tilt(self):
        return self.p_controller.actual_tilt

    @property
    def target_conf(self):
        return getattr(self.selector, "last_target_conf", 0.0)

    def select_best_target(self, boxes, center_x, center_y, dt=None):
        dt = self.p_controller.normalize_dt(dt)
        self._image_center = (float(center_x), float(center_y))
        fx = CONFIG.optics.compute_fx(int(center_x * 2))
        fy = CONFIG.optics.compute_fy(int(center_y * 2))
        if self.tracker.initialized:
            self._frame_ego = self._compute_ego_pixel_shift(dt, fx, fy)
        else:
            self.ego_step_pan_deg = 0.0
            self.ego_step_tilt_deg = 0.0
            self.ego_var_x = 0.0
            self.ego_var_y = 0.0
            self._frame_ego = (0.0, 0.0)
        self._ego_prepared = True

        shift = (0.0, 0.0)
        assoc_center = self.selector.association_center
        if assoc_center is not None and (self.ego_step_pan_deg != 0.0 or self.ego_step_tilt_deg != 0.0):
            point = np.asarray(assoc_center, dtype=float) - np.asarray(self._image_center, dtype=float)
            shift = tuple(map(float, rotated_pixel_shift(point, self.ego_step_pan_deg, self.ego_step_tilt_deg, fx, fy)))

        return self.selector.select_best_target(boxes, center_x, center_y, dt=dt, ego_shift=shift)

    def _update_tracking_and_ballistics(self, best_box, center_x, center_y, fx, fy, dt, is_firing, dist_cm):
        """Hedef acilari, menzil kapilama, ego-hareket, Kalman hiz kestirimi ve kilit durumunu gunceller."""
        self.frames_since_lost = 0
        self.consecutive_target_frames += 1
        x1, y1, x2, y2 = best_box
        bx = (x1 + x2) // 2
        by = (y1 + y2) // 2
        box_dim = (float(x2 - x1), float(y2 - y1))

        # Devriye modulune hedefin son bilinen taret koordinatlarini bildir
        self.patrol.on_target_detected(
            self.turret_pan_deg,
            self.turret_tilt_deg,
            confirmed=(self.consecutive_target_frames >= self.patrol.confirm_frames),
        )
        self.patrol_status = "TRACKING"

        # 1. Kamera merkezinden hedef acilari ve acisal hata
        pan_deg, tilt_deg = self.p_controller.compute_angles(bx, by, center_x, center_y, fx, fy, dt=dt)
        # Telemetri icin ham kamera hatasi (kontrolcu ici EMA'dan bagimsiz).
        self.last_pan_err_deg = float(np.degrees(np.arctan((bx - center_x) / max(float(fx), 1e-6))))
        self.last_tilt_err_deg = float(np.degrees(np.arctan(-(by - center_y) / max(float(fy), 1e-6))))
        angular_err = float(np.hypot(pan_deg, tilt_deg))

        # 2. Menzil Kapilama (Range Gating) - kutu ici (off_target) dogrulamasi ve geometrik fallback ile
        self._update_lidar_range(
            dist_cm, is_firing, box_dim, fx, fy,
            box_center=((x1 + x2) / 2.0, (y1 + y2) / 2.0),
            frame_center=(center_x, center_y),
        )
        if self.range_valid or self._range_has_reference:
            d_m = self.last_valid_target_dist_m
        elif self.range_geom_m is not None:
            d_m = self.range_geom_m
        else:
            d_m = self.last_valid_target_dist_m
        effective_dist_cm = float(d_m * 100.0)
        if CONFIG.ballistics.drag_enabled:
            _, t_flight_rk4 = compute_rk4_super_elevation(
                d_m,
                v0=self.p_controller.PROJECTILE_SPEED_MPS,
                g=self.p_controller.GRAVITY_ACCEL,
                k=CONFIG.ballistics.drag_k,
            )
            raw_lead_time = t_flight_rk4 + CONFIG.control.lead_time_offset_s
        else:
            raw_lead_time = (d_m / self.p_controller.PROJECTILE_SPEED_MPS) + CONFIG.control.lead_time_offset_s

        lead_time = float(np.clip(
            raw_lead_time,
            CONFIG.control.lead_time_min_s,
            CONFIG.control.lead_time_max_s,
        ))
        self.lead_time = lead_time

        # 3. Ego-Motion (son motor komutunun servo/backlash yaniti ve 3B pinhole kaymasi)
        if not self._ego_prepared:
            if self.tracker.initialized:
                ego_dx, ego_dy = self._compute_ego_pixel_shift(dt, fx, fy)
            else:
                self.ego_step_pan_deg = 0.0
                self.ego_step_tilt_deg = 0.0
                self.ego_var_x = 0.0
                self.ego_var_y = 0.0
                ego_dx, ego_dy = 0.0, 0.0
        else:
            ego_dx, ego_dy = self._frame_ego
        self._ego_prepared = False

        # 4. Kalman filtresi, balistik onleme ve yercekimi telafisi
        _, _, kvx, kvy = self.tracker.update(
            (float(bx), float(by)),
            dt=dt,
            ego_dx=ego_dx,
            ego_dy=ego_dy,
            ego_var_x=self.ego_var_x,
            ego_var_y=self.ego_var_y,
        )
        self.target_velocity = np.array([kvx, kvy], dtype=float)

        d_lead_x = kvx * lead_time
        d_lead_y = kvy * lead_time
        self.pred_center = np.array([bx + d_lead_x, by + d_lead_y], dtype=float)

        lead_pan, lead_tilt = self.p_controller.compute_lead_angles(d_lead_x, d_lead_y, fx, fy)
        self.firing_lead_offset = np.array([lead_pan, lead_tilt], dtype=float)
        self.gravity_comp_deg = self.p_controller.compute_gravity_comp(effective_dist_cm)

        # 5. Kilit durumu kontrolu (Tam 2B Gauss integrali ve hedef kutu boyutu ile)
        self.lock_mgr.update(
            angular_err,
            self.target_velocity,
            is_firing=is_firing,
            dist_m=d_m,
            fx=fx,
            fy=fy,
            box_dim=box_dim,
            target_pos_deg=(pan_deg, tilt_deg),
        )
        return bx, by, pan_deg, tilt_deg, d_m

    def _select_control_command(self, bx, by, center_x, center_y, fx, fy, pan_deg, tilt_deg, d_m, dt, slew_factor):
        """Slew rampasi ileri beslemesini ve P / RL kontrolcu komut secimini hesaplar."""
        slew = float(np.clip(slew_factor, 0.0, 1.0))
        applied_lead_pan = float(slew * self.firing_lead_offset[0])
        applied_lead_tilt = float(slew * (self.firing_lead_offset[1] + self.gravity_comp_deg))

        # Aim-Point Ileri Besleme (Direct Feedforward): Slew rampasinin anlik turevi
        delta_slew_pan = float(applied_lead_pan - self.prev_applied_lead_pan)
        delta_slew_tilt = float(applied_lead_tilt - self.prev_applied_lead_tilt)
        self.prev_applied_lead_pan = applied_lead_pan
        self.prev_applied_lead_tilt = applied_lead_tilt

        active_pan = float(pan_deg + applied_lead_pan)
        active_tilt = float(tilt_deg + applied_lead_tilt)

        u_p_pan, u_p_tilt = self.p_controller.compute_p(active_pan, active_tilt, dt=dt)
        step_limit_pan = self.p_controller.max_step_for(dt, axis="pan")
        step_limit_tilt = self.p_controller.max_step_for(dt, axis="tilt")

        # Disturbance Observer (DOB) kompanzasyonu
        dob_pan, dob_tilt = self.dob.update(active_pan, active_tilt, u_p_pan, u_p_tilt, dt)

        u_p_pan = float(np.clip(u_p_pan + delta_slew_pan + dob_pan, -step_limit_pan, step_limit_pan))
        u_p_tilt = float(np.clip(u_p_tilt + delta_slew_tilt + dob_tilt, -step_limit_tilt, step_limit_tilt))

        if self.control_mode == "RL":
            if self.rl_controller is None:
                self._init_rl()
            if self.rl_controller.is_loaded:
                screen_w = center_x * 2
                screen_h = center_y * 2
                eff_bx = float(bx) + float(np.tan(np.radians(applied_lead_pan))) * fx
                eff_by = float(by) - float(np.tan(np.radians(applied_lead_tilt))) * fy

                # RL hiz gozlemi kasitli olarak sifirlanir (Kalman gurultusunun limit cycle uretmesini onler)
                obs_vx, obs_vy = 0.0, 0.0
                d_pan_deg, d_tilt_deg = self.rl_controller.predict_angles(
                    eff_bx, eff_by, center_x, center_y, screen_w, screen_h,
                    obs_vx, obs_vy, d_m, active_pan, active_tilt, dt
                )
                u_pan = float(np.clip(d_pan_deg + delta_slew_pan, -step_limit_pan, step_limit_pan))
                u_tilt = float(np.clip(d_tilt_deg + delta_slew_tilt, -step_limit_tilt, step_limit_tilt))
                return self._integrate_command(u_pan, u_tilt)

        return self._integrate_command(u_p_pan, u_p_tilt)

    def _handle_target_lost(self, dt, fx=None, fy=None):
        """Hedef kaybinda tolerans sayacini, coasting kestirimini, sifirlamayi ve Figure-8 devriye hareketini yonetir."""
        if fx is None:
            fx = CONFIG.optics.compute_fx(CONFIG.optics.default_width)
        if fy is None:
            fy = CONFIG.optics.compute_fy(CONFIG.optics.default_height)
        self.frames_since_lost += 1
        self.consecutive_target_frames = 0
        self.last_pan_err_deg = 0.0
        self.last_tilt_err_deg = 0.0
        self._reset_lidar_range()
        self.p_controller.on_target_lost()
        if self.rl_controller is not None and hasattr(self.rl_controller, "on_target_lost"):
            self.rl_controller.on_target_lost()
        if self.frames_since_lost >= self.max_lost_frames:
            self.tracker.reset()
            self.lock_mgr.reset()
            self.target_velocity = np.array([0.0, 0.0], dtype=float)
            self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
            self.ego_step_pan_deg = 0.0
            self.ego_step_tilt_deg = 0.0
            self.ego_var_x = 0.0
            self.ego_var_y = 0.0
            self.ego_model.reset(self.turret_pan_deg, self.turret_tilt_deg)
            self.prev_applied_lead_pan = 0.0
            self.prev_applied_lead_tilt = 0.0
        elif self.tracker.initialized:
            if not self._ego_prepared:
                ego_dx, ego_dy = self._compute_ego_pixel_shift(dt, fx, fy)
            else:
                ego_dx, ego_dy = self._frame_ego
            self._ego_prepared = False
            px, py, kvx, kvy = self.tracker.predict_only(
                dt=dt, ego_dx=ego_dx, ego_dy=ego_dy,
                ego_var_x=self.ego_var_x, ego_var_y=self.ego_var_y,
            )
            self.target_velocity = np.array([kvx, kvy], dtype=float)
            self.pred_center = np.array([px, py], dtype=float)

        self.p_controller.actual_pan = 0.0
        self.p_controller.actual_tilt = 0.0
        if self.patrol_enabled:
            d_p, d_t, self.patrol_status = self.patrol.update(
                dt, current_pan=self.turret_pan_deg, current_tilt=self.turret_tilt_deg
            )
            return self._integrate_command(d_p, d_t)

        self.patrol_status = "STANDBY"
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0
        return 0.0, 0.0

    def update(self, best_box, center_x, center_y, fx, fy, dt, is_firing=False, dist_cm=None, slew_factor=0.0):
        """Hedef kutusuna gore takip, balistik, kilit ve motor komut adimini (d_pan, d_tilt) hesaplar."""
        dt = self.p_controller.normalize_dt(dt)
        self._image_center = (float(center_x), float(center_y))
        if best_box is not None:
            bx, by, pan_deg, tilt_deg, d_m = self._update_tracking_and_ballistics(
                best_box, center_x, center_y, fx, fy, dt, is_firing, dist_cm
            )
            return self._select_control_command(
                bx, by, center_x, center_y, fx, fy, pan_deg, tilt_deg, d_m, dt, slew_factor
            )
        return self._handle_target_lost(dt, fx=fx, fy=fy)

    def sync_turret_pose(self, pan_deg, tilt_deg, force=False):
        """STM32 geri beslemesinden donanimsal (sensed) taret pozunu gunceller."""
        if pan_deg is None or tilt_deg is None:
            return

        self.hw_pan_deg = float(np.clip(pan_deg, self.PAN_LIMIT_MIN, self.PAN_LIMIT_MAX))
        self.hw_tilt_deg = float(np.clip(tilt_deg, self.TILT_LIMIT_MIN, self.TILT_LIMIT_MAX))

        if force or not self._has_initial_hw_sync:
            self.turret_pan_deg = self.hw_pan_deg
            self.turret_tilt_deg = self.hw_tilt_deg
            self.ego_model.reset(self.hw_pan_deg, self.hw_tilt_deg)
            self._ego_prepared = False
            self._has_initial_hw_sync = True
            self.tracking_lag_deg = 0.0
            self.is_stalled = False
            return

        # Komut ile donanim arasindaki anlik mekanik takip gecikmesi (lag)
        err_p = self.turret_pan_deg - self.hw_pan_deg
        err_t = self.turret_tilt_deg - self.hw_tilt_deg
        self.tracking_lag_deg = float(np.hypot(err_p, err_t))
        self.is_stalled = self.tracking_lag_deg > 8.0

        # Duragan durumda tamamlayici filtre ile kayma telafisi (soft drift correction)
        if abs(self.last_cmd_pan) < 0.05 and abs(self.last_cmd_tilt) < 0.05:
            self.turret_pan_deg -= 0.15 * err_p
            self.turret_tilt_deg -= 0.15 * err_t

    def reset(self):
        self.frames_since_lost = 0
        self.consecutive_target_frames = 0
        self.patrol_status = "STANDBY"
        self._reset_lidar_range()
        self.last_valid_target_dist_m = float(CONFIG.ballistics.default_distance_m)
        self.target_velocity = np.array([0.0, 0.0], dtype=float)
        self.pred_center = np.array([0.0, 0.0], dtype=float)
        self.firing_lead_offset = np.array([0.0, 0.0], dtype=float)
        self.gravity_comp_deg = 0.0
        self.last_pan_err_deg = 0.0
        self.last_tilt_err_deg = 0.0
        self.last_cmd_pan = 0.0
        self.last_cmd_tilt = 0.0
        self.turret_pan_deg = 0.0
        self.turret_tilt_deg = 0.0
        self.hw_pan_deg = 0.0
        self.hw_tilt_deg = 0.0
        self._has_initial_hw_sync = False
        self.tracking_lag_deg = 0.0
        self.is_stalled = False
        self.ego_step_pan_deg = 0.0
        self.ego_step_tilt_deg = 0.0
        self.ego_var_x = 0.0
        self.ego_var_y = 0.0
        self._frame_ego = (0.0, 0.0)
        self._ego_prepared = False
        self.ego_model.reset()
        self.prev_applied_lead_pan = 0.0
        self.prev_applied_lead_tilt = 0.0
        self.tracker.reset()
        self.lock_mgr.reset()
        self.p_controller.reset()
        if self.rl_controller is not None:
            self.rl_controller.reset()
        self.patrol.reset()
        self.selector.reset()
