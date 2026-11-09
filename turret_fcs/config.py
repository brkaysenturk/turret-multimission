"""
================================================================================
MODUL: TARET ATIS KONTROL SISTEMI KONFIGURASYONU (SSOT - CONFIG)
================================================================================
Tum alt sistemlerin (Optik, Kontrol, Balistik, Eyleyici, Kilit, Yollar ve Durum)
ortak fiziksel ve algoritmik parametrelerini tek bir kaynakta (Single Source of Truth)
toplayan, harici kutuphane bagimliligi olmayan saf yaprak moduldur.
================================================================================
"""

from dataclasses import dataclass
from enum import IntEnum
import math


class FCSState(IntEnum):
    """
    FCS durum makinesi ve STM32 donanim protokol durumlari.
    STM32 firmware main.c protokolu ile birebir eslesir:
      0: TRACKING  (Hedef arama / dinamik takip)
      1: LOCKED    (Hedef kilit alaninda)
      2: FIRING    (Atis / Solenoid tahrigi - volan hazirsa solenoid itici tetiklenir)
      3: ESTOP     (Acil durum durdurma / tum motorlar enerjisiz)
    """
    TRACKING = 0
    LOCKED = 1
    FIRING = 2
    ESTOP = 3


@dataclass(frozen=True)
class OpticsConfig:
    """Logitech C270 kamera optik parametreleri."""
    h_fov_deg: float = 48.8
    v_fov_deg: float = 28.6
    default_width: int = 1280
    default_height: int = 720
    default_fps: int = 30

    def compute_fx(self, width: int = 1280) -> float:
        return float((width / 2.0) / math.tan(math.radians(self.h_fov_deg / 2.0)))

    def compute_fy(self, height: int = 720) -> float:
        return float((height / 2.0) / math.tan(math.radians(self.v_fov_deg / 2.0)))


@dataclass(frozen=True)
class ControlConfig:
    """P kontrolcu ve filtre parametreleri."""
    kp_pan: float = 0.1921      # P: Coklu Mesafe (1m, 2m, 3m) & Coklu Profil Optuna optimal kazanc
    kp_tilt: float = 0.1669     # P: Coklu Mesafe (1m, 2m, 3m) & Coklu Profil Optuna optimal kazanc
    deadband_pan_deg: float = 0.766
    deadband_tilt_deg: float = 0.530
    max_output_deg: float = 1.50
    max_command_step_deg: float = 1.50  # Firmware Servo_ApplyCorrection ust siniri
    cmd_accel_pan_deg_s2: float = 556.58  # P/RL pan komut ivme siniri (Coklu Mesafe Optuna optimal)
    cmd_accel_tilt_deg_s2: float = 211.81  # P/RL tilt komut ivme siniri (Coklu Mesafe Optuna optimal)
    asymmetric_braking: bool = True      # Oneri 1: Hedefe varista/yavaslamada ivme sinirini kaldir (sifir overshoot)
    in_flight_damping: float = 0.6875      # Oneri 2: Yoldaki (in-flight) 3-kare komut yaklasim sonumlemesi
    lead_time_offset_s: float = 0.090    # Balistik lead'e eklenen sabit pay (system_latency_s ile tam senkron: 90 ms)
    lead_time_min_s: float = 0.03        # Lead sure alt siniri
    lead_time_max_s: float = 0.40        # Lead sure ust siniri (dart ucus suresi olcegi)
    alpha_filter: float = 0.9475         # P giris EMA'sinda yeni deger agirligi (Coklu Mesafe Optuna optimal)
    ego_tau_s: float = 0.0300            # Ego icin komut adimi servo zaman sabiti (30 ms fiziksel taret)
    ego_motion_uncertainty_ratio: float = 0.35  # Olculmeyen servo hareketi bagil standart sapmasi
    ego_backlash_uncertainty_ratio: float = 0.25  # Disli boslugu alimi sirasindaki belirsizlik orani
    dob_enabled: bool = False            # Disturbance Observer (DOB) anahtari (True: acik, False: kapali)
    dob_gain: float = 1.2233             # DOB kazanci (Optuna TPE optimal)
    dob_tau_s: float = 0.1710            # DOB Q-filtresi zaman sabiti (saniye)
    max_dob_speed_deg_s: float = 12.0666 # DOB maksimum kompanzasyon hiz siniri (deg/s)


@dataclass(frozen=True)
class ActuatorConfig:
    """Eyleyici ve servo mekanik adim / kuantalama limitleri."""
    max_step_pan_deg: float = 1.50     # RL kare basi maksimum Pan adimi (30 FPS'te 45 deg/s)
    max_step_tilt_deg: float = 0.75    # RL kare basi maksimum Tilt adimi (30 FPS'te 22.5 deg/s - donanim eylemsizlik uyumu)
    servo_deadband_deg: float = 0.10   # STM32 0.1 deg UART kuantalama ve servo mekanik olu bolgesi
    backlash_deg: float = 1.61         # MG996R disli boslugu (canli telemetri Sistem Tanilama ile olculen 1.61 deg)


@dataclass(frozen=True)
class KalmanConfig:
    """Sabit hizli (CV) Kalman filtresi: CWNA Q/R cifti, NIS ve 100 px tekil-sicrama kapisi."""
    r_var: float = 63.21                 # Olcum gurultusu varyansi (px^2), Bayesyen Optuna TPE optimal
    sigma_a: float = 478.19              # Surec ivme gurultusu (px/s^2), Bayesyen Optuna TPE optimal
    noise_reference_dt_s: float = 1.0 / 30.0  # CWNA spektral yogunluk referans periyodu (sigma_a^2 * dt_ref)
    innovation_gate_chi2: float = 13.81551056  # 2 serbestlik derecesi, %99.9 istatistiksel inovasyon kapisi
    max_prediction_step_s: float = 0.10  # Uzun dt araliklarinda maksimum alt-adim (sub-step) suresi
    reacquire_window_s: float = 0.35     # Aykiri adaylarin gecerlilik zaman penceresi (saniye)
    initial_velocity_var: float = 90000.0  # Ilk hiz kovaryansi (px/s)^2, hizli yakinsama icin
    velocity_deadband_low_px_s: float = 30.0   # Bu hizin altinda cikis hizi sifir
    velocity_deadband_high_px_s: float = 60.0  # Bu hizdan itibaren cikis hizi tam gecerli
    outlier_jump_px: float = 100.0       # Tekil sahte kutu sicrama esigi (piksel inovasyon normu)
    reacquire_frames: int = 3            # Ust uste aykiri deger sonrasi yeni takip baslatma esigi


@dataclass(frozen=True)
class BallisticsConfig:
    """Dis balistik, yercekimi kompanzasyonu ve atesleme zamanlamalari."""
    projectile_speed_mps: float = 10.0
    gravity_accel: float = 9.80665
    default_distance_m: float = 2.0
    # Aerodinamik surukleme (drag) parametreleri
    drag_enabled: bool = True
    projectile_mass_kg: float = 0.0012        # 1.2 gram sunger dart
    projectile_cross_area_m2: float = 1.131e-4  # 12 mm cap (pi * 0.006^2)
    drag_coefficient: float = 0.60            # Kup / yarim kure burunlu dart Cd katsayisi
    air_density_kg_m3: float = 1.225          # 15 C deniz seviyesi hava yogunlugu
    # Atis sekansi zamanlamalari (saniye; spin-up sureleri FlywheelConfig icindedir)
    settle_time_s: float = 0.15     # Slew sonrasi servo oturma (150 ms, MG996R FOPDT FSM oturma ve titreşim sönümleme)
    firing_pulse_s: float = 0.08    # Solenoid itici darbe suresi (80 ms - STM32 firmware ile uyumlu)
    recovery_time_s: float = 0.25   # Mekanik geri toplama suresi (250 ms)
    eval_display_time_s: float = 1.50  # Atis sonrasi EVAL gosterim suresi
    system_latency_s: float = 0.0900 # 90.0 ms (3.0 kare @ 30 FPS / TensorRT 5ms + Donanim 81.5ms olculen uctan uca gecikme)
    camera_to_muzzle_z_m: float = 0.0570  # CAD v10: Namlu ucu Z ekseninde kameradan 57 mm yukaridadir (paralaks)

    @property
    def drag_k(self) -> float:
        """Surukleme sabiti k = (0.5 * rho * Cd * A) / m (1/m)."""
        if not self.drag_enabled:
            return 0.0
        m = max(1e-6, self.projectile_mass_kg)
        return float((0.5 * self.air_density_kg_m3 * self.drag_coefficient * self.projectile_cross_area_m2) / m)


@dataclass(frozen=True)
class RLConfig:
    """RL cikarim ve egitim ortaminin ortak sabitleri (tek kaynak)."""
    soft_deadband_enabled: bool = True   # Acisal yumusak olu bolge + taper anahtari
    reversal_damping: float = 0.6637        # Yon degistiren aksiyonun sonum orani (Coklu Mesafe Optuna optimal)
    reversal_hold_frames: int = 3        # Yon degisim sonumunu 3 kare (~90ms, gecikme+backlash) koru
    taper_pan_deg: float = 0.400           # Soft deadband gecis genisligi (pan - Coklu Mesafe Optuna optimal)
    taper_tilt_deg: float = 0.469          # Soft deadband gecis genisligi (tilt - Coklu Mesafe Optuna optimal)
    deadband_pan_deg: float = 0.490      # RL acisal olu bolge (pan)
    deadband_tilt_deg: float = 0.621     # RL acisal olu bolge (tilt - Coklu Mesafe Optuna optimal)
    nominal_dt_s: float = 0.03333        # Egitim kare periyodu (adim limiti olcekleme referansi)
    d_err_scale: float = 0.05            # Hata turevi gozlem olcegi (egitimle ayni)
    d_err_ema_alpha: float = 0.4675        # Hata turevi EMA katsayisi (Coklu Mesafe Optuna optimal)
    v_max_px_s: float = 300.0            # Hiz gozlemi Bryson olcegi
    dist_min_m: float = 0.5              # Mesafe gozlemi alt siniri
    dist_max_m: float = 5.0              # Mesafe gozlemi ust siniri
    yolo_noise_std: float = 3.5          # Ortam varsayilan YOLO kutu gurultusu (px)
    yolo_noise_std_min: float = 2.0      # Domain randomization alt siniri
    yolo_noise_std_max: float = 5.0      # Domain randomization ust siniri
    servo_tau_min_s: float = 0.025       # Domain randomization servo zaman sabiti (olculen tau=0.030 s etrafinda)
    servo_tau_max_s: float = 0.035
    episode_steps: int = 300             # Egitim bolumu uzunlugu
    lambda_u: float = 0.0433             # Kontrol eforu ceza katsayisi (Optuna TPE 40x250k Trial #31 optimal)
    mu_du: float = 0.0338                # Komut degisimi (jerk) ceza katsayisi (Optuna TPE 40x250k Trial #31 optimal)
    progress_gain: float = 2.0           # Hata azalma ilerleme odulu katsayisi
    error_gain: float = 0.05             # Kalici karesel hata ceza katsayisi
    bullseye_reward: float = 0.50        # Tam merkez (bullseye) arti odulu
    bullseye_threshold_norm: float = 0.05  # Bullseye normalize hata yaricapi esigi


@dataclass(frozen=True)
class LidarConfig:
    """TF-Luna menzil kapilama, sicrama reddi ve kararlilik filtreleri."""
    min_distance_m: float = 0.20
    max_distance_m: float = 8.00
    jump_threshold_m: float = 0.50       # Son gecerli mesafeden 50 cm'den fazla sapma = supheli
    stability_window: int = 3            # Ates icin gereken kararli okuma sayisi
    stability_tolerance_m: float = 0.10  # Bu pencerede izin verilen mesafe dalgalanmasi
    reacquire_frames: int = 3            # Gercek menzil degisimini kabul icin ust uste kare
    geometry_tolerance_ratio: float = 0.35  # Kutu boyutundan mesafe ile LiDAR uyumsuzlugu (duvar sicrama kilidi)
    camera_to_lidar_z_m: float = -0.0330  # CAD v10: TF-Luna LiDAR kameranin 33.0 mm (3.30 cm) altindadir
    core_ellipse_ratio: float = 0.60      # Balon kuresel yansitici merkez elips orani (%60 cekirdek kapisi)


@dataclass(frozen=True)
class FlywheelConfig:
    """Cift volan motoru: kilitliyken bosta donme ve ates boost parametreleri."""
    target_speed_rad_s: float = 2460.9   # 23.500 RPM nominal atis devri
    spinup_time_s: float = 0.25          # Tekil spin-up suresi (0.25 s)
    duty_pct: int = 217                  # Nominal PWM devri (0..255; 217 = ~%85 nominal devir)


@dataclass(frozen=True)
class MountConfig:
    """Gimbal mekanik limitleri ve servo dinamikleri."""
    pan_limit_min: float = -75.0
    pan_limit_max: float = 75.0
    tilt_limit_min: float = -25.0
    tilt_limit_max: float = 25.0
    pan_invert: bool = True
    tilt_invert: bool = False
    servo_tau: float = 0.0300       # 1. derece motor zaman sabiti (FOPDT saf basamak ve Bode testleri olcumu)


@dataclass(frozen=True)
class LockConfig:
    """Kilit kararlilik sayaci ve histerezis parametreleri."""
    lock_threshold_frames: int = 10
    lock_exit_margin_deg: float = 0.60
    target_radius_m: float = 0.15
    sigma_weapon_deg: float = 1.25   # DIY cift volan ve sunger dart gercekci ic balistik sacilmasi (derece)
    min_hit_probability_pct: float = 25.0  # Atis izni icin gereken minimum analitik isabet olasiligi (%)
    post_fire_grace_duration_frames: int = 22  # 0.75 saniye @ 30 FPS kilit muafiyeti (grace period)
    gate_slow_deg: float = 1.70
    gate_fast_deg: float = 2.95
    dynamic_gate_margin_deg: float = 0.40  # Hata yasasi uzerine eklenen guvenlik marji (derece)


@dataclass(frozen=True)
class SelectorConfig:
    """Hedef secici ve filtreleme esikleri."""
    min_conf: float = 0.30
    track_min_conf: float = 0.16
    min_area: int = 400
    max_area: int = 400000
    min_dim: int = 18
    max_lost_frames: int = 5
    spatial_gate_px: float = 173.0
    min_aspect_ratio: float = 0.35
    max_aspect_ratio: float = 2.50
    yolo_imgsz: int = 640
    tracker_config: str = "bytetrack.yaml"
    acquisition_frames: int = 1
    handoff_min_iou: float = 0.60
    handoff_max_size_ratio: float = 1.35
    handoff_max_center_fraction: float = 0.25
    target_class_name: str = "balloon"


@dataclass(frozen=True)
class PatrolConfig:
    """Devriye, merkeze donus ve Figure-8 tarama parametreleri."""
    wait_delay_sec: float = 1.5
    scan_pan_amp: float = 45.0
    scan_tilt_amp: float = 16.0
    scan_period_sec: float = 8.0
    return_speed_dps: float = 20.0
    max_slew_per_frame: float = 1.5
    min_dispatch_deg: float = 0.0
    confirm_frames: int = 3


@dataclass(frozen=True)
class HUDConfig:
    """MIL-STD HUD esik, sinyal, menzil ve olay gunlugu parametreleri."""
    hit_high_pct: int = 70
    hit_mid_pct: int = 40
    signal_strong: int = 1000
    signal_moderate: int = 100
    max_display_range_cm: int = 300
    min_healthy_fps: int = 25
    max_event_logs: int = 6
    default_event_lines: int = 4


@dataclass(frozen=True)
class PathConfig:
    """Agirlik ve yapay zeka model dosya yollari."""
    yolo_model: str = "weights/best.pt"
    fallback_yolo_model: str = "weights/yolov8n.pt"


@dataclass(frozen=True)
class TurretConfig:
    optics: OpticsConfig = OpticsConfig()
    control: ControlConfig = ControlConfig()
    actuator: ActuatorConfig = ActuatorConfig()
    kalman: KalmanConfig = KalmanConfig()
    ballistics: BallisticsConfig = BallisticsConfig()
    rl: RLConfig = RLConfig()
    flywheel: FlywheelConfig = FlywheelConfig()
    lidar: LidarConfig = LidarConfig()
    mount: MountConfig = MountConfig()
    lock: LockConfig = LockConfig()
    selector: SelectorConfig = SelectorConfig()
    patrol: PatrolConfig = PatrolConfig()
    hud: HUDConfig = HUDConfig()
    paths: PathConfig = PathConfig()
    telemetry_enabled: bool = True


CONFIG = TurretConfig()
