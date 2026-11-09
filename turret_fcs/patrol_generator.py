"""
================================================================================
MODUL: SAKIN MERKEZE DONUS VE LISSAJOUS SEKIZ (FIGURE-8) DEVIYE URETECI
================================================================================
Bu modul, hedef gorunmediginde taretin sakin ve kararli bir sekilde hava sahasi
taramasi yapmasini saglar:

1. HEDEF KAYBOLDUGUNDA (1.5 SANIYE BEKLEME):
   - Hedef kadrajdan ciktiginda veya anlik kesintilerde taret hemen yerinden oynamaz.
   - Son goruldugu konumda 1.5 saniye sakince bekler. Hedef tekrar belirirse
     hic motor hareketi yapmadan aninda takibe devam eder.

2. MERKEZE DONUS (0°, 0°):
   - 1.5 saniye icinde hedef donmezse, taret sakin ve orta duzey bir hizla (20°/s)
     merkez (0°, 0°) noktasina doner.

3. LISSAJOUS SEKIZ (FIGURE-8) HAVA SAHASI TARAMASI (Pan: +/- 45°, Tilt: +/- 16°, 8.0s Periyot):
   - Merkeze ulasinca hem yatay hem dikey eksenleri harmonik frekanslarda (1:2 oraninda)
     calistirarak gokyuzunde genis bir sonsuzluk / sekiz motifi (Figure-8) taramasi yapar.
   - Merkezden sekiz motifine gecerken yumusak bir gecis ile acilir (sifir jerk/vuruntu).
   - Koseli hareket olmadigi icin servolar surekli akici, titresimsiz ve sakin doner.

4. HEDEF GORULDUGUNDE:
   - Devriye aninda devreden cikar, kontrol P takip kontrolcusune gecer.
================================================================================
"""

import numpy as np

from .config import CONFIG


class PatrolGenerator:
    """
    1.5s hold, smooth return-to-center (odometry-aware), and Figure-8 airspace scan generator.
    """

    def __init__(
        self,
        wait_delay_sec=CONFIG.patrol.wait_delay_sec,
        scan_pan_amp=CONFIG.patrol.scan_pan_amp,
        scan_tilt_amp=CONFIG.patrol.scan_tilt_amp,
        scan_period_sec=CONFIG.patrol.scan_period_sec,
        return_speed_dps=CONFIG.patrol.return_speed_dps,
        max_slew_per_frame=CONFIG.patrol.max_slew_per_frame,
        min_dispatch_deg=CONFIG.patrol.min_dispatch_deg,
        confirm_frames=CONFIG.patrol.confirm_frames,
    ):
        self.wait_delay_sec = float(wait_delay_sec)
        self.scan_pan_amp = float(scan_pan_amp)
        self.scan_tilt_amp = float(scan_tilt_amp)
        self.scan_freq = float(2.0 * np.pi / max(0.5, scan_period_sec))
        self.return_speed_dps = float(return_speed_dps)
        self.max_slew = float(max_slew_per_frame)
        self.confirm_frames = max(1, int(confirm_frames))

        self.state = "WAITING"   # "WAITING", "RETURNING", "SCANNING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # Kesintisiz geri donus adimlayici (0.1 deg altindaki kusuratlar FCSCommunicator'da birikir)
        self.accum_pan = 0.0
        self.accum_tilt = 0.0
        self.min_dispatch_deg = float(min_dispatch_deg)

    def _rate_scaled_slew(self, dt):
        """STM32 Servo_ApplyCorrection (45 deg/s * dt) hiz siniriyla uyumlu kare basi adim tavanini hesaplar."""
        nominal_dt = max(1e-6, float(CONFIG.rl.nominal_dt_s))
        return float(min(self.max_slew, self.max_slew * (dt / nominal_dt)))

    def on_target_detected(self, actual_pan=0.0, actual_tilt=0.0, confirmed=True):
        """Reset patrol generator upon confirmed target acquisition."""
        self.accum_pan = 0.0
        self.accum_tilt = 0.0
        self.current_pan = float(actual_pan)
        self.current_tilt = float(actual_tilt)
        if confirmed or self.state == "WAITING":
            self.state = "WAITING"
            self.lost_timer = 0.0
            self.scan_time = 0.0

    def update(self, dt, current_pan=None, current_tilt=None):
        """
        Calculates the next patrol step (delta_pan, delta_tilt) and status text.
        """
        dt = 0.033 if (dt is None or dt <= 0.0) else float(dt)
        curr_p = self.current_pan if current_pan is None else float(current_pan)
        curr_t = self.current_tilt if current_tilt is None else float(current_tilt)
        max_step = self._rate_scaled_slew(dt)

        # STAGE 1: HOLD LAST KNOWN POSITION FOR 1.5s
        if self.state == "WAITING":
            self.current_pan = curr_p
            self.current_tilt = curr_t
            self.lost_timer += dt
            if self.lost_timer < self.wait_delay_sec:
                remaining = self.wait_delay_sec - self.lost_timer
                return 0.0, 0.0, f"HOLDING ({remaining:.1f}s)"
            else:
                self.state = "RETURNING"

        # STAGE 2: SMOOTH CONTINUOUS RETURN TO BORESIGHT CENTER (0, 0)
        if self.state == "RETURNING":
            err_pan = -curr_p
            err_tilt = -curr_t
            dist_to_center = float(np.hypot(err_pan, err_tilt))

            # Merkeze tam yerlestiğinde (veya cok yaklastiginda) taramaya gec
            if dist_to_center < 0.6:
                self.state = "SCANNING"
                self.scan_time = 0.0
                self.accum_pan = 0.0
                self.accum_tilt = 0.0
                self.current_pan = curr_p
                self.current_tilt = curr_t
                return 0.0, 0.0, "BORESIGHT LOCKED -> COMMENCING SCAN"

            step = min(self.return_speed_dps * dt, max_step)
            dir_pan = float(np.sign(err_pan))
            dir_tilt = float(np.sign(err_tilt))

            self.accum_pan += dir_pan * min(abs(err_pan), step)
            self.accum_tilt += dir_tilt * min(abs(err_tilt), step)

            delta_p = 0.0
            if abs(self.accum_pan) >= self.min_dispatch_deg or abs(err_pan) <= self.min_dispatch_deg:
                delta_p = float(np.clip(self.accum_pan, -max_step, max_step))
                self.accum_pan -= delta_p

            delta_t = 0.0
            if abs(self.accum_tilt) >= self.min_dispatch_deg or abs(err_tilt) <= self.min_dispatch_deg:
                delta_t = float(np.clip(self.accum_tilt, -max_step, max_step))
                self.accum_tilt -= delta_t

            self.current_pan = curr_p + delta_p
            self.current_tilt = curr_t + delta_t
            return delta_p, delta_t, f"RTB BORESIGHT ({dist_to_center:.1f} deg)"

        # STAGE 3: SMOOTH FIGURE-8 LISSAJOUS SCAN PATTERN (+/-45 x +/-16 deg)
        elif self.state == "SCANNING":
            self.scan_time += dt

            # 2.0s smooth spiral expansion from center (zero step-jerk)
            ramp = min(1.0, self.scan_time / 2.0)

            # 1:2 Figure-8 Lissajous infinity scan (passes through center twice per loop)
            target_pan = ramp * self.scan_pan_amp * float(np.sin(self.scan_freq * self.scan_time))
            target_tilt = ramp * self.scan_tilt_amp * float(np.sin(2.0 * self.scan_freq * self.scan_time))

            err_pan = target_pan - curr_p
            err_tilt = target_tilt - curr_t

            delta_p = float(np.clip(err_pan, -max_step, max_step))
            delta_t = float(np.clip(err_tilt, -max_step, max_step))

            self.current_pan = curr_p + delta_p
            self.current_tilt = curr_t + delta_t
            return delta_p, delta_t, f"FIGURE-8 SCAN (+/-{int(self.scan_pan_amp)} x +/-{int(self.scan_tilt_amp)} deg)"

        return 0.0, 0.0, "STANDBY"

    def reset(self):
        """Resets the patrol generator state."""
        self.state = "WAITING"
        self.lost_timer = 0.0
        self.scan_time = 0.0
        self.accum_pan = 0.0
        self.accum_tilt = 0.0
        self.current_pan = 0.0
        self.current_tilt = 0.0
