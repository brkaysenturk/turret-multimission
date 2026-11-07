#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET BASKET 2-ASAMALI ANGAJMAN DURUM MAKINESI ZAMAN CIZELGESI ANALIZI
================================================================================
Bu betik; BasketEngagement durum makinesinin (FSM) ve fiziksel eyleyicilerin
(Tilt Servosu, TB6612FNG Volan Motorlari, 12V Solenoid Darbesi) 1.58 saniyelik
tam atis dongusundeki zaman-domen profilini akademik grafiklerle modeller:
  - Phase 1: SPINUP_ELEVATE (0.00 s - 0.60 s): Tilt yukselisi ve motor devir alma
  - Phase 2: FIRING (0.60 s - 0.68 s / 80 ms): STM32 donanimsal darbe tetiklemesi
  - Phase 3: RECOVER (0.68 s - 1.58 s): Solenoid yayla donusu, tilt inisi ve hazirlik
================================================================================
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

# Font ve stil ayarlari (Akademik yayin formati)
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 0.8

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[4]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

FIG_DIR = PROJECT_ROOT / "turret_basket" / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

from turret_basket.engagement import BasketEngagement
from turret_basket.basket_manager import BasketManager


def plot_fig4_engagement_profile():
    """Figure 4: 4-Asamali Firlatma Durum Makinesi Zaman Cizelgesi ve Eyleyici Profili."""
    eng = BasketEngagement()
    mgr = BasketManager(comm=None)

    # 100 Hz zaman simülasyonu (0.0 saniye ile 1.55 saniye arasi)
    dt = 0.01
    times = np.arange(0.0, 1.55, dt)

    tilt_at_lock = 0.0
    fire_speed = 4.4
    fire_tilt = 21.8
    super_elevation = 21.8
    target_pwm = mgr.compute_flywheel_pwm(fire_speed)

    # Atisi 0.05 saniyede tetikle
    t_trigger = 0.05
    triggered = False

    time_hist = []
    tilt_hist = []
    flywheel_pwm_hist = []
    solenoid_hist = []
    state_hist = []
    phase_hist = []

    curr_tilt = tilt_at_lock
    curr_pwm = 0.0
    solenoid_val = 0.0

    for t in times:
        if not triggered and t >= t_trigger:
            eng.trigger(
                now=t,
                tilt_at_lock=tilt_at_lock,
                fire_speed=fire_speed,
                fire_tilt=fire_tilt,
                super_elevation=super_elevation,
            )
            triggered = True

        step = eng.update(now=t, current_tilt=curr_tilt)

        # Aktüatör dinamikleri simülasyonu (4-Asamali FSM: SPINUP_ELEVATE -> SETTLE -> FIRING -> RECOVER)
        if step.is_active:
            if step.phase == "SPINUP_ELEVATE":
                elapsed_phase = t - t_trigger
                prog = min(1.0, elapsed_phase / eng.spinup_time_s)
                smooth_p = prog * prog * (3.0 - 2.0 * prog)
                curr_tilt = tilt_at_lock + smooth_p * super_elevation
                curr_pwm = min(target_pwm, smooth_p * target_pwm)
                solenoid_val = 0.0

            elif step.phase == "SETTLE":
                curr_tilt = tilt_at_lock + super_elevation
                curr_pwm = target_pwm
                solenoid_val = 0.0

            elif step.phase == "FIRING":
                curr_tilt = tilt_at_lock + super_elevation
                curr_pwm = target_pwm
                solenoid_val = 1.0

            elif step.phase == "RECOVER":
                solenoid_val = 0.0
                elapsed_rec = t - (t_trigger + eng.spinup_time_s + eng.settle_time_s + eng.firing_pulse_s)
                prog_rec = min(1.0, max(0.0, elapsed_rec / eng.recovery_time_s))
                smooth_rec = prog_rec * prog_rec * (3.0 - 2.0 * prog_rec)
                curr_tilt = (tilt_at_lock + super_elevation) - smooth_rec * super_elevation
                curr_pwm = max(0.0, target_pwm * (1.0 - smooth_rec * 1.5))
        else:
            curr_tilt = tilt_at_lock
            curr_pwm = 0.0
            solenoid_val = 0.0

        time_hist.append(t)
        tilt_hist.append(curr_tilt)
        flywheel_pwm_hist.append(curr_pwm)
        solenoid_hist.append(solenoid_val)
        state_hist.append(step.cmd_state)
        phase_hist.append(step.phase if step.phase else "IDLE")

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    fig.patch.set_facecolor("#ffffff")
    fig.suptitle("Turret Basket: 4-Asamali Firlatma Durum Makinesi Zaman Cizelgesi ve Eyleyici Profili (1.33s)\n"
                 "SPINUP_ELEVATE (0.50s) -> SETTLE (0.25s) -> FIRING (80ms Solenoid) -> RECOVER (0.50s)",
                 fontsize=13, fontweight="bold", y=0.98, color="#0f172a")

    # Faz Arkaplan Renklendirmeleri (Tüm panellere)
    t_start = t_trigger
    t_spinup_end = t_start + eng.spinup_time_s
    t_settle_end = t_spinup_end + eng.settle_time_s
    t_fire_end = t_settle_end + eng.firing_pulse_s
    t_total_end = t_start + eng.total_seq_time_s

    for ax in axes:
        ax.set_facecolor("#fcfcfc")
        ax.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")
        ax.axvspan(t_start, t_spinup_end, color="#fef08a", alpha=0.35, label="SPINUP_ELEVATE (0.50s)" if ax == axes[0] else "")
        ax.axvspan(t_spinup_end, t_settle_end, color="#bae6fd", alpha=0.40, label="SETTLE (0.25s Mekanik Oturma)" if ax == axes[0] else "")
        ax.axvspan(t_settle_end, t_fire_end, color="#fca5a5", alpha=0.55, label="FIRING (0.08s Solenoid)" if ax == axes[0] else "")
        ax.axvspan(t_fire_end, t_total_end, color="#bbf7d0", alpha=0.35, label="RECOVER (0.50s)" if ax == axes[0] else "")

    # 1. Tilt Acisi Profili
    ax1 = axes[0]
    ax1.plot(time_hist, tilt_hist, color="#2563eb", linewidth=2.5, label="Tilt Servo Pozisyonu")
    ax1.axhline(super_elevation, color="#dc2626", linestyle=":", linewidth=1.4, label=f"Super-elevasyon Zirvesi (+{super_elevation:.1f}°)")
    ax1.axhline(0.0, color="#64748b", linestyle="--", linewidth=1.0, label="Optik Hedef Kilit Seviyesi (0.0°)")
    ax1.set_ylabel("Tilt Acisi [derece]", fontsize=10)
    ax1.set_ylim(-2.0, 26.0)
    ax1.legend(loc="upper right", framealpha=0.92, fontsize=8.5, edgecolor="#cbd5e1")
    ax1.set_title("1. Tilt Servosu Kinematik Super-elevasyon Egrisi", fontsize=10.5, fontweight="bold", pad=6)

    # 2. Volan PWM ve Hiz
    ax2 = axes[1]
    ax2.plot(time_hist, flywheel_pwm_hist, color="#d97706", linewidth=2.5, label="TB6612FNG PWM Devri (0-255)")
    ax2.set_ylabel("Volan PWM [Duty]", fontsize=10)
    ax2.set_ylim(-10, 220)
    ax2.legend(loc="upper right", framealpha=0.92, fontsize=8.5, edgecolor="#cbd5e1")
    ax2.set_title("2. Fircasiz Cift Volan Motoru Hiz Rampasi (4.4 m/s Cikis)", fontsize=10.5, fontweight="bold", pad=6)

    # 3. Solenoid Darbe Sinyali (12V Darbe)
    ax3 = axes[2]
    ax3.step(time_hist, solenoid_hist, where="post", color="#dc2626", linewidth=2.5, label="STM32 Solenoid Surucu GPIO (12V Pulse)")
    ax3.set_ylabel("Solenoid [Mantiksal]", fontsize=10)
    ax3.set_ylim(-0.2, 1.3)
    ax3.set_yticks([0, 1])
    ax3.set_yticklabels(["KAPALI (0V)", "TETIKLE (12V)"])
    ax3.legend(loc="upper right", framealpha=0.92, fontsize=8.5, edgecolor="#cbd5e1")
    ax3.set_title("3. 80 ms Kenar Tetiklemeli Dart Itici Solenoid Darbesi", fontsize=10.5, fontweight="bold", pad=6)

    # 4. Angajman Durum Kodu (FSM State)
    ax4 = axes[3]
    ax4.step(time_hist, state_hist, where="post", color="#7c3aed", linewidth=2.5, label="Ikili Iletisim Komut Durumu")
    ax4.set_ylabel("FCS Protokol Durumu", fontsize=10)
    ax4.set_ylim(-0.4, 2.5)
    ax4.set_yticks([0, 1, 2])
    ax4.set_yticklabels(["TRACKING", "LOCKED", "FIRING"])
    ax4.set_xlabel("Zaman [saniye]", fontsize=10)
    ax4.legend(loc="upper right", framealpha=0.92, fontsize=8.5, edgecolor="#cbd5e1")
    ax4.set_title("4. Gömülü FCS Protokol Durum Degisimi (Binary Command State)", fontsize=10.5, fontweight="bold", pad=6)

    plt.tight_layout()
    out_path = FIG_DIR / "python_fig4_engagement_timing_profile.png"
    plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor())
    plt.close()
    print(f"[FIG 4 OK] Kaydedildi: {out_path}")


if __name__ == "__main__":
    plot_fig4_engagement_profile()
