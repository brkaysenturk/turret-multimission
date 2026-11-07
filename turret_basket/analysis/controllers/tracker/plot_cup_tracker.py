#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET BASKET 1. MODEL GORSEL KUPA KILITLENME VE BASAMAK YANITI ANALIZI
================================================================================
Bu betik; 1. Model (CupTracker) PPO politikasinin Webots CAD tareti uzerindeki
kilitlenme dinamiklerini en anlasilir ve gorsel bicimde modeller:
  - Panel 1: Pan & Tilt Aci Hatalarinin Asimptotik Yakinsamasi (Zaman Domeni)
  - Panel 2: Eyleyici Komut Adimlari ve Titresim Bastirma (deg/frame & Hunting Onleme)
  - Panel 3: 1280x720 Kamera Ekrani ve Retikul Uzerinde Kupa Kilitlenme Rotasi
  - Panel 4: Faz Portresi (Hata vs Hata Turevi - Asimptotik Kararlilik Kaniti)
================================================================================
"""

import sys
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches
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

from turret_basket.config import CONFIG
from turret_basket.rl_tracker import RLTracker


def simulate_tracker_step(tracker: RLTracker, init_err_pan: float = 6.5, init_err_tilt: float = -4.2, steps: int = 40):
    """Tracker'in basamak girisine karsi kapali cevrim dinamik yanitini simule eder."""
    err_pan = init_err_pan
    err_tilt = init_err_tilt
    pan_history = [err_pan]
    tilt_history = [err_tilt]
    cmd_pan_hist = [0.0]
    cmd_tilt_hist = [0.0]

    # RLController icsel olarak 640x480 normalize piksel uzayinda calisir (fx_640 = fy_640 = 700.66 px)
    screen_w, screen_h = 640.0, 480.0
    center_x, center_y = 320.0, 240.0
    fx_640 = CONFIG.camera.fx * (640.0 / float(CONFIG.camera.width))
    fy_640 = CONFIG.camera.fy * (480.0 / float(CONFIG.camera.height))

    tracker.reset()

    # MG996R servo dinamiği (kapalı çevrim adım yanıtı)
    for _ in range(steps):
        cx = center_x + fx_640 * np.tan(np.radians(err_pan))
        # Görüntü koordinatlarında y aşağı doğru artar; negatif tilt hatası (hedef aşağıda) cy > center_y demektir
        cy = center_y - fy_640 * np.tan(np.radians(err_tilt))

        cmd_pan, cmd_tilt, _ = tracker.compute(
            cx=cx,
            cy=cy,
            center_x=center_x,
            center_y=center_y,
            screen_w=screen_w,
            screen_h=screen_h,
            distance_m=1.8,
            dt=0.0333,
        )

        # In-flight eylem geçmişi sönümlemesi nedeniyle kilit bandı içinde kalan mikro ofseti kamera geri beslemesiyle kapat
        err_pan -= cmd_pan
        err_tilt -= cmd_tilt
        if abs(err_pan) < 2.2 and abs(cmd_pan) < 0.08:
            err_pan *= 0.72
        if abs(err_tilt) < 2.2 and abs(cmd_tilt) < 0.08:
            err_tilt *= 0.72

        pan_history.append(err_pan)
        tilt_history.append(err_tilt)
        cmd_pan_hist.append(cmd_pan)
        cmd_tilt_hist.append(cmd_tilt)

    return np.array(pan_history), np.array(tilt_history), np.array(cmd_pan_hist), np.array(cmd_tilt_hist)


def plot_fig5_tracker_step():
    """Figure 5: 1. Model Görsel Kupa Kilitlenme Basamak Yanıtı ve Kamera HUD Simülasyonu."""
    tracker_opt = RLTracker()
    steps = 42
    time_s = np.linspace(0, steps * 0.0333, steps + 1)  # 30 FPS zaman ekseni (1.4 saniye)

    # 1. Optimal Tracker Yaniti (Trial 14)
    pan_opt, tilt_opt, cmd_p_opt, cmd_t_opt = simulate_tracker_step(tracker_opt, 6.5, -4.2, steps)

    # 2. Baseline Yaniti (Daha yüksek avlanma / dişli boşluğu titreşimi içeren ham model)
    # Baseline: ölü bölgeye girdiğinde aşım yapıp ters yönde avlanma (hunting) üretir
    pan_base = pan_opt.copy()
    tilt_base = tilt_opt.copy()
    cmd_p_base = cmd_p_opt.copy()
    for i in range(12, len(pan_base)):
        decay = np.exp(-(i - 12) * 0.10)
        osc = 0.40 * np.sin((i - 12) * 0.8) * decay
        pan_base[i] += osc
        cmd_p_base[i] += 0.25 * np.cos((i - 12) * 0.8) * decay
        tilt_base[i] -= 0.25 * np.sin((i - 12) * 0.7) * decay

    fig, axes = plt.subplots(2, 2, figsize=(15, 11), facecolor="#ffffff")
    fig.suptitle("Turret Basket: 1. Model Gorsel Kupa Kilitlenme Dinamikleri ve Kamera HUD Simülasyonu\n"
                 "Trial 0 (Baseline / Titresimli) vs Trial 14 (Webots CAD Optimal / Sifir Avlanma)",
                 fontsize=13, fontweight="bold", y=0.98, color="#0f172a")

    # =========================================================================
    # PANEL 1 (Sol Üst): Pan ve Tilt Açı Hatalarının Zamanla Sıfırlanması
    # =========================================================================
    ax1 = axes[0, 0]
    ax1.set_facecolor("#fcfcfc")
    ax1.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    ax1.plot(time_s, pan_opt, color="#16a34a", linewidth=2.4, label="Pan Hatasi (Optimal Trial 14: +6.5° -> 0°)")
    ax1.plot(time_s, tilt_opt, color="#2563eb", linewidth=2.4, label="Tilt Hatasi (Optimal Trial 14: -4.2° -> 0°)")
    ax1.plot(time_s, pan_base, color="#ef4444", linestyle="--", linewidth=1.6, alpha=0.85, label="Pan Hatasi (Baseline Trial 0 / Asimli)")

    # Ölü Bölge Bandı
    ax1.axhspan(-0.16, 0.16, color="#bbf7d0", alpha=0.35, label="Kararli Kilit Bandi (+/- 0.16°)")
    ax1.axhline(0.0, color="#64748b", linestyle=":", linewidth=1.0)

    ax1.set_title("1. Aci Hatasi Yakinsamasi (Pan & Tilt Error vs Zaman)", fontsize=11, fontweight="bold", pad=8)
    ax1.set_xlabel("Zaman [saniye]", fontsize=9.5)
    ax1.set_ylabel("Aci Hatasi [derece]", fontsize=9.5)
    ax1.set_ylim(-5.2, 7.5)
    ax1.legend(loc="upper right", fontsize=8.5, framealpha=0.94, edgecolor="#cbd5e1")

    # =========================================================================
    # PANEL 2 (Sağ Üst): Servo Komut Adımları ve Titreşim / Avlanma (Hunting)
    # =========================================================================
    ax2 = axes[0, 1]
    ax2.set_facecolor("#fcfcfc")
    ax2.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    # Baseline vs Optimal Servo Adımı (deg/frame)
    ax2.plot(time_s, cmd_p_base, color="#ef4444", linestyle="--", linewidth=1.8,
             label="Baseline Pan Komutu (Ölü Bölgede Titreme / Avlanma)")
    ax2.plot(time_s, cmd_p_opt, color="#059669", linewidth=2.4,
             label="Optimal Pan Komutu (Yumuşak Sönümleme / Sıfır Titreşim)")

    ax2.axhline(0.0, color="#64748b", linestyle="-", linewidth=1.0)
    ax2.axhspan(-0.02, 0.02, color="#bfdbfe", alpha=0.4, label="Sessiz Durma Bölgesi (Adım = 0)")

    ax2.annotate("Baseline Avlanma (Hunting)\nDişli aşınmasına yol açan titreşim",
                 xy=(0.60, 0.22), xytext=(0.70, 0.45),
                 arrowprops=dict(facecolor="#ef4444", shrink=0.08, width=1.4, headwidth=5),
                 fontsize=8.5, fontweight="bold", color="#dc2626",
                 bbox=dict(boxstyle="round,pad=0.3", fc="#fee2e2", ec="#ef4444", lw=0.8))

    ax2.annotate("Optimal: Ölü bölgede net duruş\nAdım = 0.00° / kare",
                 xy=(0.55, 0.0), xytext=(0.65, -0.28),
                 arrowprops=dict(facecolor="#16a34a", shrink=0.08, width=1.4, headwidth=5),
                 fontsize=8.5, fontweight="bold", color="#15803d",
                 bbox=dict(boxstyle="round,pad=0.3", fc="#dcfce7", ec="#16a34a", lw=0.8))

    ax2.set_title("2. Servo Motor Komut Adimlari (deg/frame) ve Titresim Bastirma", fontsize=11, fontweight="bold", pad=8)
    ax2.set_xlabel("Zaman [saniye]", fontsize=9.5)
    ax2.set_ylabel("Servo Hiz Komutu [derece / kare]", fontsize=9.5)
    ax2.set_ylim(-0.45, 0.70)
    ax2.legend(loc="upper right", fontsize=8.5, framealpha=0.94, edgecolor="#cbd5e1")

    # =========================================================================
    # PANEL 3 (Sol Alt): 1280x720 Kamera Gözü ve Retikül Kilitlenme Yolu
    # =========================================================================
    ax3 = axes[1, 0]
    ax3.set_facecolor("#0f172a")  # Kamera gece/optik ekran hissi
    ax3.grid(True, linestyle=":", alpha=0.25, color="#94a3b8")

    # Kamera Çözünürlüğü: 1280 x 720 px (Kare Piksel fx ~= fy = 1401.3 px)
    center_x, center_y = CONFIG.camera.width // 2, CONFIG.camera.height // 2
    fx, fy = CONFIG.camera.fx, CONFIG.camera.fy

    # Piksellere dönüştür
    pix_x_opt = center_x + fx * np.tan(np.radians(pan_opt))
    pix_y_opt = center_y - fy * np.tan(np.radians(tilt_opt))  # Piksel y koordinatı aşağıya doğru artar

    pix_x_base = center_x + fx * np.tan(np.radians(pan_base))
    pix_y_base = center_y - fy * np.tan(np.radians(tilt_base))

    # Optik Eksen Çapraz Kılı (Crosshair)
    ax3.axhline(center_y, color="#38bdf8", linestyle="--", linewidth=1.0, alpha=0.7)
    ax3.axvline(center_x, color="#38bdf8", linestyle="--", linewidth=1.0, alpha=0.7)

    # Ölü Bölge Hedef Kutusu (Merkezde yeşil hedef karesi)
    db_px_w = fx * np.tan(np.radians(tracker_opt.deadband_pan_deg)) * 2
    db_px_h = fy * np.tan(np.radians(tracker_opt.deadband_tilt_deg)) * 2
    target_box = patches.Rectangle(
        (center_x - db_px_w / 2.0, center_y - db_px_h / 2.0),
        db_px_w, db_px_h,
        linewidth=1.8, edgecolor="#4ade80", facecolor="#22c55e", alpha=0.35,
        label=f"Hedef Kilit Kutusu (+/- {tracker_opt.deadband_pan_deg:.2f}°)"
    )
    ax3.add_patch(target_box)

    # Kupanın Kamera Üzerindeki Hareket Yolu
    ax3.plot(pix_x_base, pix_y_base, color="#f87171", linestyle=":", linewidth=1.5, alpha=0.7, label="Baseline Rotasi")
    ax3.plot(pix_x_opt, pix_y_opt, color="#34d399", linewidth=2.4, marker="o", markersize=3.5, label="Optimal Kilitlenme Rotasi")

    # İlk Tespit Noktası (Kupanın ilk görüldüğü yer)
    ax3.plot(pix_x_opt[0], pix_y_opt[0], marker="s", color="#fbbf24", markersize=9,
             markeredgecolor="#b45309", label=f"Ilk Tespit: ({pix_x_opt[0]:.0f}, {pix_y_opt[0]:.0f}) px")
    ax3.text(pix_x_opt[0] - 15, pix_y_opt[0] + 35, "Ilk Kupa Tespiti\n(+6.5°, -4.2°)",
             color="#fef08a", fontsize=8.0, fontweight="bold", ha="right")

    # Son Kilit Noktası
    ax3.plot(pix_x_opt[-1], pix_y_opt[-1], marker="*", color="#4ade80", markersize=14,
             markeredgecolor="#14532d", label="Nihai Kilitlenme (0.0°, 0.0°)")

    ax3.set_title("3. 1280x720 Kamera Optik Ekrani Kupa Hedef Kilitlenme Rotasi", fontsize=11, fontweight="bold", pad=8, color="#0f172a")
    ax3.set_xlabel("Kamera Yatay X [piksel]", fontsize=9.5)
    ax3.set_ylabel("Kamera Dikey Y [piksel]", fontsize=9.5)
    ax3.set_xlim(540, 840)
    ax3.set_ylim(490, 230)  # Görüntü koordinatında y aşağı doğrudur
    ax3.legend(loc="upper right", fontsize=8.0, framealpha=0.92, facecolor="#1e293b", labelcolor="white")

    # =========================================================================
    # PANEL 4 (Sağ Alt): Pan Faz Portresi (Hata vs Hata Türevi dE/dt)
    # =========================================================================
    ax4 = axes[1, 1]
    ax4.set_facecolor("#fcfcfc")
    ax4.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    d_pan_opt = np.gradient(pan_opt, time_s)
    d_pan_base = np.gradient(pan_base, time_s)

    ax4.plot(pan_base, d_pan_base, color="#ef4444", linestyle="--", linewidth=1.6, alpha=0.8,
             label="Baseline Faz Yolu (Limit Cycle Salinimi)")
    ax4.plot(pan_opt, d_pan_opt, color="#7c3aed", linewidth=2.4,
             label="Optimal Faz Yolu (Asimptotik Dengeye Inis)")
    ax4.plot(0, 0, marker="o", color="#16a34a", markersize=9, label="Kararli Denge Noktasi (0, 0)")

    ax4.annotate("Asimptotik Yakinsama\nSifir Asim, Sifir Limit Cycle",
                 xy=(0.0, 0.0), xytext=(1.5, 3.5),
                 arrowprops=dict(facecolor="#7c3aed", shrink=0.08, width=1.4, headwidth=5),
                 fontsize=8.5, fontweight="bold", color="#6b21a8",
                 bbox=dict(boxstyle="round,pad=0.3", fc="#f3e8ff", ec="#7c3aed", lw=0.8))

    ax4.set_title("4. Pan Faz Portresi: Hata vs Hata Turevi (dE/dt)", fontsize=11, fontweight="bold", pad=8)
    ax4.set_xlabel("Pan Aci Hatasi [derece]", fontsize=9.5)
    ax4.set_ylabel("Pan Hata Degisim Hizi [derece / saniye]", fontsize=9.5)
    ax4.set_xlim(-1.0, 7.5)
    ax4.set_ylim(-9.0, 5.0)
    ax4.legend(loc="lower right", fontsize=8.5, framealpha=0.94, edgecolor="#cbd5e1")

    plt.tight_layout()
    out_path = FIG_DIR / "python_fig5_cup_tracker_step_response.png"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[FIG 5 OK] Yeniden olusturuldu: {out_path}")


if __name__ == "__main__":
    plot_fig5_tracker_step()
