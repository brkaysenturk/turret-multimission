#!/usr/bin/env python3
"""
================================================================================
MODUL: TURRET BASKET AERODINAMIK RK4 BALISTIK YORUNGE VE TOLERANS ANALIZI
================================================================================
Bu betik; Turret Basket sisteminin 2. Model Balistik gereksinimlerini akademik
seviyede modeller:
  1. python_fig1_ballistic_trajectories_rk4.png:
     - 4. Mertebe Runge-Kutta (RK4) aerodinamik surtunmeli ucus yorungeleri.
     - 1.2m, 1.8m, 2.4m ve 3.0m mesafelerdeki kupalara tam kupa agzi girisi (Basket).
     - Kupa geometrisi (dis govde, kulp, ic agiz kavitesi, taban ve masa seviyesi).
     - Tepe noktalari (apex), inis acisi (derece) ve ucus suresi (ms) metrikleri.
  2. python_fig3_basket_tolerance_heatmap.png:
     - 2.0m nominal mesafede Pitch Aci Hatasi (+/- 1.5 derece) vs Volan Hiz Hatasi
       (+/- 1.0 m/s) faz uzayinda basket basari zarfi (Tolerance Envelope).
     - Kupa ic agiz acikligi (7.6 cm cap) icine dusen basarili basket bolgesi.
================================================================================
"""

import math
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

from turret_basket.rl_ballistics import RLBallistics

# Fiziksel Geometri Sabitleri (Webots CAD v10 ve CONFIG ile birebir uyumlu)
TABLE_REL_Y = -0.3287   # Masa yuzeyi (0.710m - 1.0387m CAD v10 namlu seviyesi)
RIM_REL_Y = -0.2222     # Kupa agiz cemberi (0.8165m - 1.0387m)
BOTTOM_REL_Y = -0.3137  # Kupa ic taban seviyesi (0.725m - 1.0387m)
CUP_OUTER_R = 0.045     # 9.0 cm dis govde yaricapi
CUP_INNER_R = 0.038     # 7.6 cm ic agiz yaricapi
K_DRAG = 0.05           # Webots b=0.05 kati cisim dart lineer sonumleme katsayisi
GRAVITY = 9.80665       # Yercekimi ivmesi (m/s^2)


def simulate_full_trajectory(dist_m: float, v0: float, theta_launch_deg: float, dt: float = 0.0005):
    """Dartin namludan cikip hedefe ve masaya ulasmasina kadar tam RK4 yorungesini kaydeder."""
    theta_rad = math.radians(theta_launch_deg)
    vx = v0 * math.cos(theta_rad)
    vy = v0 * math.sin(theta_rad)
    x, y, t = 0.0, 0.0, 0.0

    xs, ys, times = [0.0], [0.0], [0.0]
    hit_event = "FLIGHT"

    while t < 2.0:
        step_dt = dt
        def deriv(s_vx, s_vy):
            ax = -K_DRAG * s_vx
            ay = -GRAVITY - K_DRAG * s_vy
            return s_vx, s_vy, ax, ay

        vx1, vy1, ax1, ay1 = deriv(vx, vy)
        vx2, vy2, ax2, ay2 = deriv(vx + 0.5 * step_dt * ax1, vy + 0.5 * step_dt * ay1)
        vx3, vy3, ax3, ay3 = deriv(vx + 0.5 * step_dt * ax2, vy + 0.5 * step_dt * ay2)
        vx4, vy4, ax4, ay4 = deriv(vx + step_dt * ax3, vy + step_dt * ay3)

        x += (step_dt / 6.0) * (vx1 + 2.0 * vx2 + 2.0 * vx3 + vx4)
        y += (step_dt / 6.0) * (vy1 + 2.0 * vy2 + 2.0 * vy3 + vy4)
        vx += (step_dt / 6.0) * (ax1 + 2.0 * ax2 + 2.0 * ax3 + ax4)
        vy += (step_dt / 6.0) * (ay1 + 2.0 * ay2 + 2.0 * ay3 + ay4)
        t += step_dt

        xs.append(x)
        ys.append(y)
        times.append(t)

        # 1. Kupa Agiz Duzleminden Ic Kaviteye Giris (BASKET: inis kolunda agiz duzlemini keserken)
        if vy < 0.0 and y <= (RIM_REL_Y - 0.005):
            if (dist_m - CUP_INNER_R) <= x <= (dist_m + CUP_INNER_R):
                hit_event = "BASKET"
                break
            elif (dist_m - CUP_OUTER_R) <= x < (dist_m - CUP_INNER_R):
                hit_event = "FRONT_WALL"
                break

        # 2. Kupa On Duvar Carpismasi (agiz duzleminden once dis govdeye carpma)
        if (dist_m - CUP_OUTER_R) <= x < (dist_m - CUP_INNER_R):
            if y < RIM_REL_Y:
                hit_event = "FRONT_WALL"
                break

        # 3. Masa Seviyesi veya Kupa Agiz Duzlemi Gecisi
        if vy < 0.0 and y <= (RIM_REL_Y - 0.005):
            if x < (dist_m - CUP_OUTER_R):
                hit_event = "TABLE_SHORT"
            else:
                hit_event = "OVERSHOOT"
            break

    entry_angle_deg = math.degrees(math.atan2(vy, vx))
    speed_end = math.hypot(vx, vy)
    return np.array(xs), np.array(ys), np.array(times), hit_event, entry_angle_deg, speed_end


def draw_cup(ax, cup_x: float, scale_note: bool = False):
    """Masa uzerindeki kulplu kupanin kesitini cizer."""
    cup_h = RIM_REL_Y - TABLE_REL_Y
    # 1. Seramik Dis Govde
    body = patches.Rectangle(
        (cup_x - CUP_OUTER_R, TABLE_REL_Y),
        CUP_OUTER_R * 2,
        cup_h,
        facecolor="#cbd5e1",
        edgecolor="#334155",
        linewidth=1.4,
        zorder=3
    )
    ax.add_patch(body)

    # 2. Ic Agiz Kavitesi (Dartin girecegi bosluk)
    cavity = patches.Rectangle(
        (cup_x - CUP_INNER_R, BOTTOM_REL_Y),
        CUP_INNER_R * 2,
        (RIM_REL_Y - BOTTOM_REL_Y),
        facecolor="#bae6fd",
        edgecolor="#0284c7",
        linewidth=1.2,
        linestyle="-",
        zorder=4
    )
    ax.add_patch(cavity)

    # 3. Kupa Kulpu (Sag Taraf)
    handle_x = cup_x + CUP_OUTER_R
    handle_y = TABLE_REL_Y + cup_h * 0.25
    handle_h = cup_h * 0.50
    handle = patches.Arc(
        (handle_x, handle_y + handle_h / 2.0),
        0.035,
        handle_h,
        angle=0,
        theta1=-90,
        theta2=90,
        color="#475569",
        linewidth=2.0,
        zorder=2
    )
    ax.add_patch(handle)

    # Kupa Agiz Seviyesi Cizgisi
    ax.plot([cup_x - CUP_OUTER_R, cup_x + CUP_OUTER_R], [RIM_REL_Y, RIM_REL_Y],
            color="#0369a1", linewidth=1.8, zorder=5)

    if scale_note:
        ax.text(cup_x, RIM_REL_Y + 0.025, f"Kupa\n{cup_x:.1f}m", ha="center", va="bottom",
                fontsize=8.5, fontweight="bold", color="#0f172a", zorder=6)


def plot_fig1_trajectories():
    """Figure 1: RK4 Balistik Ucus Yorungeleri ve Kupa Agzi Giris Dinamigi."""
    fig = plt.figure(figsize=(15, 11), facecolor="#ffffff")
    gs = fig.add_gridspec(2, 2, height_ratios=[1.4, 1.0], hspace=0.30, wspace=0.22)

    ai = RLBallistics()

    raw_cases = [
        {"dist": 1.2, "col": "#2563eb"},
        {"dist": 1.8, "col": "#059669"},
        {"dist": 2.4, "col": "#d97706"},
        {"dist": 3.0, "col": "#dc2626"},
    ]
    test_cases = []
    for rc in raw_cases:
        d_val = rc["dist"]
        lp_val = ai.compute_launch_parameters(d_val, RIM_REL_Y)
        _, _, ts_val, _, _, _ = simulate_full_trajectory(d_val, lp_val.speed_mps, lp_val.tilt_offset_deg)
        tof_ms = int(round(ts_val[-1] * 1000.0))
        test_cases.append({
            "dist": d_val,
            "v0": lp_val.speed_mps,
            "theta": lp_val.tilt_offset_deg,
            "col": rc["col"],
            "label": f"Hedef {d_val:.1f}m (v0={lp_val.speed_mps:.2f} m/s, pitch={lp_val.tilt_offset_deg:.1f}°, ToF={tof_ms} ms)"
        })

    # --- Panel 1 (Ust Genis): Yorunge Profilleri ve Kupa Masasi ---
    ax_main = fig.add_subplot(gs[0, :])
    ax_main.set_facecolor("#fcfcfc")
    ax_main.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    # Masa Yuzeyi ve Namlu Seviyesi
    ax_main.axhline(0.0, color="#64748b", linestyle=":", linewidth=1.2, label="Namlu Referans Kotu (y = 0.0 m)")
    ax_main.axhline(TABLE_REL_Y, color="#78350f", linestyle="-", linewidth=3.0, alpha=0.8,
                    label="Ahsap Masa Yuzeyi (y = -0.329 m)")

    # Taret Temsili (x=0, y=0)
    turret_box = patches.Rectangle((-0.08, TABLE_REL_Y), 0.08, abs(TABLE_REL_Y),
                                   facecolor="#475569", edgecolor="#1e293b", linewidth=1.5, zorder=3)
    ax_main.add_patch(turret_box)
    ax_main.plot([0, 0.08], [0, 0.032], color="#0f172a", linewidth=4.0, zorder=4)  # Namlu
    ax_main.text(-0.04, TABLE_REL_Y + 0.05, "Pan-Tilt\nTaret", ha="center", fontsize=8.5, fontweight="bold", color="white")

    entry_angles = []
    flight_times = []
    distances = []

    for tc in test_cases:
        d = tc["dist"]
        v0 = tc["v0"]
        theta = tc["theta"]
        col = tc["col"]
        lbl = tc["label"]

        xs, ys, ts, event, angle_deg, v_end = simulate_full_trajectory(d, v0, theta)

        distances.append(d)
        entry_angles.append(abs(angle_deg))
        flight_times.append(ts[-1] * 1000.0)

        # Yorungeyi ciz
        ax_main.plot(xs, ys, color=col, linewidth=2.2, label=lbl, zorder=5)

        # Tepe Noktasi (Apex)
        apex_idx = int(np.argmax(ys))
        ax_main.plot(xs[apex_idx], ys[apex_idx], marker="^", color=col, markersize=8, zorder=6)
        ax_main.text(xs[apex_idx], ys[apex_idx] + 0.018, f"+{ys[apex_idx]:.2f}m",
                     ha="center", fontsize=8.5, color=col, fontweight="bold", zorder=7)

        # Kupa ve Basket Giris Noktasi
        draw_cup(ax_main, d, scale_note=True)
        # Basket Basari Isareti (Kupanin tam icine inis)
        ax_main.plot(xs[-1], ys[-1], marker="*", color="#facc15", markersize=14,
                     markeredgecolor="#854d0e", markeredgewidth=1.2, zorder=8)

    ax_main.set_title("Turret Basket: Aerodinamik RK4 Balistik Yorungeleri ve Kupa Agzi Giris Dinamigi\n"
                      "4. Mertebe Runge-Kutta, Hava Surtunmesi (k_drag=0.05) ve Model 2 RL Politika Kararlari",
                      fontsize=12.5, fontweight="bold", pad=12, color="#0f172a")
    ax_main.set_xlabel("Yatay Mesafe x [metre]", fontsize=10.5, labelpad=6)
    ax_main.set_ylabel("Dikey Konum y [metre]", fontsize=10.5, labelpad=6)
    ax_main.set_xlim(-0.15, 3.25)
    ax_main.set_ylim(-0.38, 0.35)
    ax_main.legend(loc="upper right", framealpha=0.94, edgecolor="#cbd5e1", fontsize=8.8)

    # --- Panel 2 (Sol Alt): Kupa Agzina Inis / Diklik Acisi ---
    ax_sub1 = fig.add_subplot(gs[1, 0])
    ax_sub1.set_facecolor("#fcfcfc")
    ax_sub1.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    # Surekli mesafe taramasi
    fine_dists = np.linspace(1.0, 3.1, 30)
    fine_angles = []
    fine_times = []
    fine_speeds = []

    for fd in fine_dists:
        lp_f = ai.compute_launch_parameters(fd, -0.2287)
        _, _, ts_c, _, ang_c, v_c = simulate_full_trajectory(fd, lp_f.speed_mps, lp_f.tilt_offset_deg)
        fine_angles.append(abs(ang_c))
        fine_times.append(ts_c[-1] * 1000.0)
        fine_speeds.append(v_c)

    ax_sub1.plot(fine_dists, fine_angles, color="#7c3aed", linewidth=2.4, marker="o", markersize=4,
                 label="Kupa Agzina Inis Acisi")
    ax_sub1.axhline(30.0, color="#16a34a", linestyle="--", linewidth=1.4,
                    label="Guvenli Dik Giris Esigi (>= 30°)")
    ax_sub1.fill_between(fine_dists, 30.0, 50.0, color="#bbf7d0", alpha=0.25, label="Optimal Basket Zonu")
    ax_sub1.set_title("Kupa Agzina Inis Diklik Acisi vs Mesafe", fontsize=11, fontweight="bold", color="#0f172a", pad=8)
    ax_sub1.set_xlabel("Kupa Mesafesi [metre]", fontsize=9.5)
    ax_sub1.set_ylabel("Inis Acisi [derece]", fontsize=9.5)
    ax_sub1.set_ylim(20, 48)
    ax_sub1.legend(loc="lower right", fontsize=8.5, edgecolor="#cbd5e1", framealpha=0.92)

    # --- Panel 3 (Sag Alt): Ucus Suresi ve Darbe Kinetigi ---
    ax_sub2 = fig.add_subplot(gs[1, 1])
    ax_sub2.set_facecolor("#fcfcfc")
    ax_sub2.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    ax_sub2.plot(fine_dists, fine_times, color="#0284c7", linewidth=2.4, marker="s", markersize=4,
                 label="Ucus Suresi (ToF, ms)")
    ax_sub2_twin = ax_sub2.twinx()
    ax_sub2_twin.plot(fine_dists, fine_speeds, color="#ea580c", linewidth=2.0, linestyle="--",
                      label="Inis Hizi (m/s)")
    ax_sub2_twin.set_ylabel("Dart Inis Hizi [m/s]", fontsize=9.5, color="#ea580c")
    ax_sub2_twin.tick_params(colors="#ea580c")

    ax_sub2.set_title("Ucus Suresi ve Inis Hizi vs Mesafe", fontsize=11, fontweight="bold", color="#0f172a", pad=8)
    ax_sub2.set_xlabel("Kupa Mesafesi [metre]", fontsize=9.5)
    ax_sub2.set_ylabel("Ucus Suresi [milisaniye]", fontsize=9.5, color="#0284c7")
    ax_sub2.tick_params(colors="#0284c7")
    ax_sub2.set_ylim(200, 650)

    # Birlestirilmis lejant
    lines1, labels1 = ax_sub2.get_legend_handles_labels()
    lines2, labels2 = ax_sub2_twin.get_legend_handles_labels()
    ax_sub2.legend(lines1 + lines2, labels1 + labels2, loc="upper left", fontsize=8.5, edgecolor="#cbd5e1", framealpha=0.92)

    out_path = FIG_DIR / "python_fig1_ballistic_trajectories_rk4.png"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[FIG 1 OK] Yeniden olusturuldu: {out_path}")


def plot_fig3_tolerance_heatmap():
    """Figure 3: Balistik Hata Tolerans Zarflari ve Basket Basari Haritasi (2.0m Nominal)."""
    fig, (ax_heat, ax_scatter) = plt.subplots(1, 2, figsize=(15, 6.5))
    fig.patch.set_facecolor("#ffffff")

    d_nom = 2.0
    ai = RLBallistics()
    lp_nom = ai.compute_launch_parameters(d_nom, RIM_REL_Y)
    v_nom = lp_nom.speed_mps
    th_nom = lp_nom.tilt_offset_deg

    n_pts = 35
    d_thetas = np.linspace(-1.4, +1.4, n_pts)
    d_vels = np.linspace(-0.8, +0.8, n_pts)

    miss_dist_grid = np.zeros((n_pts, n_pts))
    hit_class_grid = np.zeros((n_pts, n_pts))  # 0: Basket, 1: Front Wall, 2: Short, 3: Long

    for i, dv in enumerate(d_vels):
        for j, dth in enumerate(d_thetas):
            v_curr = v_nom + dv
            th_curr = th_nom + dth
            xs, ys, _, event, _, _ = simulate_full_trajectory(d_nom, v_curr, th_curr)
            last_x = xs[-1]
            miss_mm = (last_x - d_nom) * 1000.0
            miss_dist_grid[i, j] = miss_mm

            if event == "BASKET":
                hit_class_grid[i, j] = 0
            elif event == "FRONT_WALL":
                hit_class_grid[i, j] = 1
            elif event == "TABLE_SHORT":
                hit_class_grid[i, j] = 2
            else:
                hit_class_grid[i, j] = 3

    # Isı Haritası (Miss Distance in mm)
    ax_heat.set_facecolor("#fcfcfc")
    cmap = plt.cm.coolwarm
    extent = [d_thetas[0], d_thetas[-1], d_vels[0], d_vels[-1]]
    im = ax_heat.imshow(miss_dist_grid, extent=extent, origin="lower", cmap=cmap, aspect="auto", vmin=-120, vmax=120)
    cbar = plt.colorbar(im, ax=ax_heat)
    cbar.set_label("Hedef Merkezinden Sapma [mm]", fontsize=9.5)

    # Basket Konturu (Agiz Cemberi Siniri: +/- 38 mm)
    cs = ax_heat.contour(d_thetas, d_vels, miss_dist_grid, levels=[-38.0, 38.0],
                         colors=["#15803d", "#15803d"], linewidths=2.5, linestyles="--")
    ax_heat.clabel(cs, inline=True, fmt="%1.0f mm", fontsize=8.5)

    ax_heat.plot(0, 0, marker="*", color="#facc15", markersize=14, markeredgecolor="#854d0e",
                 label=f"Optimal Karar (v0={v_nom:.2f} m/s, pitch={th_nom:.1f}°)")
    ax_heat.set_title("Turret Basket: 2.0m Mesafe Inis Sapmasi Kontur Haritasi", fontsize=11.5, fontweight="bold", pad=10)
    ax_heat.set_xlabel("Pitch Aci Hatasi [derece]", fontsize=10)
    ax_heat.set_ylabel("Volan Hiz Hatasi [m/s]", fontsize=10)
    ax_heat.legend(loc="upper left", fontsize=8.5, framealpha=0.9)
    ax_heat.grid(True, linestyle=":", alpha=0.4)

    # Bolge Siniflandirma Grafigi (Scatter / Region Map)
    ax_scatter.set_facecolor("#fcfcfc")
    region_colors = {
        0: ("#16a34a", "BASARILI BASKET (Kupa Ici)"),
        1: ("#dc2626", "On Govde Duvari Carpismasi"),
        2: ("#ea580c", "Masa Yuzeyi Kisa Atis"),
        3: ("#7c3aed", "Kupa Arkasi Asiri Atis"),
    }

    mesh_th, mesh_v = np.meshgrid(d_thetas, d_vels)
    for c_id, (color, lbl) in region_colors.items():
        mask = (hit_class_grid == c_id)
        if np.any(mask):
            ax_scatter.scatter(mesh_th[mask], mesh_v[mask], color=color, s=28, alpha=0.82, label=lbl)

    ax_scatter.plot(0, 0, marker="*", color="#facc15", markersize=14, markeredgecolor="#854d0e")
    ax_scatter.set_title("Basket Tolerans Zarfi (Cup Tolerance Envelope)", fontsize=11.5, fontweight="bold", pad=10)
    ax_scatter.set_xlabel("Pitch Aci Hatasi [derece]", fontsize=10)
    ax_scatter.set_ylabel("Volan Hiz Hatasi [m/s]", fontsize=10)
    ax_scatter.legend(loc="upper left", fontsize=8.5, framealpha=0.92, edgecolor="#cbd5e1")
    ax_scatter.grid(True, linestyle="--", alpha=0.5, color="#cbd5e1")

    plt.tight_layout()
    out_path = FIG_DIR / "python_fig3_basket_tolerance_heatmap.png"
    plt.savefig(out_path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"[FIG 3 OK] Yeniden olusturuldu: {out_path}")


if __name__ == "__main__":
    plot_fig1_trajectories()
    plot_fig3_tolerance_heatmap()
