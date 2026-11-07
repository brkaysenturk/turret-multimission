import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT_IMG = ROOT / "turret_fcs" / "analysis" / "comparison" / "trial_12_vs_24_rl_comparison.png"
JSON_PATH = ROOT / "turret_fcs" / "analysis" / "comparison" / "webots_multiprofile_rl_results.json"

data = json.loads(JSON_PATH.read_text(encoding="utf-8"))
base_m = data.get("baseline_metrics", {})
best_m = data.get("best_metrics", {})
base_cost = data.get("baseline_cost", 184.83)
best_cost = data.get("best_cost", 139.00)
best_trial = data.get("best_trial", 12)

distances = ["1.0 m", "2.0 m", "3.0 m"]
x = np.arange(len(distances))
width = 0.35

plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
fig, axes = plt.subplots(3, 2, figsize=(15, 12))
fig.suptitle(
    f"Turret FCS - RL Kontrolcu: Baslangic (Baseline) vs. 24-Trial Optuna Optimal Karsilastirmasi\n"
    f"Baseline (Cost={base_cost:.2f}) vs. Optuna T{best_trial} (Cost={best_cost:.2f}, -%{((base_cost-best_cost)/base_cost)*100:.1f} Kazanc)",
    fontsize=13,
    fontweight="bold"
)

# 1. Kose Giris Asimi (Overshoot deg) - Profil 8
ax = axes[0, 0]
over_base = [base_m.get(f"p8_{d}", {}).get("overshoot_deg", 0.0) for d in ["100cm", "200cm", "300cm"]]
over_best = [best_m.get(f"p8_{d}", {}).get("overshoot_deg", 0.0) for d in ["100cm", "200cm", "300cm"]]
ax.bar(x - width/2, over_base, width, label="Baseline", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, over_best, width, label="Optuna 24-Trial (RL)", color="#27ae60", alpha=0.85)
ax.set_ylabel("Asim (derece)", fontweight="bold")
ax.set_title("Profil 8: Kose Giris Ilk Kilit Asimi (Overshoot) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, over_base[i] + 0.08, f"{over_base[i]:.2f}°", ha="center", fontsize=9)
    ax.text(x[i] + width/2, over_best[i] + 0.08, f"{over_best[i]:.2f}°", ha="center", fontsize=9, fontweight="bold")

# 2. Standart Sinus RMS Hatasi - Profil 2
ax = axes[0, 1]
sin_rms_base = [base_m.get(f"p2_{d}", {}).get("rms_deg", 0.0) for d in ["100cm", "200cm", "300cm"]]
sin_rms_best = [best_m.get(f"p2_{d}", {}).get("rms_deg", 0.0) for d in ["100cm", "200cm", "300cm"]]
ax.bar(x - width/2, sin_rms_base, width, label="Baseline", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, sin_rms_best, width, label="Optuna 24-Trial (RL)", color="#2980b9", alpha=0.85)
ax.set_ylabel("Takip Hatasi RMS (derece)", fontweight="bold")
ax.set_title("Profil 2: Standart Sinus Takip Hatasi (RMS) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, sin_rms_base[i] + 0.08, f"{sin_rms_base[i]:.2f}°", ha="center", fontsize=9)
    ax.text(x[i] + width/2, sin_rms_best[i] + 0.08, f"{sin_rms_best[i]:.2f}°", ha="center", fontsize=9, fontweight="bold")

# 3. Figure-8 Dinamik Takip RMS Hatasi - Profil 7
ax = axes[1, 0]
fig8_rms_base = [base_m.get(f"p7_{d}", {}).get("rms_deg", 0.0) for d in ["100cm", "200cm", "300cm"]]
fig8_rms_best = [best_m.get(f"p7_{d}", {}).get("rms_deg", 0.0) for d in ["100cm", "200cm", "300cm"]]
ax.bar(x - width/2, fig8_rms_base, width, label="Baseline", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, fig8_rms_best, width, label="Optuna 24-Trial (RL)", color="#8e44ad", alpha=0.85)
ax.set_ylabel("Figure-8 RMS (derece)", fontweight="bold")
ax.set_title("Profil 7: Figure-8 Dinamik Takip Hatasi (RMS) [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, fig8_rms_base[i] + 0.08, f"{fig8_rms_base[i]:.2f}°", ha="center", fontsize=9)
    ax.text(x[i] + width/2, fig8_rms_best[i] + 0.08, f"{fig8_rms_best[i]:.2f}°", ha="center", fontsize=9, fontweight="bold")

# 4. Standart Sinus Kilit Orani (%) - Profil 2
ax = axes[1, 1]
sin_lock_base = [base_m.get(f"p2_{d}", {}).get("lock_pct", 0.0) for d in ["100cm", "200cm", "300cm"]]
sin_lock_best = [best_m.get(f"p2_{d}", {}).get("lock_pct", 0.0) for d in ["100cm", "200cm", "300cm"]]
ax.bar(x - width/2, sin_lock_base, width, label="Baseline", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, sin_lock_best, width, label="Optuna 24-Trial (RL)", color="#27ae60", alpha=0.85)
ax.set_ylabel("Kilit Orani (%)", fontweight="bold")
ax.set_title("Profil 2: Standart Sinus Kilitlenme Orani (Lock %) [Yuksek = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, sin_lock_base[i] + 1.0, f"%{sin_lock_base[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, sin_lock_best[i] + 1.0, f"%{sin_lock_best[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

# 5. Profil 8 Kilit Orani (%) - Kose Giris
ax = axes[2, 0]
p8_lock_base = [base_m.get(f"p8_{d}", {}).get("lock_pct", 0.0) for d in ["100cm", "200cm", "300cm"]]
p8_lock_best = [best_m.get(f"p8_{d}", {}).get("lock_pct", 0.0) for d in ["100cm", "200cm", "300cm"]]
ax.bar(x - width/2, p8_lock_base, width, label="Baseline", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, p8_lock_best, width, label="Optuna 24-Trial (RL)", color="#16a085", alpha=0.85)
ax.set_ylabel("Kilit Orani (%)", fontweight="bold")
ax.set_title("Profil 8: Kose Giris Kilitlenme Orani (Lock %) [Yuksek = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, p8_lock_base[i] + 1.5, f"%{p8_lock_base[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, p8_lock_best[i] + 1.5, f"%{p8_lock_best[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

# 6. Senaryo Basina Toplam Maliyet (Cost) - Profil 8 Kose
ax = axes[2, 1]
cost_base = [base_m.get(f"p8_{d}", {}).get("cost", 0.0) for d in ["100cm", "200cm", "300cm"]]
cost_best = [best_m.get(f"p8_{d}", {}).get("cost", 0.0) for d in ["100cm", "200cm", "300cm"]]
ax.bar(x - width/2, cost_base, width, label="Baseline", color="#e74c3c", alpha=0.85)
ax.bar(x + width/2, cost_best, width, label="Optuna 24-Trial (RL)", color="#d35400", alpha=0.85)
ax.set_ylabel("Senaryo Maliyeti", fontweight="bold")
ax.set_title("Profil 8: Kose Giris Toplam Maliyeti [Dusuk = Iyi]")
ax.set_xticks(x)
ax.set_xticklabels(distances)
ax.legend()
for i in range(len(x)):
    ax.text(x[i] - width/2, cost_base[i] + 2.0, f"{cost_base[i]:.1f}", ha="center", fontsize=9)
    ax.text(x[i] + width/2, cost_best[i] + 2.0, f"{cost_best[i]:.1f}", ha="center", fontsize=9, fontweight="bold")

plt.tight_layout()
fig.savefig(OUT_IMG, dpi=200)
print(f"[OK] RL Baseline vs 24-Trial grafigi kaydedildi: {OUT_IMG}")
