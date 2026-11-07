"""
================================================================================
MODUL: WEBOTS 3B HAREKETLI HEDEF KONTROLCUSU (BALLOON MOVER - v2.1)
================================================================================
Balonun 9 test hareketini, secilebilir profil ile uretir. Profil secimi:
  - WEBOTS_BALLOON_PROFILE ortam degiskeni (0-8), veya
  - webots/balloon_profile.json icindeki active_profile.
Profil 8 (KOSE GIRIS / ILK KILIT): Her 5 saniyede bir tareti merkeze cekip
takipciyi sifirlar ve balonu 4 farkli kadraj kosesinden (+/-17 deg pan,
+/-6 deg tilt) aniden sokarak ilk kilitlenme osilasyonunu test eder.
================================================================================
"""

import json
import math
import os
from pathlib import Path

from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

balloon_node = robot.getSelf()
trans_field = balloon_node.getField("translation")

initial_pos = trans_field.getSFVec3f()
start_x = initial_pos[0]
fixed_y = initial_pos[1]
dist_env = os.environ.get("WEBOTS_BALLOON_DISTANCE")
if dist_env is not None:
    try:
        fixed_y = float(dist_env)
    except ValueError:
        pass
fixed_z = initial_pos[2]

# 0 statik, 1 yavas, 2 standart, 3 cevik, 4 step, 5 zigzag, 6 tilt-only, 7 figure-8, 8 kose giris
PROFILES = {
    0: {"name": "STATIK", "kind": "static", "ax": 0.00, "az": 0.00, "wx": 1.0, "wz": 1.0},
    1: {"name": "YAVAS SINUS", "kind": "sine", "ax": 0.20, "az": 0.06, "wx": 0.50, "wz": 0.60},
    2: {"name": "STANDART SINUS", "kind": "sine", "ax": 0.38, "az": 0.10, "wx": 0.60, "wz": 0.70},
    3: {"name": "CEVIK SINUS", "kind": "sine", "ax": 0.55, "az": 0.16, "wx": 0.75, "wz": 0.90},
    4: {"name": "STEP / HOLD", "kind": "step", "ax": 0.45, "az": 0.10, "wx": 0.35, "wz": 0.35},
    5: {"name": "ZIGZAG", "kind": "zigzag", "ax": 0.45, "az": 0.08, "wx": 0.55, "wz": 0.55},
    6: {"name": "TILT-ONLY", "kind": "tilt", "ax": 0.00, "az": 0.20, "wx": 1.0, "wz": 0.70},
    7: {"name": "FIGURE-8", "kind": "figure8", "ax": 0.40, "az": 0.14, "wx": 0.60, "wz": 1.20},
    8: {"name": "KOSE GIRIS / ILK KILIT", "kind": "corner_entry", "ax": 0.60, "az": 0.20, "wx": 0.05, "wz": 0.02},
}

CORNER_SEQUENCE = (
    (1.0, 1.0),    # Sag-Ust Kose
    (-1.0, -1.0),  # Sol-Alt Kose
    (-1.0, 1.0),   # Sol-Ust Kose
    (1.0, -1.0),   # Sag-Alt Kose
)


def _profile_id():
    env = os.environ.get("WEBOTS_BALLOON_PROFILE")
    if env is not None:
        try:
            return max(0, min(8, int(env)))
        except ValueError:
            pass
    path = Path(__file__).resolve().parents[2] / "balloon_profile.json"
    try:
        return max(0, min(8, int(json.loads(path.read_text(encoding="utf-8")).get("active_profile", 2))))
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return 2


PROFILE_ID = _profile_id()
P = PROFILES[PROFILE_ID]
TREMOR_M = float(os.environ.get("WEBOTS_BALLOON_TREMOR", "0.005"))


def _triangle(phase):
    """0..1 fazini -1..+1 ucgen dalgasina cevirir."""
    phase = phase % 1.0
    if phase < 0.5:
        return -1.0 + 4.0 * phase
    return 3.0 - 4.0 * phase


def target_offsets(t):
    kind = P["kind"]
    dist_scale = fixed_y / 2.0 if fixed_y > 0 else 1.0
    ax, az, wx, wz = P["ax"] * dist_scale, P["az"] * dist_scale, P["wx"], P["wz"]
    if kind == "static":
        return 0.0, 0.0
    if kind == "sine":
        return ax * math.sin(wx * t), az * math.sin(wz * t)
    if kind == "step":
        # 4 s sagda, 4 s solda; gecislerde yumusak rampa
        period = 8.0
        phase = (t % period) / period
        if phase < 0.45:
            x = ax
        elif phase < 0.50:
            x = ax * (1.0 - (phase - 0.45) / 0.05 * 2.0)
        elif phase < 0.95:
            x = -ax
        else:
            x = -ax * (1.0 - (phase - 0.95) / 0.05 * 2.0)
        return x, az * math.sin(wz * t)
    if kind == "zigzag":
        return ax * _triangle(wx * t / (2.0 * math.pi)), az * math.sin(wz * t)
    if kind == "tilt":
        return 0.0, az * math.sin(wz * t)
    if kind == "figure8":
        return ax * math.sin(wx * t), az * math.sin(2.0 * wx * t)
    if kind == "corner_entry":
        # 5.0 saniyelik periyotlarla 4 farkli kadraj kosesinden ilk giris testi:
        # - 0.00 .. 3.40 s: Balon kosede belirir ve hafif iceri suzulur (Ilk Kilitlenme testi)
        # - 3.40 .. 4.40 s: Balon merkeze cekilir (taret merkeze doner)
        # - 4.40 .. 5.00 s: Balon kadraj disinda gizlenir (takipci sifirlanir, yeni koseye hazirlanir)
        cycle_s = 5.0
        idx = int(t // cycle_s) % len(CORNER_SEQUENCE)
        local_t = t % cycle_s
        sx, sz = CORNER_SEQUENCE[idx]
        if local_t < 3.40:
            # Koseden giris ve cok yavas iceri suzulme
            drift = local_t
            return sx * (ax - wx * drift), sz * (az - wz * drift)
        if local_t < 4.40:
            # Merkeze geri cekilme rampasi
            ratio = 1.0 - (local_t - 3.40) / 1.00
            end_x = sx * (ax - wx * 3.40)
            end_z = sz * (az - wz * 3.40)
            return end_x * ratio, end_z * ratio
        # 0.60 s boyunca gizle (z = -20 m) ki FCS hedef kaybi yasayip sifirlansin
        return None, None
    return 0.0, 0.0


print(f"[HEDEF] Webots profil {PROFILE_ID}: {P['name']}")

is_popped = False
pop_time = 0.0
RESPAWN_DELAY_SEC = 2.0
sim_time = 0.0

while robot.step(timestep) != -1:
    sim_time = robot.getTime()
    dx, dz = target_offsets(sim_time)
    if dx is None or dz is None:
        trans_field.setSFVec3f([0.0, 0.0, -20.0])
        continue
    # El tutusu / hava akimi kaynakli kucuk, tekrarlanabilir tremor.
    dx += TREMOR_M * math.sin(5.3 * sim_time)
    dz += TREMOR_M * math.sin(7.7 * sim_time + 0.4)
    new_x = start_x + dx
    new_z = fixed_z + dz

    if is_popped:
        trans_field.setSFVec3f([0.0, 0.0, -20.0])
        if (sim_time - pop_time) >= RESPAWN_DELAY_SEC:
            is_popped = False
            trans_field.setSFVec3f([new_x, fixed_y, new_z])
            print("[BALON] Yeni hedef balon belirdi! Takip devam ediyor.")
    else:
        trans_field.setSFVec3f([new_x, fixed_y, new_z])
        cmd = robot.getCustomData()
        if cmd == "POP":
            is_popped = True
            pop_time = sim_time
            robot.setCustomData("")
            print("\n" + "=" * 65)
            print(">>> [BALON PATLADI / TARGET POPPED!] <<<")
            print("=" * 65 + "\n")
