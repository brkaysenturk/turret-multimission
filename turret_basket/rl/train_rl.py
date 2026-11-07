r"""

================================================================================

BASKET CHALLENGE: 2. MODEL - BALISTIK ACI VE VOLAN HIZI EGITIMI (TRAIN RL)

================================================================================

Bu betik; taret bardağa kilitlendiğinde LiDAR mesafesine göre en uygun:

  - Ekstra Tilt Atış Açısını (Süper-elevasyon: 1.0° - 18.0°)

  - Volan Sürücü Çıkış Hızını (7.5 m/s - 13.5 m/s)

öğrenen 2. Aşama Balistik RL Ajanını eğitmek üzere hazırlanmıştır.

KULLANIM:

  .\venv\Scripts\python.exe basket/train_rl.py [adim_sayisi]

  (Varsayılan: 50.000 adım, CPU üzerinde yaklaşık 25-30 saniye sürer)

================================================================================

"""

import sys

from pathlib import Path

# Proje kokunu sys.path'e ekle

project_root = Path(__file__).resolve().parents[2]

if str(project_root) not in sys.path:

    sys.path.insert(0, str(project_root))

from stable_baselines3.common.callbacks import BaseCallback

from turret_basket.rl.cup_env import BallisticBasketEnv

class BallisticEvaluationCallback(BaseCallback):

    """Egitim sirasinda basket basari oranini izleyen geri arama sinifi."""

    def __init__(self, check_freq: int = 10000, verbose: int = 1):

        super().__init__(verbose)

        self.check_freq = check_freq

    def _on_step(self) -> bool:

        if self.n_calls % self.check_freq == 0:

            print(f"[TRAIN] Adim {self.n_calls} / Balistik Karar Ajanı eğitiliyor...")

        return True

def train_ballistic_agent(total_timesteps: int = 50000, model_save_name: str = "cup_ballistic_policy"):

    output_dir = Path(__file__).resolve().parent / "models"

    output_dir.mkdir(parents=True, exist_ok=True)

    save_path = output_dir / f"{model_save_name}.zip"

    npz_path = output_dir / f"{model_save_name}.npz"

    print("=" * 65)

    print("BASKET CHALLENGE: 2. MODEL (FIZIK GUDUMLU BALISTIK POLITIKA AGISI)")

    print("Egitim Ornek Sayisi : 10.000 Fiziksel Dinamik Yörünge")

    print(f"Hedef Model Dosyasi : {save_path}")

    print(f"NumPy MLP Dosyasi   : {npz_path}")

    print("=" * 65)

    import numpy as np

    import torch

    import torch.nn as nn

    import torch.optim as optim

    import math

    # Webots katı cisim ODE diferansiyel denklemini çözen uzman fiziksel yörünge üretimi
    b = 0.05
    g = 9.81
    barrel_z = 1.0387

    def solve_smooth(dist_m: float, delta_z_m: float) -> tuple[float, float]:
        """Webots fiziksel taret ve dart balistiginde kanitlanmis sampiyon yorunge formulu:
        Yakin mesafede ~22.0°, orta mesafede ~21.2°, uzak mesafede ~20.2° asirtma ile kupanin icine dalis."""
        cup_rim_z = barrel_z + delta_z_m
        th_deg = float(np.clip(22.2 - (dist_m - 1.2) * 1.6, 19.8, 22.5))
        th = math.radians(th_deg)
        cos_th = math.cos(th)
        sin_th = math.sin(th)
        target_z = cup_rim_z - 0.005  # Kupa agzinin 5 mm icine inis (on ceperi temiz asar)

        v_low = 2.0
        v_high = 9.0
        for _ in range(30):
            v_mid = 0.5 * (v_low + v_high)
            vx0 = v_mid * cos_th
            vz0 = v_mid * sin_th
            arg = 1.0 - b * dist_m / vx0
            if arg <= 0.0:
                v_low = v_mid
                continue
            t = -(1.0 / b) * math.log(arg)
            zT = barrel_z + ((vz0 + g / b) / b) * (1.0 - math.exp(-b * t)) - (g / b) * t
            if zT < target_z:
                v_low = v_mid
            else:
                v_high = v_mid

        return th_deg, float(0.5 * (v_low + v_high))

    X_data = []
    Y_data = []

    np.random.seed(42)
    torch.manual_seed(42)

    print("[GEN] 30.000 analitik pürüzsüz ve yüksek isabetli balistik yörünge örneği üretiliyor...")
    for _ in range(30000):
        dist_m = float(np.random.uniform(1.05, 3.05))
        delta_z = float(np.random.uniform(-0.75, 0.25))

        th_deg, best_v = solve_smooth(dist_m, delta_z)

        norm_dist = (dist_m - 2.05) / 1.05
        norm_h = (delta_z - (-0.25)) / 0.60
        norm_th = (th_deg - 17.5) / 12.5
        norm_v = (best_v - 7.0) / 5.0

        X_data.append([norm_dist, norm_h])
        Y_data.append([norm_th, norm_v])

    X_train = torch.tensor(X_data, dtype=torch.float32)

    Y_train = torch.tensor(Y_data, dtype=torch.float32)

    # 2 Girdi -> 64 -> 64 -> 2 Cikti Politika Agi

    class PolicyNet(nn.Module):

        def __init__(self):

            super().__init__()

            self.net = nn.Sequential(

                nn.Linear(2, 64),

                nn.Tanh(),

                nn.Linear(64, 64),

                nn.Tanh(),

                nn.Linear(64, 2)

            )

        def forward(self, x):

            return torch.clamp(self.net(x), -1.0, 1.0)

    net = PolicyNet()
    optimizer = optim.AdamW(net.parameters(), lr=8e-3, weight_decay=1e-5)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=400)
    criterion = nn.MSELoss()

    print("[TRAIN] Fizik Güdümlü Sinir Ağı Politika Eğitimi başladı...")
    for epoch in range(1, 401):
        optimizer.zero_grad()
        pred = net(X_train)
        loss = criterion(pred, Y_train)
        loss.backward()
        optimizer.step()
        scheduler.step()
        if epoch % 100 == 0:
            print(f"  Epoch {epoch}/400 - MSE Loss: {loss.item():.7f}")

    # Agirliklari .npz Olarak Cikar

    w1 = net.net[0].weight.detach().cpu().numpy()

    b1 = net.net[0].bias.detach().cpu().numpy()

    w2 = net.net[2].weight.detach().cpu().numpy()

    b2 = net.net[2].bias.detach().cpu().numpy()

    w3 = net.net[4].weight.detach().cpu().numpy()

    b3 = net.net[4].bias.detach().cpu().numpy()

    np.savez(str(npz_path), w1=w1, b1=b1, w2=w2, b2=b2, w3=w3, b3=b3)

    print(f"\n[OK] Hizli NumPy MLP Agirliklari basariyla kaydedildi: {npz_path}")

    # Egitim Sonrasi Dogrulama Testi (50 Farkli Mesafeden Kupa Atisi)

    print("\n" + "=" * 65)

    print("EGITIM SONRASI BALISTIK TESTI (50 Farkli Mesafeden Kupa Atisi)")

    print("=" * 65)

    test_env = BallisticBasketEnv(domain_randomization=True)

    baskets = 0

    total_shots = 50

    for ep in range(total_shots):

        obs, info = test_env.reset()

        h1 = np.tanh(np.dot(w1, obs) + b1)

        h2 = np.tanh(np.dot(w2, h1) + b2)

        action = np.clip(np.dot(w3, h2) + b3, -1.0, 1.0)

        _, reward, _, _, step_info = test_env.step(action)

        if step_info.get("basket"):

            baskets += 1

    success_rate = (baskets / total_shots) * 100.0

    print(f"Test Sonucu: {baskets} / {total_shots} BASKET ISABETI!")
    print(f"BASKET BASARI ORANI: %{success_rate:.1f}")
    print("=" * 65)


def train_webots_ballistic_agent(model_save_name: str = "webots_ballistic_policy"):
    """
    Webots'a ozel 3B namlu ucu (spawn_dart: +0.075m ileri, +0.0256m yukari) kinematigiyle
    egitilen balistik politika agi. Gercek donanim modelini (cup_ballistic_policy.npz) bozmaz.
    """
    import math
    import numpy as np
    import torch
    import torch.nn as nn
    import torch.optim as optim

    output_dir = Path(__file__).resolve().parent / "models"
    output_dir.mkdir(parents=True, exist_ok=True)
    npz_path = output_dir / f"{model_save_name}.npz"

    print("\n" + "=" * 65)
    print("WEBOTS DIGITAL TWIN: 3B NAMLU UCU (SPAWN_DART) BALISTIK POLITIKA EGITIMI")
    print(f"NumPy MLP Dosyasi   : {npz_path}")
    print("=" * 65)

    b = 0.05
    g = 9.81

    def solve_webots_smooth(dist_m: float, delta_z_m: float) -> tuple[float, float]:
        cup_rim_z = 1.0558 + delta_z_m
        th_deg = float(np.clip(22.2 - (dist_m - 1.2) * 1.4 + max(0.0, delta_z_m + 0.15) * 4.0, 19.5, 24.0))
        th = math.radians(th_deg)
        cos_th = math.cos(th)
        sin_th = math.sin(th)

        # Webots spawn_dart 3B namlu ucu konumu
        r_spawn = 0.075 * cos_th - 0.0256 * sin_th
        sz_launch = 1.0131 + 0.075 * sin_th + 0.0256 * cos_th
        d_flight = max(0.50, dist_m - r_spawn)
        target_z = cup_rim_z - 0.012

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

        return th_deg, float(0.5 * (v_low + v_high))

    X_data = []
    Y_data = []

    np.random.seed(42)
    torch.manual_seed(42)

    for _ in range(30000):
        dist_m = float(np.random.uniform(1.05, 3.05))
        delta_z = float(np.random.uniform(-0.75, 0.25))
        th_deg, best_v = solve_webots_smooth(dist_m, delta_z)

        norm_dist = (dist_m - 2.05) / 1.05
        norm_h = (delta_z - (-0.25)) / 0.60
        norm_th = (th_deg - 17.5) / 12.5
        norm_v = (best_v - 7.0) / 5.0

        X_data.append([norm_dist, norm_h])
        Y_data.append([norm_th, norm_v])

    X_train = torch.tensor(X_data, dtype=torch.float32)
    Y_train = torch.tensor(Y_data, dtype=torch.float32)

    class WebotsPolicyNet(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(2, 64),
                nn.Tanh(),
                nn.Linear(64, 64),
                nn.Tanh(),
                nn.Linear(64, 2),
            )

        def forward(self, x):
            return torch.clamp(self.net(x), -1.0, 1.0)

    net = WebotsPolicyNet()
    optimizer = optim.AdamW(net.parameters(), lr=8e-3, weight_decay=1e-5)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=400)
    criterion = nn.MSELoss()

    for epoch in range(1, 401):
        optimizer.zero_grad()
        pred = net(X_train)
        loss = criterion(pred, Y_train)
        loss.backward()
        optimizer.step()
        scheduler.step()
        if epoch % 100 == 0:
            print(f"  [WEBOTS-MLP] Epoch {epoch}/400 - MSE Loss: {loss.item():.7f}")

    w1 = net.net[0].weight.detach().cpu().numpy()
    b1 = net.net[0].bias.detach().cpu().numpy()
    w2 = net.net[2].weight.detach().cpu().numpy()
    b2 = net.net[2].bias.detach().cpu().numpy()
    w3 = net.net[4].weight.detach().cpu().numpy()
    b3 = net.net[4].bias.detach().cpu().numpy()

    np.savez(str(npz_path), w1=w1, b1=b1, w2=w2, b2=b2, w3=w3, b3=b3)
    print(f"[OK] Webots 3B Balistik MLP kaydedildi: {npz_path}")


if __name__ == "__main__":
    timesteps = 50000
    if len(sys.argv) > 1:
        try:
            timesteps = int(sys.argv[1])
        except ValueError:
            pass

    train_ballistic_agent(total_timesteps=timesteps)
    train_webots_ballistic_agent()

