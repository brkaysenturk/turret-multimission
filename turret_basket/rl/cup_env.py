"""

================================================================================

BASKET CHALLENGE: 2. MODEL - BALISTIK ACI VE VOLAN HIZI ORTAMI (CUP BASKET ENV)

================================================================================

Bu ortam; taret bardağa kilitlendikten sonra (1. Model sayesinde), kupanın

mesafesine ve yüksekliğine göre:

  1. Ekstra Atış Açısını (Tilt Offset / Süper-elevasyon: 1.0° - 18.0°)

  2. Volan Fırlatma Hızını (Dart Muzzle Velocity: 7.5 m/s - 13.5 m/s)

belirleyen 2. Aşama Balistik RL Karar Ajanının eğitim ortamıdır.

================================================================================

"""



import math

from typing import Dict, Any, Tuple

import numpy as np

import gymnasium as gym

from gymnasium import spaces



from turret_basket.config import CONFIG





def solve_rk4_trajectory(

    dist_m: float,

    v0: float,

    theta_launch_rad: float,

    table_rel_y: float = -0.3287,  # Masa yuzeyi (0.710m - 1.0387m CAD v10 namlu)

    rim_rel_y: float = -0.2222,    # Bardak agzi kenari (0.8165m - 1.0387m CAD v10 namlu)

    bottom_rel_y: float = -0.3137, # Bardak dibi ic yuzeyi (0.725m - 1.0387m CAD v10 namlu)

    cup_outer_r: float = 0.045,    # 9.0 cm dis cap (cup.obj)

    cup_inner_r: float = 0.038,    # 7.6 cm ic agiz acikligi

    g: float = 9.80665,

    k_drag: float = 0.05,          # Webots b=0.05 katı cisim dart lineer sonumleme katsayisi

    dt: float = 0.0005,

    max_time_s: float = 1.5

) -> Tuple[float, float, float, float, float, bool, bool]:

    """

    Dartın namludan (y=0) çıkıp hedef mesafesine (dist_m) ulaşmasını RK4 ile simüle eder.

    Masa yüzeyi çarpışması ve bardağın ön çeper duvar bariyerini tam Webots fiziğiyle denetler.

    Dönüş: (final_y, t_flight, v_end, vy, apex_y, hit_table, hit_front_lip)

    """

    x = 0.0

    y = 0.0

    vx = v0 * math.cos(theta_launch_rad)

    vy = v0 * math.sin(theta_launch_rad)

    t = 0.0

    apex_y = 0.0

    hit_table = False

    hit_front_lip = False



    while x < dist_m and t < max_time_s:

        step_dt = dt

        if vx > 1e-4 and (dist_m - x) < vx * step_dt:

            step_dt = max(1e-5, (dist_m - x) / vx)



        def deriv(s_vx, s_vy):

            ax = -k_drag * s_vx

            ay = -g - k_drag * s_vy

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

        if y > apex_y:

            apex_y = y



        # 1. Masa Yuzeyi Carpismasi (Z = 0.710m ahsap masa)

        if y <= table_rel_y:

            hit_table = True

            break



        # 2. Bardagin On Govde/Kenar Duvari Engeli (On dis kenar girisi)
        # Dart tam on kenar girisine ulastiginda yuksekligi agiz seviyesinin altindaysa on duvara carpar!
        if (dist_m - cup_outer_r) <= x < (dist_m - cup_outer_r + 0.005):
            if y < rim_rel_y:
                hit_front_lip = True
                break



    return y, t, math.hypot(vx, vy), vy, apex_y, hit_table, hit_front_lip





class BallisticBasketEnv(gym.Env):

    """

    2. Model: Kupa mesafesine gore Tilt Ofseti ve Volan Hizi Karar Ortami.

    Tek adimlik karar ve balistik atis (Contextual Bandit / Episodic RL).

    """



    metadata = {"render_modes": ["human"], "render_fps": 30}



    def __init__(self, domain_randomization: bool = True):

        super().__init__()

        self.domain_randomization = domain_randomization



        # Gozlem Uzayi: 2 Boyut [-1.0, 1.0]

        # [norm_distance, norm_height_offset]

        self.observation_space = spaces.Box(

            low=-1.0,

            high=1.0,

            shape=(CONFIG.rl.obs_dim,),

            dtype=np.float32

        )



        # Eylem Uzayi: 2 Boyut [-1.0, 1.0]

        # [norm_tilt_offset, norm_flywheel_speed]

        self.action_space = spaces.Box(

            low=-1.0,

            high=1.0,

            shape=(CONFIG.rl.act_dim,),

            dtype=np.float32

        )



        self.cup_distance_m = 2.0

        self.cup_height_offset_m = 0.0

        self.cup_mouth_radius_m = CONFIG.cup.mouth_diameter_m / 2.0  # 0.045 m



    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict[str, Any]]:

        super().reset(seed=seed)



        # Kupa mesafesini rastgele sec (1.15 metre ile 2.85 metre arasi)
        # Masa yukseklik ofsetini rastgele sec (-0.30m alcak sehpa ile +0.22m yuksek masa arasi)
        if self.domain_randomization:
            self.cup_distance_m = float(self.np_random.uniform(1.15, 2.85))
            self.cup_height_offset_m = float(self.np_random.uniform(-0.30, 0.22))
        else:
            self.cup_distance_m = 2.0
            self.cup_height_offset_m = 0.0



        obs = self._get_obs()

        return obs, {"distance_m": self.cup_distance_m}



    def _get_obs(self) -> np.ndarray:
        # Ultra-genisletilmis zarf: [1.0m .. 3.1m] ve [-0.85m .. +0.35m]
        norm_dist = np.clip((self.cup_distance_m - 2.05) / 1.05, -1.0, 1.0)
        delta_z = -0.2287 + self.cup_height_offset_m
        norm_height = np.clip((delta_z - (-0.25)) / 0.60, -1.0, 1.0)
        return np.array([norm_dist, norm_height], dtype=np.float32)



    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:

        # Eylemleri Fiziksel Birimlere Olcekle

        # Action 0: Tilt Ofseti (1.0° .. 18.0°)

        tilt_min = CONFIG.rl.tilt_offset_min_deg

        tilt_max = CONFIG.rl.tilt_offset_max_deg

        tilt_offset_deg = float(tilt_min + (action[0] + 1.0) * 0.5 * (tilt_max - tilt_min))



        # Action 1: Volan Cikis Hizi (7.5 m/s .. 13.5 m/s)

        speed_min = CONFIG.rl.speed_min_mps

        speed_max = CONFIG.rl.speed_max_mps

        launch_speed = float(speed_min + (action[1] + 1.0) * 0.5 * (speed_max - speed_min))



        # Fiziksel Parametreler ve Domain Randomization

        v0 = launch_speed

        k_drag = 0.05

        if self.domain_randomization:

            # Gercek volan surucu devir hassasiyeti ve hava kosullari

            v0 += float(self.np_random.normal(0.0, 0.05))

            k_drag += float(self.np_random.normal(0.0, 0.001))



        # A Yontemi: Taret zaten dogrudan kupa agzina (rim) kilitlenir.

        # Bu nedenle nisan hatti ile kupa agzi ayni optik eksendedir (relative target y = 0).

        # Firlatma acisi saf asirtma ofsetidir:

        theta_launch_rad = math.radians(tilt_offset_deg)

        # CAD v10: Namlu dunya yuksekligi 1.0387m, bardak agzi 0.8165m, bardak ici dibi 0.725m
        # Masa yukseklik ofsetini fiziksel olarak dinamik ekle:
        cur_table_y = -0.3287 + self.cup_height_offset_m
        cur_rim_y = -0.2222 + self.cup_height_offset_m
        cur_bottom_y = -0.3137 + self.cup_height_offset_m
        target_mouth_y = cur_rim_y - 0.015

        final_y, t_flight, v_end, vy, apex_y, hit_table, hit_front_lip = solve_rk4_trajectory(
            dist_m=self.cup_distance_m,
            v0=v0,
            theta_launch_rad=theta_launch_rad,
            table_rel_y=cur_table_y,
            rim_rel_y=cur_rim_y,
            bottom_rel_y=cur_bottom_y,
            k_drag=k_drag
        )

        vertical_miss_m = abs(final_y - target_mouth_y)

        # BASKET KONTROLU (Gercek Webots Katı Cisim Dart Fiziği):
        # 1. Masaya çarpmamış olmalı (hit_table == False)
        # 2. Bardağın ön gövdesine/kenar duvarına çarpmamış olmalı (hit_front_lip == False)
        # 3. Dart bardağın ağız çemberinden içeri girmiş olmalı (cur_bottom_y <= final_y <= cur_rim_y)
        # 4. Dart yukarıdan aşağıya dik inmeli (vy < -0.30 m/s)
        is_in_mouth = (cur_bottom_y <= final_y <= cur_rim_y)
        is_descending = (vy < -0.30)
        is_basket = (not hit_table) and (not hit_front_lip) and is_in_mouth and is_descending



        # Enerji tasarrufu ve pürüzsüzlük: Gereksiz aşırı yüksek motor devrini hafifçe cezalandır

        speed_penalty = float(0.02 * (launch_speed ** 2))



        # Odul Hesaplama (Pruzsuz Gauss Yakinlik + Basket Bonusu)

        if hit_table:

            # Masaya çakılma cezası (yetersiz hız veya hatalı açı)

            reward = -50.0 - 15.0 * vertical_miss_m

        elif hit_front_lip:

            # Bardağın ön gövdesine çarpma cezası (alçak yatay yörünge)

            reward = -40.0 - 15.0 * vertical_miss_m

        elif is_basket:

            # Temiz aşırtmalı basket ödülü (+100) + dik iniş bonusu

            reward = CONFIG.rl.basket_reward + min(30.0, abs(vy) * 15.0) - speed_penalty

        else:

            # Bardağın üstünden veya arkasından kaçırma

            reward = 60.0 * math.exp(-0.5 * (vertical_miss_m / 0.035) ** 2) - 20.0 - speed_penalty



        info = {

            "basket": is_basket,

            "miss_dist_m": vertical_miss_m,

            "tilt_deg": tilt_offset_deg,

            "speed_mps": launch_speed,

            "distance_m": self.cup_distance_m,

            "descending": is_descending,

            "hit_table": hit_table,

            "hit_front_lip": hit_front_lip

        }



        terminated = True  # Tek adimlik karar ve atis

        truncated = False

        obs = self._get_obs()



        return obs, float(reward), terminated, truncated, info





# Geriye donuk isim uyumlulugu

CupBasketEnv = BallisticBasketEnv

