# Turret Basket — Analiz ve Doğrulama Araçları (Analysis Suite)

Bu dizin; **Basket Challenge (Kulplu Kupaya Otonom Basket Atma)** projesinin aerodinamik balistik yörüngelerini, 1. Model Görsel Kilitlenme (Cup Tracker) dinamiklerini, 2. Model Balistik Politika (RL Ballistics) kararlarını, 2-aşamalı fırlatma durum makinesi zaman çizelgesini ve Webots CAD tareti 24 Optuna hiperparametre optimizasyonu sonuçlarını incelemek için geliştirilen akademik analiz ve grafik motorlarını içerir.

Tüm analiz betikleri `turret_fcs` klasör mimarisiyle birebir aynı modüler standartta tasarlanmış olup, çıktılar **300 DPI** çözünürlükte, yüksek kontrastlı akademik yayın formatında kaydedilir.

---

## 📁 Dizin Hiyerarşisi

```text
turret_basket/analysis/
├── README.md                           # Modül dokümantasyonu ve kullanım kılavuzu
├── run_all_basket_analyses.py          # Tüm figürleri tek adımda üreten master yürütücü
├── webots_cup_tracker_trials.json      # Webots 24 Optuna denemesi ham telemetri verisi
├── baseline_vs_24_basket_table_comparison.png # Doğrudan tablo karşılaştırması
├── system/                             # Sistem aerodinamiği ve durum makinesi analizleri
│   └── python/
│       ├── plot_ballistics_rk4.py      # RK4 aerodinamik yörünge ve tolerans zarfları (Fig 1 & Fig 3)
│       └── plot_engagement_profile.py  # 2-Aşamalı atış FSM ve eyleyici zaman profili (Fig 4)
├── controllers/                        # Hiyerarşik RL kontrolcü karar ve takip analizleri
│   ├── tracker/
│   │   └── plot_cup_tracker.py         # 1. Model görsel kilitlenme basamak yanıtı ve faz portresi (Fig 5)
│   └── ballistics/
│       └── plot_rl_ballistics.py       # 2. Model RL balistik süper-elevasyon ve volan hız kestirimi (Fig 2)
├── comparison/                         # Kıyaslama, Optuna optimizasyonu ve benchmark motorları
│   ├── plot_optuna_trials.py           # 24-Trial Optuna minimax maliyet ve kilit yakınsama analizi (Fig 6)
│   └── plot_baseline_vs_24_basket_table.py # Baseline vs Trial 14 8-metrikli karşılaştırma tablosu (Fig 7)
└── figures/                            # Akademik yayın kalitesinde üretilen grafikler (300 DPI)
    └── python/
        ├── python_fig1_ballistic_trajectories_rk4.png      # RK4 Balistik Uçuş Yörüngeleri & Kupa Giriş Açısı
        ├── python_fig2_rl_ballistics_policy_decision.png   # 2. Model Balistik Politika Süper-elevasyon & Hız
        ├── python_fig3_basket_tolerance_heatmap.png        # Balistik Açı-Hız Basket Tolerans Zarfı (Heatmap)
        ├── python_fig4_engagement_timing_profile.png       # 2-Aşamalı Angajman Zaman Çizelgesi (1.58s Döngü)
        ├── python_fig5_cup_tracker_step_response.png       # 1. Model Görsel Kupa Kilitlenme Basamak Yanıtı
        ├── python_fig6_optuna_24trials_convergence.png     # Webots 24-Trial Optuna Yakınsama & Kararlılık
        └── python_fig7_baseline_vs_24_table.png            # Baseline vs Trial 14 Birebir Metrik Tablosu
```

---

## 📊 Üretilen Akademik Grafikler ve Kapsamları

### 1. [python_fig1_ballistic_trajectories_rk4.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig1_ballistic_trajectories_rk4.png)
- **Modül:** [plot_ballistics_rk4.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/system/python/plot_ballistics_rk4.py)
- **İçerik:** 4. Mertebe Runge-Kutta (RK4) sayısal integrasyonu ve aerodinamik hava sürtünmesi (k_drag = 0.05).
- **Kapsam:** 1.2m, 1.8m, 2.4m ve 3.0m mesafelerdeki seramik kupaya fırlatılan dartların uçuş eğrileri, tepe noktaları (apex), masa yüzeyi çarpışma kotu (-0.329m), kupa ön gövde bariyeri (-0.222m), kupa ağzı dik iniş açısı (>= 30 derece) ve uçuş süresi (200 - 600 ms).

### 2. [python_fig2_rl_ballistics_policy_decision.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig2_rl_ballistics_policy_decision.png)
- **Modül:** [plot_rl_ballistics.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/controllers/ballistics/plot_rl_ballistics.py)
- **İçerik:** 2. Model Balistik Karar Ajanlarının (`cup_ballistic_policy.npz` ve `webots_ballistic_policy.npz`) 0.8m - 3.2m menzil aralığındaki kestirimleri.
- **Kapsam:** ToF LiDAR menziline ve masa dikey kot farkına (`delta_z = dist * tan(tilt) - 0.0634`) göre hesaplanan süper-elevasyon açı ofseti (+15.0° - +30.0°), fırlatma volan hızı (2.5 - 9.0 m/s), TB6612FNG PWM duty döngüsü (80 - 255 ticks) ve tepe noktası yükselişi.

### 3. [python_fig3_basket_tolerance_heatmap.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig3_basket_tolerance_heatmap.png)
- **Modül:** [plot_ballistics_rk4.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/system/python/plot_ballistics_rk4.py)
- **İçerik:** 2.0m nominal mesafede Pitch Açı Hatası (+/- 1.4 derece) ile Volan Hız Hatası (+/- 0.8 m/s) faz uzayı.
- **Kapsam:** Kupa iç ağız çapı (7.6 cm / +/- 38 mm) tolerans zarfı, başarılı basket bölgesi, ön çeper çarpışması, kısa atış ve aşırı atış bölgelerinin 2B kontur ve saçılım analizi.

### 4. [python_fig4_engagement_timing_profile.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig4_engagement_timing_profile.png)
- **Modül:** [plot_engagement_profile.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/system/python/plot_engagement_profile.py)
- **İçerik:** `BasketEngagement` durum makinesinin 1.33 saniyelik 4-aşamalı tam atış döngüsü (100 Hz ayrık zaman).
- **Kapsam:** SPINUP_ELEVATE (0.50 s, Tilt yükselmesi ve volan devir alma), SETTLE (0.25 s, mekanik titreşim sönümleme), FIRING (80 ms solenoid 12V tetik darbesi), RECOVER (0.50 s solenoid geri çekilme ve optik merkeze yumuşak iniş) ve ikili protokol (STATE_TRACKING, STATE_LOCKED, STATE_FIRING) geçişleri.

### 5. [python_fig5_cup_tracker_step_response.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig5_cup_tracker_step_response.png)
- **Modül:** [plot_cup_tracker.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/controllers/tracker/plot_cup_tracker.py)
- **İçerik:** 1. Model Görsel Kupa Kilitlenme Motorunun (+6.5° pan, -4.2° tilt) ilk tespit hatasına kapalı çevrim basamak yanıtı.
- **Kapsam:** Pan ve tilt açılarının asimptotik yakınsaması, yumuşak Hermite taper ölü bölgesi (+/-0.16° pan, +/-0.15° tilt), 2B kamera retikül düzlemindeki hata rotası ve limit döngüsünü engelleyen faz portresi (Phase Portrait).

### 6. [python_fig6_optuna_24trials_convergence.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig6_optuna_24trials_convergence.png)
- **Modül:** [plot_optuna_trials.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/comparison/plot_optuna_trials.py)
- **İçerik:** Webots CAD tareti üzerinde koşturulan 24 Optuna denemesinin yakınsama eğrileri.
- **Kapsam:** Minimax robust maliyetin 3.60'tan 2.99'a inmesi (-%16.9), RMS kilitlenme hatasının 0.580°'den 0.452°'ye düşmesi (-%22.1), 2.6m mesafede kilitlenme oranının %76.1'den %100.0'e çıkması ve yön değiştirme oranının (%1.63) mekanik kararlılık sınırında kalması.

### 7. [python_fig7_baseline_vs_24_table.png](file:///c:/YTU/turret-fcs/turret_basket/analysis/figures/python/python_fig7_baseline_vs_24_table.png)
- **Modül:** [plot_baseline_vs_24_basket_table.py](file:///c:/YTU/turret-fcs/turret_basket/analysis/comparison/plot_baseline_vs_24_basket_table.py)
- **İçerik:** Baseline (Trial 0) ile 24-Trial Optuna Optimal (Trial 14) parametrelerinin 8 temel başarı metriğindeki doğrudan çubuk grafik karşılaştırması.

---

## 🚀 Hızlı Çalıştırma Komutları

Tüm analiz figürlerini tek seferde sırayla üretmek için:
```powershell
python turret_basket/analysis/run_all_basket_analyses.py
```

Modülleri tekil olarak çalıştırmak için:
```powershell
# 1. Aerodinamik RK4 Balistik Yörüngeler ve Tolerans Zarfları (Fig 1 & Fig 3)
python turret_basket/analysis/system/python/plot_ballistics_rk4.py

# 2. RL Balistik Politika Karar Motoru (Fig 2)
python turret_basket/analysis/controllers/ballistics/plot_rl_ballistics.py

# 3. 2-Aşamalı Angajman ve Eyleyici Zaman Çizelgesi (Fig 4)
python turret_basket/analysis/system/python/plot_engagement_profile.py

# 4. 1. Model Görsel Kilitlenme ve Basamak Yanıtı (Fig 5)
python turret_basket/analysis/controllers/tracker/plot_cup_tracker.py

# 5. Webots 24 Optuna Denemesi Yakınsama Analizi (Fig 6)
python turret_basket/analysis/comparison/plot_optuna_trials.py

# 6. Baseline vs 24-Trial Kıyaslama Tablosu (Fig 7)
python turret_basket/analysis/comparison/plot_baseline_vs_24_basket_table.py
```
