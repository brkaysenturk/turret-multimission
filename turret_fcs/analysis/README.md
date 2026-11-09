# Analysis Tools & Verification Suite (Analiz ve Doğrulama Araçları)

Bu dizin; Turret FCS projesinin kontrol dinamiklerini, sistem tanıma modellerini, Kalman filtresi kestirim doğruluğunu, Webots simülasyon ve gerçek donanım telemetri loglarını incelemek için geliştirilen akademik analiz ve doğrulama araçlarını modüler bir hiyerarşide içerir.

Tüm Python ve MATLAB betikleri işlevsel kategorilerine göre organize edilmiştir. Çıktı grafikleri 300 DPI çözünürlükte, akademik makale formatında (beyaz arka plan, koyu gri yüksek kontrastlı eksen ve etiketler) kaydedilir.

---

## 📁 Dizin Hiyerarşisi

```text
analysis/
├── system/                    # Sistem tanılama, transfer fonksiyonları ve dijital ikiz
│   ├── matlab/                # MATLAB & Simulink sistem modelleri ve HIL testleri
│   │   ├── system_id_turret.m          # FOPDT/SOPDT transfer fonksiyonu ve gecikme kestirimi
│   │   ├── control_stability_analysis.m # Root Locus, Bode, Nyquist kararlılık payları analizi
│   │   ├── build_turret_simulink.m     # Simulink dijital ikiz modeli oluşturucu
│   │   ├── run_turret_simulation.m     # Simulink kapalı çevrim benzetim çalıştırıcısı
│   │   ├── turret_hil_runner.m         # Donanım döngüde (HIL) gerçek zamanlı STM32 aracı
│   │   ├── turret_digital_twin.slx     # Simulink blok diyagramı
│   │   └── README.md                   # MATLAB & Simulink modül kılavuzu
│   └── python/                # Python analitik transfer fonksiyonu ve gecikme kestirimleri
│       ├── run_tau_identification.py   # Gerçek donanım saf basamak ve Bode tau/gecikme tanılama
│       └── tau_comparison_results.json # 81.5 ms gecikme ve 22 ms tau doğrulama raporu
├── logs/                      # Telemetri ve saha test logu inceleme motorları
│   ├── inspect_logs.py                 # Kilit süresi, açısal hata, salınım ve rezonans dedektörü
│   └── consolidate_telemetry_logs.py   # Parçalı telemetrileri birleştirme ve KPI özeti
├── controllers/               # FCS kontrolcü optimizasyon ve tarama araçları
│   ├── p/                     # Tip-1 P kontrolcü doğrulama ve tarama araçları
│   │   ├── optimize_p_controller.py    # Optuna TPE ile Kp kazancı hiperparametre optimizasyonu
│   │   ├── p_controller_benchmark.py   # Farklı hedef profillerinde P adım yanıtı benchmark'ı
│   │   ├── p_controller_validation.py  # Webots telemetrisinden balon yer gerçeği nişan analizi
│   │   ├── p_gain_sweep.py             # Kazanç taraması ve doyum analizi
│   │   ├── p_profile_sweep.py          # 6 farklı hedef uçuş profiline karşı kazanç matrisi
│   │   └── p_optuna_results.json
│   └── rl/                    # PPO Pekiştirmeli Öğrenme kontrolcü araçları
│       ├── optimize_rl_wrapper.py      # RL sarmalayıcı sönümleme ve ölü bölge optimizasyonu
│       ├── export_rl_policy.py         # PyTorch politikasını hafif 9-64-64-2 NumPy MLP'ye aktarır
│       ├── rl_diagnose.py              # PPO politikasının eylem sürekliliği ve başarım teşhisi
│       └── best_rl_wrapper_params.json
├── kalman/                    # CV Kalman filtresi optimizasyon ve kestirim doğrulaması
│   ├── optimize_kalman.py              # Süreç ve ölçüm kovaryansı optimizasyonu
│   ├── generate_kalman_plots.py        # CV Kalman hız kestirimi vs yer gerçeği karşılaştırması
│   └── best_kalman_params.json
├── patrol/                    # Otonom devriye ve alan tarama analiz araçları
│   └── plot_patrol_log.py              # Lissajous devriye tarama yörüngesi ve servo adımları
├── comparison/                # Çoklu kontrolcü kıyaslama ve genel sistem analizleri
│   ├── generate_benchmark_plots.py     # Saf PPO RL vs Tip-1 P TurretEnv karşılaştırması
│   ├── generate_readme_diagrams.py     # Güncel CONFIG SVG mimari şemaları üreticisi
│   ├── plot_tuned_system.py            # 8 panelli analitik model ve faz portresi
│   ├── run_webots_benchmark.py         # Webots simülasyon benchmark yürütücüsü
│   ├── optimize_ballistics_roc.py      # Atış kapılama ve balistik ROC eğrisi optimizasyonu
│   ├── optimize_selector_evt.py        # Hedef seçici EVT eşik optimizasyonu
│   ├── optimal_lock_params.json
│   └── optimal_selector_params.json
└── figures/                   # Tüm akademik analiz ve doğrulama grafikleri
    ├── python/                # Python analiz ve doğrulama figürleri (python_fig1 - python_fig18)
    └── matlab/                # MATLAB & Simulink kararlılık ve HIL figürleri (matlab_fig1 - matlab_fig6)
```

---

## 🚀 Hızlı Başlangıç Komutları

### 1. Sistem Tanılama ve Kararlılık Analizleri
```bash
# MATLAB Sistem Tanılama (FOPDT/SOPDT, 50.1 ms Gecikme, 0.88 deg Dişli Boşluğu)
matlab -batch "addpath('analysis/system/matlab'); system_id_turret; exit"

# MATLAB Klasik Kontrol Kararlılık Analizi (Bode GM=41.9 dB, PM=89.2 deg, Root Locus, Nyquist)
matlab -batch "addpath('analysis/system/matlab'); control_stability_analysis; exit"

# Simulink Dijital İkiz Benzetimi (Kapalı Çevrim P Takip)
matlab -batch "addpath('analysis/system/matlab'); run_turret_simulation; exit"

# Donanım Döngüde (HIL) Canlı Test (Port: COM1 veya Otomatik Mock Fallback)
matlab -batch "addpath('analysis/system/matlab'); turret_hil_runner; exit"

# Python Sistem Tanılama Grafiği
python analysis/system/python/system_identification.py
```

### 2. Kontrolcü Benchmark ve Optimizasyon
```bash
# Tip-1 P Kontrolcü Optuna TPE Optimizasyonu
python analysis/controllers/p/optimize_p_controller.py

# P Kontrolcü Farklı Hedef Profili Doğrulaması
python analysis/controllers/p/p_controller_benchmark.py

# P vs RL Nihai TurretEnv Benchmark Karşılaştırması (fig11)
python analysis/comparison/generate_benchmark_plots.py

# 8 Panelli Taret Dinamik Davranış Figürleri (fig1 - fig8)
python analysis/comparison/plot_tuned_system.py
```

### 3. Kalman ve Telemetri İnceleme
```bash
# Kalman Filtresi Hız Kestirim Başarımı (fig10)
python analysis/kalman/generate_kalman_plots.py

# En Son Telemetri Kütüğünün Hızlı İncelenmesi
python analysis/logs/inspect_logs.py

# Otonom Devriye (Patrol) Kütüğü Analizi
python analysis/patrol/plot_patrol_log.py
```
