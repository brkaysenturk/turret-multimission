# Basket Challenge — Tekrar Üretilebilirlik Kılavuzu (Reproducibility Guide)

Bu doküman; Kulplu Kupaya Basket Atma (Basket Challenge) projesinin testlerinin, 2-aşamalı pekiştirmeli öğrenme modellerinin (1. Aşama Görsel Takip ve 2. Aşama Balistik Karar Ajanı), simülasyon ortamlarının ve analiz grafiklerinin bağımsız araştırmacılar tarafından sıfırdan nasıl tekrar üretilebileceğini adım adım açıklamaktadır.

---

## 1. Ortam Kurulumu ve Bağımlılıklar

Proje Python 3.10, 3.11 ve 3.12 sürümleri ile test edilmiş ve doğrulanmıştır.

```bash
# Depoyu klonlayın ve proje kök dizinine geçin
git clone https://github.com/brkaysenturk/turret-multimission.git
cd turret-multimission

# Sanal ortam oluşturun ve aktif edin
python -m venv venv
venv\Scripts\activate   # Windows
# source venv/bin/activate # Linux / macOS

# Projeyi ve test bağımlılıklarını kurun
pip install -e .[test]
```

---

## 2. Otomatik Test Paketinin Koşulması

Sistem, donanıma (kamera veya fiziksel STM32) ihtiyaç duymadan mock sürücüler ve analitik simülatörler üzerinden tüm alt bileşenleri doğrular.

### Basket Challenge Özel Testleri (15 Test)

```bash
pytest tests/test_basket_*.py -v
```

Bu test paketi şu kritik işlevleri doğrular:

1. **`test_basket_config_ssot`:** Kamera parametreleri (640x480, 30 FPS, odak uzaklığı), donanım mekanik açı sınırları, hedef kupa boyutları (9.0 cm ağız çapı, 10.6 cm yükseklik) ve atış zamanlamalarının (600 ms devirlenme, 80 ms darbe, 900 ms geri çekilme) tüm modüllerde tutarlı okunduğunu denetler.
2. **`test_basket_protocol_constants`:** [protocol.py](file:///c:/YTU/turret-multimission/turret_basket/protocol.py) ile [protocol.h](file:///c:/YTU/turret-multimission/turret_basket/stm32/protocol.h) arasındaki bayt uzunlukları, başlıklar (`0xAA`, `0x55`, `0x59`) ve açı ölçekleme (0.1 derece) sabitlerinin birebir eşleştiğini doğrular.
3. **`test_basket_comm_packet_serialization_and_checksum`:** 8 baytlık komut paketi serileştirmesini, 10 baytlık telemetri ayrıştırmasını ve 8-bit checksum algoritmasını doğrular.
4. **`test_basket_rl_tracker_inference`:** 1. Modelin (CupTracker) 9-64-64-2 NumPy MLP mimarisinde doğru eylem ürettiğini ve yumuşak ölü bölge (soft deadband) sönümlemesini doğrular.
5. **`test_basket_rl_ballistics_inference`:** 2. Modelin (RLBallistics) ToF LiDAR mesafesine göre dinamik süper-elevasyon (+21.8° ile +22.6°) ve volan fırlatma hızı (4.4 m/s) ürettiğini doğrular.
6. **`test_basket_engagement_fsm`:** 2-Aşamalı atış durum makinesinin (SPINUP_ELEVATE -> FIRING -> RECOVER) zaman aşımı sürelerini ve bayraklarını doğrular.
7. **`test_basket_manager_lifecycle`, `test_basket_manager_range_latch_on_lock`, `test_basket_manager_real_system_improvements`:** Hedef tespiti, menzil kapılama (range gating), kilit anı mesafe mandallama ve atış yetkilendirme yaşam döngüsünü doğrular.
8. **`test_basket_ballistics_rk4_trajectory` & `test_webots_3d_muzzle_ballistics_and_ccd`:** 4. Derece Runge-Kutta (RK4) aerodinamik sürtünmeli parabolik yörünge çözücüsünü ve Webots 3B namlu CCD çarpışma modelini test eder.
9. **`test_basket_ballistics_gym_environment`:** `BallisticBasketEnv` Gymnasium ortamının adımlama, gözlem normalizasyonu ve sepet ödül fonksiyonunu doğrular.
10. **`test_basket_hud_and_presenter`:** OpenCV tabanlı baş-üstü gösterge (HUD) katmanlarının ve durum panellerinin hatasız çizildiğini doğrular.
11. **`test_basket_keymap_events`:** Klavye komut eşlemelerinin (SPACE, Y, N, R, E, Q, ESC) durum makinesiyle uyumlu çalıştığını denetler.
12. **`test_cup_mouth_lockon_and_distance_standardization`:** Kupa ağzı elips oranı ve mesafe standardizasyonunu doğrular.

### Tüm Proje Test Paketi (52 Test)

```bash
pytest -v
```

Turret FCS ve Basket Challenge'ın tüm 52 testini koşturarak sistem genelinde hiçbir regresyon olmadığını doğrular.

---

## 3. Model Ağırlıkları ve Dosya Yapısı

Basket Challenge görevi iki aşamalı hiyerarşik model mimarisi kullanır:

```
turret_basket/rl/models/
├── cup_tracker_policy.npz         # 1. Model: Dışa aktarılmış 9-64-64-2 saf NumPy MLP ağırlıkları
├── cup_tracker_params.json        # 1. Model: Optuna TPE ile optimize edilmiş takip parametreleri
├── cup_tracker_params_webots.json # 1. Model: Webots dijital ikizinde optimize edilmiş parametreler
├── cup_ballistic_policy.npz       # 2. Model: Dışa aktarılmış saf NumPy MLP balistik karar ağırlıkları
└── cup_ballistic_policy.zip       # 2. Model: Stable-Baselines3 PPO politika arşivi
```

- **Nesne Tespiti (YOLOv8n):** Masadaki kulplu kupayı tespit etmek için `weights/yolov8n.pt` ağırlıkları kullanılır (COCO sınıfları: 41 `cup`, 40 `wine glass`, 39 `bottle`, 44 `bowl`).
- **1. Model (Görsel Kilitlenme - CupTracker):** Kupayı görüntünün optik merkezine kilitler (`err_x -> 0`, `err_y -> 0`).
- **2. Model (Balistik Karar Ajanı - RLBallistics):** LiDAR mesafesini (1.0m - 3.0m) alarak dartın kupa ağzına girmesi için gereken ekstra tilt açısını (süper-elevasyon) ve volan hızını belirler.

---

## 4. 2. Modelin Sıfırdan Eğitilmesi (Balistik Karar Ajanı)

LiDAR mesafesine göre atış açısını ve volan motoru hızını öğrenen fizik güdümlü 2. Modeli eğitmek için:

```bash
python turret_basket/rl/train_rl.py 50000
```

Bu eğitim süreci:
1. `BallisticBasketEnv` ortamında 4. Derece Runge-Kutta (RK4) aerodinamik sürtünmeli simülatör ile 10.000 farklı parabolik yörünge verisi üretir.
2. PyTorch ile 3 katmanlı MLP ağını 400 epoch boyunca MSE kaybı ile eğitir.
3. Eğitilen ağırlıkları saf NumPy formatında `turret_basket/rl/models/cup_ballistic_policy.npz` olarak dışa aktarır (bağımsız ve mikro saniyeler mertebesinde hızlı çıkarım sağlar).
4. Eğitim sonunda 50 farklı mesafeden test atışı yaparak sepet isabet oranını doğrular (tipik başarı oranı: %100).

---

## 5. 1. Modelin Optuna ile İnce Ayarı ve Analizi

Kupanın merkezine kilitlenirken motor titreşimini (hunting), aşımı ve kilitlenme süresini optimize etmek için Optuna TPE algoritması kullanılır.

### Simülatör Üzerinde Optuna Optimizasyonu

9 kanonik kupa konumunda (Yakın 1.0m, Nominal 2.0m, Uzak 3.0m; Sol, Merkez, Sağ) hiperparametre optimizasyonu yapmak için:

```bash
python turret_basket/rl/optimize_cup_tracker.py
```

En iyi parametreler `turret_basket/rl/models/cup_tracker_params.json` dosyasına kaydedilir.

### Webots Kapalı Çevrim Optuna Optimizasyonu

Webots simülatöründe gerçekçi motor eylemsizliği ve dişli boşluğu (backlash) altında kapalı çevrim optimizasyon için:

```bash
python turret_basket/rl/optimize_cup_tracker_webots.py
```

### Optuna Karşılaştırma Grafiğinin Üretilmesi

12 denemelik optimizasyon seyrini, maliyet düşüşünü ve kilit kararlılığını gösteren 4 panelli grafik figürünü üretmek için:

```bash
python turret_basket/analysis/plot_optuna_results.py
```

Üretilen görsel: `turret_basket/analysis/cup_tracker_trial_comparison.png`.

---

## 6. Webots Simülasyon Dünyasında Çalıştırma

Webots R2025a ortamında taret ve kulplu kupa dijital ikizini koşturmak için:

1. Webots'u açın ve dünyayı yükleyin:
   ```
   webots/worlds/turret_basket.wbt
   ```
2. Dünyada seramik kupa (`DEF CUP`) ve tabanında temas algılayıcı (`cup_basket_sensor TouchSensor`) yer almaktadır.
3. Kontrolcü: [webots_turret_basket.py](file:///c:/YTU/turret-multimission/webots/controllers/webots_turret_basket/webots_turret_basket.py).
4. Simülasyonu başlatın:
   - **SPACE:** Dartı fırlatır (2-aşamalı durum makinesi devrededir).
   - **R:** Kupayı masada rastgele yeni bir mesafeye taşır (1.3m - 2.7m).
   - Ekranda canlı [basket_hud.py](file:///c:/YTU/turret-multimission/turret_basket/basket_hud.py) baş-üstü göstergesi görüntülenir; dart kupa tabanına değdiğinde temas sensörü algılar ve `BASKET!` skoru kaydedilir.

---

## 7. Gerçek Donanım Üzerinde Çalıştırma (Production Main)

Gerçek STM32F401RE taretini ve Logitech C270 kamerasını bağlayıp masadaki kupaya basket atmak için ana giriş noktası:

```bash
python turret_basket/main.py
```

veya Windows komut dosyası ile:

```bat
turret_basket\main.bat
```

### Operatör Klavye Kontrolleri:

- **SPACE veya Y:** Atış emrini onaylar ve 2 aşamalı fırlatmayı başlatır (Süper-elevasyona yükseliş -> 600 ms Spin-up -> 80 ms Solenoid itme darbesi -> Optik kilide geri sönümleme).
- **N:** Başlatılan atış emrini derhal iptal eder (Abort).
- **R:** Hedef takipçisini ve kilit geçmişini sıfırlar.
- **E:** E-Stop acil durum freni; motorları keser ve servoları kilitler.
- **Q veya ESC:** Sistemi, kamerayı ve seri portu güvenli şekilde kapatır.
