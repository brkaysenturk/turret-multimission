# Modular Autonomous Turret Platform (Turret Multi-Mission)

## Yıldız Teknik Üniversitesi — Kontrol ve Otomasyon Mühendisliği Bölümü

Bitirme Çalışması ve Tasarım Projesi (Senior Capstone & Design Project)

[![Release: v3.0](https://img.shields.io/badge/Release-v3.0%20Production-success.svg)](https://github.com/brkaysenturk/turret-fcs/releases/tag/v3.0)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10-3.12](https://img.shields.io/badge/Python-3.10--3.12-blue.svg)](https://www.python.org/)
[![CI](https://github.com/brkaysenturk/turret-fcs/actions/workflows/ci.yml/badge.svg)](https://github.com/brkaysenturk/turret-fcs/actions/workflows/ci.yml)
[![Tests](https://img.shields.io/badge/Tests-51%20passed-brightgreen.svg)](tests/)
[![Hardware](https://img.shields.io/badge/Hardware-STM32F401RE-002B49.svg)](https://www.st.com/)
[![LiDAR](https://img.shields.io/badge/LiDAR-TF--Luna%20ToF-blueviolet.svg)](https://en.benewake.com/)
[![Simulation](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![Detection](https://img.shields.io/badge/Detection-YOLOv8n%20%2B%20ByteTrack-00AEEF.svg)](https://ultralytics.com/)
[![Control](https://img.shields.io/badge/Control-Type--1%20P%20%2B%20PPO-orange.svg)](turret_fcs/p_controller.py)

**Turret Multi-Mission Platform**, modüler bir 2-eksenli elektro-optik taret üzerinde çalışan iki bağımsız ve uzmanlaşmış otonom görevi bünyesinde barındıran monorepo mimarisidir:
1. **Hava Savunma Atış Kontrol Sistemi (`turret_fcs`):** Dinamik hava tehditlerine (balon/İHA) karşı optik tespit, sabit hızlı Kalman kestirimi, Tip-1 P veya PPO RL takip ve kapalı form vuruş olasılığı ile önleme.
2. **Kupa Bardağı Basketi Görevi (`turret_basket`):** Dar ağızlı bardağa kavisli dart basketi atan 2-aşamalı hiyerarşik yapay zeka (Cup Mouth Tracker + Sinir Ağı / RK4 Balistiği) ve Webots temas fiziği optimizasyonu.

Her iki görev de aynı fiziksel donanım omurgasını (STM32F401RE Nucleo, MG996R/DS3218 servolar, TF-Luna ToF LiDAR, çift FF-180 volan, JF-0530B solenoid ve C270 kamera) ve Webots R2025a dijital ikiz altyapısını paylaşır.

> **English Overview:** Modular Autonomous Turret Platform developed as a senior capstone project at Yildiz Technical University. Built as an extensible monorepo, it houses two specialized mission pipelines sharing the same physical and simulated hardware:
> - **Mission 1 (`turret_fcs`):** Autonomous Air Defense Fire Control System with YOLOv8n + ByteTrack, CV Kalman filter with outlier rejection, dual Type-1 P / 11-D PPO tracking, lead compensation, and closed-form Gaussian hit probability.
> - **Mission 2 (`turret_basket`):** Cup Basket Challenge featuring a 2-stage hierarchical AI (Cup Mouth Ellipse Tracker + Neural / RK4 Ballistic Elevation) solving projectile arc dynamics into a narrow cup.

---

## 🎯 İki Görev Bir Bakışta (Dual-Mission Matrix)

| Yetenek / Özellik | 🎯 Görev 1: Hava Savunma FCS (`turret_fcs`) | 🏀 Görev 2: Bardak Basketi (`turret_basket`) |
| :--- | :--- | :--- |
| **Ana Amaç** | Hareketli asimetrik hava hedeflerini havada önleme | Dar ağızlı kupa bardağı içine kavisli dart basketi atma |
| **Hedef Tipi** | Dinamik uçan balonlar ve mikro İHA'lar | Statik / yarı-statik kupa bardağı ağız elipsi |
| **Görsel Model** | YOLOv8n + ByteTrack (uzamsal kapılama & histerezis) | YOLO CupDetector + Geometrik elips tepe oranı (%10) |
| **Kestirim Modeli** | CV Kalman Filtresi (100 px inovasyon kapısı & aday geri oynatma) | Düşük gecikmeli kutu EMA + Parallaks geometrik düzeltme |
| **Kontrol Mimarisi** | A/B Modu: Tip-1 P Kontrolcü veya 11-D PPO RL Modeli | Aşama 1: PPO Cup Mouth Tracker (hassas yönlendirme) |
| **Balistik Çözücü** | Doğrusal süper-elevasyon + Kinematik lead ve uçuş süresi | Aşama 2: Sinir Ağı Balistiği (MLP) + RK4 aerodinamik yörünge |
| **Volan Kontrolü** | 217/255 (~%85 sabit nominal devir) | 130..255 mesafeye duyarlı dinamik PWM devri |
| **Webots Dünyası** | `webots/worlds/turret_fcs.wbt` (Balon hareketlendirici) | `webots/worlds/turret_basket.wbt` (Bardak & dart kontakt fiziği) |
| **Tek Tıkla Başlatma** | `turret_fcs\main.bat` | `turret_basket\main.bat` |
| **Modül Komutu** | `python -m turret_fcs.main` | `python -m turret_basket.main` |

## İçindekiler

1. [Sistem Ne Yapar?](#1-sistem-ne-yapar)
2. [Sistem Bir Bakışta](#2-sistem-bir-bakışta)
3. [Güncel Sistem Mimarisi](#3-güncel-sistem-mimarisi)
4. [Çalışma Akışı ve Durum Makinesi](#4-çalışma-akışı-ve-durum-makinesi)
5. [Donanım ve Elektriksel Mimari](#5-donanım-ve-elektriksel-mimari)
6. [Yazılım Modülleri](#6-yazılım-modülleri)
7. [Kontrol ve Kestirim Teorisi](#7-kontrol-ve-kestirim-teorisi)
8. [Güvenlik ve Fail-Safe Mimarisi](#8-güvenlik-ve-fail-safe-mimarisi)
9. [Performans ve Doğrulama](#9-performans-ve-doğrulama)
10. [Hızlı Başlangıç](#10-hızlı-başlangıç)
11. [Operatör Kullanımı](#11-operatör-kullanımı)
12. [Telemetri ve Analiz](#12-telemetri-ve-analiz)
13. [Dizin Yapısı](#13-dizin-yapısı)
14. [Dokümantasyon](#14-dokümantasyon)
15. [Lisans, Atıf ve Güvenlik](#15-lisans-atıf-ve-güvenlik)

---

## 1. Sistem Ne Yapar?

Sistem, tek bir operatörün hava sahasını izlemesine ve yalnızca güvenli kilit koşulları sağlandığında atış yapmasına yardımcı olur:

1. **Arar:** Hedef yokken taret, 1.5 saniyelik bekleme ve merkeze dönüşün ardından 8 saniyelik Lissajous (figure-8) taraması yapar.
2. **Algılar:** Her karede YOLOv8n hedef adaylarını üretir; ByteTrack kimlikleri ve güven ağırlıklı hedef seçici sahte tespitleri eler.
3. **Kestirir:** Sabit hızlı Kalman filtresi hedefin konumunu ve dünya referanslı hızını kestirir; taretin kendi dönüşünden kaynaklanan optik kayma ego-hareket gözlemcisiyle çıkarılır.
4. **Köprüler:** Hedef 1-5 kare boyunca kaybolursa, ego-telafili `predict_only` tahminiyle yörünge kısa süreliğine sürdürülür.
5. **Önler:** LiDAR menzili, mermi hızı ve sistem gecikmesi kullanılarak uçuş süresi, kinematik lead ve yerçekimi süper-elevasyonu hesaplanır.
6. **Yönlendirir:** Operatörün seçtiği P (hassas kilit) veya RL (çevik takip) modu, pan/tilt için artımlı açı komutu üretir.
7. **Kilitler:** Dinamik eşik, 10 karelik kararlılık ve histerezis ile kilit doğrulanır; 2B Gauss modeliyle anlık vuruş olasılığı hesaplanır.
8. **Ateşler:** Operatör `[Y]` tuşuna bastığında SPINUP, SETTLE, FIRING, RECOVER ve EVAL aşamaları sırayla çalışır.
9. **Korur:** 300 ms heartbeat, 800 ms bağımsız watchdog, mandallı E-stop, zaman tabanlı hız sınırı ve donanımsal pull-down ile güvenli duruma geçer.

### Öne çıkan özellikler

- **30 FPS optik algılama + 100 Hz LiDAR:** Görüntü ve menzil farklı hızlarda akar; FCS ikisini aynı karar döngüsünde birleştirir.
- **Aday yörünge geri oynatmalı Kalman:** 100 px tekil sıçrama ve %99.9 Ki-kare inovasyon kapısı; 3 kare süren tutarlı sıçramada aday yörünge geri oynatımıyla yeni hedefe gecikmesiz geçer.
- **Kısa kayıp köprüsü:** 1. derece servo ego modeli ve `predict_only` ile 1-5 karelik tespit boşlukları kapatılır.
- **Çift kontrolcü:** P modu düşük titreşimli hassas kilit için, RL modu hızlı manevralarda gecikmeye dayanıklı takip için kullanılır.
- **Kapalı form vuruş olasılığı:** Ölümcül hedef çekirdeği, hedef hızı, uçuş süresi, gecikme ve silah saçılımı tek bir olasılığa dönüştürülür.
- **Katmanlı güvenlik:** Python, STM32 firmware ve elektriksel katman birbirinden bağımsız koruma sağlar.
- **Operatör yetkisi:** Otomatik atış yoktur; her atış `[Y]` onayı, kilit, menzil ve minimum vuruş olasılığı kapılarından geçer.

---

## 2. Sistem Bir Bakışta

| Alt Sistem | Uygulama | Görevi |
| :--- | :--- | :--- |
| **Görsel algılama** | Logitech C270, 1280x720 @ 30 FPS, yatay FOV 48.8°, dikey FOV 28.6° | Hedef görüntüsünü sıfır kuyruk gecikmesiyle sağlar |
| **Derin öğrenme** | Ultralytics YOLOv8n + ByteTrack (FCS) / CupDetector (Basket) | Hava hedefi ve kupa bardağı tespiti |
| **Menzil** | Benewake TF-Luna ToF LiDAR, 850 nm, 0.2 - 8.0 m, 100 Hz UART | Uçuş süresi ve balistik düşüş için gerçek mesafe |
| **Kestirim** | Sabit hızlı (CV) Kalman, Joseph formu, 100 px sıçrama ve %99.9 Ki-kare kapısı, aday geri oynatma | Hedef konumu ve dünya referanslı hız |
| **Kontrol** | Tip-1 P ve saf PPO RL (`turret_fcs`); Cup Mouth Tracker + Sinir Ağı Balistiği (`turret_basket`) | Pan/tilt açı ve süper-elevasyon komutu |
| **Kilit** | Dinamik eşik, 10 kare kararlılık, 2B Gauss vuruş olasılığı | Atış izni ve kilit kalitesi |
| **Gömülü denetleyici** | STM32F401RE Nucleo, ARM Cortex-M4 @ 84 MHz | 100 Hz servo/LiDAR döngüsü, güvenlik bekçileri |
| **Eyleyiciler** | MG996R/DS3218 servolar, çift FF-180 volan, JF-0530B solenoid | Taret yönlendirme ve dart fırlatma |
| **Arayüz** | MIL-STD taktik HUD + Telemetri şeması 2.1 | Operatör kararı ve mühendislik kaydı |
| **Dijital ikiz** | Webots R2025a, 90.0 ms taşıma gecikmesi, 30.0 ms servo tau, 1.61° backlash (`turret_fcs.wbt`, `turret_basket.wbt`) | Donanıma zarar vermeden doğrulama |

---

## 3. Güncel Sistem Mimarisi

Sistem dört katmandan oluşur. Üst katmanlar karar üretir, alt katmanlar bu kararı fiziksel eyleme dönüştürür.

![Turret FCS Sistem Mimarisi](turret_fcs/docs/assets/turret_fcs_architecture.svg?v=2)

*Şema, katmanların ve veri akışının görsel özetidir; sayısal parametrelerde tek kaynak `turret_fcs/config.py` içindeki `CONFIG`'dir.*

### Katmanların görevleri

1. **Algılama ve sensör füzyonu:** Kamera kareleri, YOLOv8n/ByteTrack tespitleri ve TF-Luna menzil ölçümleri toplanır. Kamera thread'i her zaman en güncel kareyi verir; LiDAR verisi STM32 üzerinden 100 Hz akar.
2. **FCS çekirdeği:** Hedef seçici, Kalman filtresi, balistik/lead hesabı, kilit yöneticisi, kısa kayıp köprüsü ve devriye üreteci bu katmanda çalışır. Kararın tek kaynağı `turret_fcs/config.py` içindeki `CONFIG` nesnesidir.
3. **Kontrol katmanı:** Operatörün seçtiği P veya RL kontrolcüsü, hedef hatasından pan/tilt adımı üretir. İki mod aynı anda çalışmaz; `[M]` tuşuyla geçiş yapılır.
4. **Gömülü denetleyici, eyleme ve arayüz:** STM32 komutu servo PWM'ine, volan enable hattına ve solenoid darbesine çevirir; HUD ve telemetri aynı anda operatöre ve kayda sunulur.

---

## 4. Çalışma Akışı ve Durum Makinesi

![Turret FCS Algoritmik Akış Şeması](turret_fcs/docs/assets/turret_fcs_flowchart.svg)

### Durumlar ve geçişler

| Durum | Ne zaman? | Sistem ne yapar? | Çıkış |
| :--- | :--- | :--- | :--- |
| **ARAMA / DEVRİYE** | Güvenilir hedef yok | 1.5 s son konumu korur, merkeze döner, 8 s figure-8 taraması yapar | Yeni hedef adayı |
| **TESPİT** | YOLO güven eşiğini geçen aday var | Track ID, alan, en-boy oranı, uzamsal kapı ve güven ağırlığı denetimi | Takip edilecek hedef kutusu |
| **TAKİP** | Hedef seçildi | Kalman kestirimi, ego telafisi, LiDAR kapılama, lead ve süper-elevasyon | Pan/tilt açı komutu |
| **KISA KAYIP** | Tespit 1-5 kare kaçtı | Ego-telafili `predict_only` ile yörünge ilerletilir | Tahmini takip komutu |
| **KİLİT** | 10 kare kararlılık + dinamik eşik | Vuruş olasılığı, histerezis ve HUD kilit göstergesi | Atış izni beklenir |
| **ANGAJMAN** | Operatör `[Y]` onayı | SPINUP, SETTLE, FIRING, RECOVER, EVAL | Dart fırlatma ve değerlendirme |
| **E-STOP** | `[Space]`, B1 butonu veya kritik hata | Mandallı durdurma; motorlar merkeze hız sınırlı iner | Tüm komutlar kilitli |

### Kademeli atış zaman çizelgesi

```text
[Y] ──> SPINUP + SLEW (0.25 s)
          │  volan nominal devre çıkar, taret lead + süper-elevasyon açısına yönelir
          v
        SETTLE (0.15 s)
          │  nişan açısı sabit tutulur, servo lag ve dişli boşluğu oturur
          v
        FIRING (80 ms)
          │  STM32 solenoidi kenar tetiklemeli olarak sürer
          v
        RECOVER (0.25 s)
          │  solenoid yayla geri çekilir, taret optik eksene döner
          v
        EVAL (1.50 s)
             atış sonucu gösterilir; ilk 0.75 s kilit muafiyeti uygulanır
```

> Atış yalnızca dört koşul birlikte sağlandığında başlar: hedef kilitli, vuruş olasılığı en az %25, LiDAR menzili geçerli/kararlı ve angajman sekansı boşta. Bu kapılar `engagement.py` içinde tek bir yerde tutulur.

---

## 5. Donanım ve Elektriksel Mimari

![Turret FCS Donanım ve Güç Dağıtım Şeması](turret_fcs/docs/assets/turret_fcs_hardware.svg)

*Şema bağlantı topolojisini özetler; pin ve güç katı ayrıntılarında [firmware kılavuzu](turret_fcs/docs/firmware.md) ve aşağıdaki tablolar esastır.*

### Temel bileşenler

| Bileşen | Model / Arayüz | Görevi |
| :--- | :--- | :--- |
| **İşlemci** | PC (Windows 10/11) | Görüntü işleme, YOLO, Kalman, kontrol ve HUD |
| **Gömülü denetleyici** | STM32F401RE Nucleo-64, 84 MHz | Gerçek zamanlı servo/LiDAR döngüsü ve güvenlik |
| **Kamera** | Logitech C270, USB UVC | 1280x720 @ 30 FPS görüntü |
| **LiDAR** | Benewake TF-Luna, UART 115200 | 100 Hz ToF mesafe ve sinyal gücü |
| **Pan servo** | MG996R / DS3218, 50 Hz PWM | Yatay eksen, ±75° |
| **Tilt servo** | MG996R / DS3218, 50 Hz PWM | Dikey eksen, ±25° |
| **Volan** | Çift FF-180, MOSFET enable | 23.500 RPM nominal atış devri |
| **Solenoid** | JF-0530B, 12 V | 80 ms dart itici darbe |
| **Güç** | 12 V adaptör + XL4015 6 V buck | Servo/volan ve solenoid hatları |

### STM32 pin bağlantıları

| Fonksiyon | Pin | Arayüz | Not |
| :--- | :--- | :--- | :--- |
| Pan servo | `PB4` | TIM3_CH1, 50 Hz PWM | 570 - 2470 µs, 15° - 165° |
| Tilt servo | `PB5` | TIM3_CH2, 50 Hz PWM | 570 - 2470 µs, 65° - 115° |
| Volan enable | `PB0` | GPIO output | N-MOSFET enable |
| Solenoid tetik | `PB1` | GPIO output | Optokuplör / MOSFET |
| TF-Luna RX/TX | `PA10` / `PA9` | USART1, 115200 | 100 Hz ToF |
| PC RX/TX | `PA3` / `PA2` | USART2, 115200 | ST-Link VCP |
| Durum LED | `PA5` | GPIO output | 500 ms kalp atışı |
| Kullanıcı butonu | `PC13` | EXTI, dahili pull-up | Mandallı yazılım E-stop (250 ms debounce) |

### Elektriksel koruma notları

- 12 V ana hat, XL4015 ile 6 V'a düşürülür; servo ve volan motorları 6 V hattından beslenir.
- 6 V hattında 1000 µF düşük ESR filtre kondansatörü bulunur.
- Solenoid, IRLZ44N logic-level N-MOSFET ve 1N4007 ters EMK diyodu ile sürülür; tetik optokuplörle izole edilir.
- Tüm modüller tek bir yıldız noktasında topraklanır; motor dönüş akımları sinyal toprağına taşınmaz.
- STM32 açılışta volan ve solenoid pinlerini donanımsal pull-down ile 0 V'ta tutar.

### Mekanik tasarım

![Turret FCS Elektromekanik Donanım Yerleşimi](turret_fcs/docs/assets/mechanical_design.jpg)

![Turret FCS 3D Mekanik Montaj](turret_fcs/docs/assets/turret_cad_render_annotated.jpg)

Taret; 3D baskı açık beşik, doğrudan tahrikli pan/tilt servolar ve minimum optik paralaks yerleşimiyle kurulmuştur. Kamera ve LiDAR aynı yönelimde monte edilir; CAD v10 tasarımında namlu ucu Z ekseninde kameradan 57 mm yukarıdadır (`camera_to_muzzle_z_m = 0.0570 m`, 5.7 cm dikey paralaks). FCS balistik motoru bu geometrik dikey paralaksı süper-elevasyon hesabında otomatik olarak telafi eder.

---

## 6. Yazılım Modülleri

Bu bölüm, her modülün **ne olduğunu ve sistem içinde hangi işi yaptığını** açıklar. Modüllerin tamamı `turret_fcs/` paketi altında yer alır; ortak parametreler `CONFIG` üzerinden okunur.

### 6.1 Modül haritası

| Katman | Modül | Ne işe yarar? | Ana çıktı |
| :--- | :--- | :--- | :--- |
| **Orkestrasyon** | `main.py` | Kamera, YOLO, FCS, STM32, HUD ve telemetriyi tek döngüde birleştirir | 30 FPS çalışma döngüsü |
| **Orkestrasyon** | `fcs_manager.py` | Hedef seçimi, Kalman, balistik, kilit, mod, kayıp köprüsü ve devriye koordinasyonu | Pan/tilt açı komutu |
| **Orkestrasyon** | `engagement.py` | Atış güvenlik kapıları ve 5 aşamalı angajman zaman çizelgesi | FCS durum baytı |
| **Orkestrasyon** | `keymap.py` | Operatör tuşlarını FCS, angajman ve E-stop eylemlerine çevirir | Mod/atış/E-stop olayı |
| **Algılama** | `camera_thread.py` | Kamerayı ayrı thread'de okur, en güncel kareyi sunar | Kare + kare kimliği |
| **Algılama** | `target_selector.py` | YOLO kutuları arasından güvenilir, kararlı hedefi seçer | Hedef kutusu |
| **Algılama** | `weights.py` | YOLO ağırlıklarını indirir ve SHA256 ile doğrular | `best.pt`, `yolov8n.pt` |
| **Kestirim** | `kalman_filter.py` | Hedef konumu/hızını kestirir, ego hareketini çıkarır, sıçramayı geri oynatır | `[x, y, vx, vy]` |
| **Kilit** | `lock_manager.py` | Dinamik kilit eşiği, histerezis ve vuruş olasılığı | `is_locked`, `hit_probability` |
| **Kontrol** | `p_controller.py` | Tip-1 P ile hassas, düşük titreşimli takip | Pan/tilt adımı |
| **Kontrol** | `rl_controller.py` | PPO politikasıyla çevik takip | Pan/tilt adımı |
| **Kontrol** | `config.py` | Tüm alt sistemlerin tek doğruluk kaynağı | `CONFIG` |
| **İletişim** | `protocol.py` | PC-STM32 çerçeve uzunluğu, başlık, ölçek ve durum kodları | Ortak protokol sözleşmesi |
| **İletişim** | `fcs_comm.py` | STM32 ile 115200 Baud ikili protokol | TX/RX paketleri |
| **Arayüz** | `fcs_hud.py` | Nişangah, kilit bari, LEAD, vuruş olasılığı çizer | HUD katmanı |
| **Arayüz** | `presenter.py` | HUD çizim sırasını ve durum metinlerini yönetir | Tam kare |
| **Arayüz** | `telemetry.py` | 35 kolonlu CSV, meta.json ve olay günlüğü yazar | Oturum kaydı |
| **Davranış** | `patrol_generator.py` | Hedef yokken bekleme, merkeze dönüş ve figure-8 tarama | Devriye adımı |
| **RL eğitimi** | `rl/turret_env.py` | Gerçek plant kısıtlarıyla Gymnasium ortamı | 11-D gözlem |
| **RL eğitimi** | `rl/train_rl.py` | PPO politikasını eğitir | `.zip` + `.npz` model |
| **Webots** | `webots/controllers/webots_turret_fcs/` | Üretim kodunu dijital ikizde çalıştırır | Simülasyon oturumu |
| **Gömülü** | `turret_fcs/stm32/` | Servo, LiDAR, volan ve solenoidin gerçek zamanlı denetimi | PWM / UART / GPIO |
| **Basket (Orkestrasyon)** | `turret_basket/basket_manager.py` | Bardağa kilitlenme, ToF menzil kapılama ve 2-aşamalı yapay zeka | Basket atış komutu |
| **Basket (Takip & Balistik)** | `turret_basket/rl_tracker.py`, `rl_ballistics.py` | Cup Mouth Tracker ve sinir ağı / RK4 süper-elevasyon balistiği | Açı + volan hızı |
| **Basket (Gömülü)** | `turret_basket/stm32/` | Kupa dart atışı için optimize edilmiş 100 Hz STM32 firmware | PWM / UART / GPIO |
| **Basket (Webots)** | `webots/controllers/webots_turret_basket/` | Dart-bardak temas fiziği ve kupa dünyası simülasyonu | Simülasyon oturumu |

### 6.2 Çalışma zamanı ve orkestrasyon

#### `main.py` — Ana çalışma döngüsü

Sistemin giriş noktasıdır. Kamerayı ve STM32 portunu otomatik bulur, YOLOv8 modelini yükler, her karede hedef tespiti ve FCS güncellemesi yapar, motor komutlarını STM32'ye gönderir, HUD'u çizer ve telemetriyi kaydeder. STM32 bulunamazsa sistem çevrimdışı modda çalışmaya devam eder; kamera bulunamazsa HUD uyarısı verilir ve görüntü döngüsü başlamaz. Çıkışta telemetri dosyalarını, seri portu ve kamerayı güvenli biçimde kapatır.

**Ne sağlar:** Tüm alt sistemlerin aynı zaman tabanında ve aynı kare üzerinde çalışmasını sağlar. Kamera, YOLO, kontrol ve UART gecikmeleri aynı döngüde ölçülür.

#### `fcs_manager.py` — FCS çekirdeği

Sistemin karar beynidir. Sırasıyla:

- hedef kutusunu seçer ve açısal hatayı hesaplar,
- LiDAR menzilini kapılar (sıçrama reddi, 3 kare kararlılık, geometri denetimi),
- ego-hareketi çıkararak Kalman filtresini günceller,
- hedef 1-5 kare kaybolursa ego-telafili `predict_only` köprüsünü çalıştırır,
- uçuş süresi, lead ve yerçekimi telafisini hesaplar,
- kilit durumunu ve vuruş olasılığını günceller,
- seçili kontrolcüden (P veya RL) pan/tilt adımı alır,
- hedef kaybında devriye üretecini çalıştırır.

**Ne sağlar:** Algılama ile eyleme arasındaki tüm karar zincirini tek bir yerde toplar; alt modüller birbirinden bağımsız test edilebilir.

#### `engagement.py` — Angajman durum makinesi

Atışın güvenlik kapılarını ve zaman çizelgesini yönetir. `trigger()` yalnızca hedef kilitli, vuruş olasılığı en az %25, menzil geçerli ve sekans boştaysa `True` döner. `update()` simülasyon zamanına göre SPINUP, SETTLE, FIRING, RECOVER ve EVAL aşamalarını üretir; her aşamada FCS durum baytı ve slew faktörü hesaplanır.

**Ne sağlar:** Atışın tek bir yerden, deterministik ve tekrarlanabilir biçimde yönetilmesini sağlar. HUD ve STM32 aynı durum bilgisini kullanır.

#### `keymap.py` — Operatör girişi

`ESC/Q`, `Space`, `M`, `P`, `Y` ve `N` tuşlarını işler. E-stop mandallıdır: ilk basışta devriye kapanır, FCS sıfırlanır, angajman iptal edilir; ikinci basışta sistem yeniden devreye alınır. `M` tuşu P/RL modunu değiştirir; `Y` yalnızca kilit, menzil ve vuruş olasılığı kapıları açıksa angajman başlatır.

**Ne sağlar:** Operatörün sistem üzerindeki tüm yetkisini tek bir denetlenebilir giriş noktasında toplar.

### 6.3 Algılama ve hedef seçimi

#### `camera_thread.py` — Sıfır gecikmeli kamera

OpenCV `VideoCapture` nesnesini ayrı bir daemon thread içinde çalıştırır. MSMF, gerekirse DirectShow arka ucuyla 1280x720 MJPG akışı açar; tamponu 1'e düşürür ve ana döngüye her zaman en güncel kareyi verir. `wait_for_new` ile aynı karenin tekrar işlenmesi engellenir; 350 ms'den uzun kare gelmezse okuma başarısız sayılır.

**Ne sağlar:** USB kamera tamponundan kaynaklanan bayat kare gecikmesini ve kare tekrarını önler.

#### `target_selector.py` — Hedef önceliklendirme

YOLO kutularını güven, alan, en-boy oranı ve nisangah mesafesine göre filtreler. Kilitli hedefin ByteTrack kimliğini korur; kimlik atlaması şüpheliyse 173 px'lik uzamsal kapı uygular. Hedef kaybolduğunda `max_lost_frames` (5 kare) boyunca histerezisi korur. Arama modunda `conf >= 0.30`, kilitli takipte `conf >= 0.16` eşiği kullanılır. Son konuma yakınlık ve ekran merkezine yakınlık, güven skoruyla ağırlıklandırılır (`dist_sq / max(conf, 0.1)`).

**Ne sağlar:** Sahte tespitleri ve hedef kimliği sıçramalarını eler; kısa kayıplarda takibi düşürmez.

#### `weights.py` — Model ağırlığı yöneticisi

`best.pt` ve `yolov8n.pt` dosyalarının varlığını ve SHA256 özetini denetler. Dosya yoksa GitHub Releases üzerinden indirir, geçici dosyada doğrular ve atomik olarak yerine taşır.

**Ne sağlar:** Ağırlıkların elle kurulum gerektirmeden, bütünlük doğrulamasıyla hazır olmasını sağlar.

### 6.4 Kestirim, balistik ve kilit

#### `kalman_filter.py` — CV Kalman filtresi

Durum vektörü `[x, y, vx, vy]`, ölçüm `[x, y]` şeklindedir. Sabit hızlı model, tek süreç gürültüsü `sigma_a = 478.19 px/s^2` ve tek ölçüm gürültüsü `r_var = 63.21 px^2` ile çalışır. Kovaryans güncellemesi Joseph formundadır. Taretin son motor komutundan üretilen ego kayması kestirim adımına eklenir; böylece hız durumu dünya referanslı kalır.

Tek karelik 100 px üstü sahte kutu sıçramasında veya %99.9 Ki-kare istatistiksel inovasyon kapısında ölçüm durumu bozmaz; filtre yalnızca tahminle (`predict-only`) ilerletilir. Aynı yeni bölgede 3 tutarlı ölçüm görülürse ilk sıçrama karesinden itibaren aday ölçümler geri oynatılır (`replay`) ve hız vektörü sıfırlanmadan yeni hedefe geçilir. Çıkışta 30-60 px/s yumuşak hız ölü bölgesi vardır.

**Ne sağlar:** YOLO kutu gürültüsünü ve taret dönüşünün yarattığı sahte hızı bastırır; kısa kayıplarda ve hedef sıçramalarında kararlı hız üretir.

#### `lock_manager.py` — Kilit ve vuruş olasılığı

Hedef hızına göre dinamik kilit eşiği uygular: 150 px/s altında 1.70°, 150-350 px/s arasında 2.25°, üstünde 2.95°. Çıkış eşiği +0.60° histerezisli, kararlılık sayacı 10 karedir. Atış sonrası 22 karelik (0.75 s) kilit muafiyeti tanır.

Vuruş olasılığında hedefin yalnızca %60'lık ölümcül çekirdeği kullanılır; bu çekirdek fiziksel hedef yarıçapıyla üstten sınırlanır. Toplam belirsizlik, silah saçılımı (1.25°) ve `0.35 * omega * (t_flight + 0.0900 s)` hareket teriminin geometrik toplamıdır. Sonuç, takip kararlılık sayacı (`s_track`) ve dinamik kapı aşımı (`s_gate`) ile modüle edilir; böylece taret hedefe yönelirken sahte %99 vuruş olasılığı oluşmaz.

**Ne sağlar:** Titreşimli kilit açılıp kapanmasını önler ve operatöre anlık, fiziksel temelli bir vuruş kalitesi gösterir.

#### `p_controller.py` — Hassas kilit kontrolcüsü

Tek EMA (alpha = 0.9475) ile olcum gurultusunu yumusatir. Yumusak olu bolge (pan 0.766 deg, tilt 0.530 deg) icinde motoru sifirlar; disinda Kp * (|e| - db) ile oransal komut uretir. Pan Kp = 0.1921, tilt Kp = 0.1669 degerindedir. Komut, pan 556.58 deg/s^2 ve tilt 211.81 deg/s^2 ivme sinirina ve kare basina 1.50 deg / 0.75 deg adim sinirina tabidir. Ki ve Kd yoktur; asimetrik frenleme (asymmetric braking) ve ucus suresi tahmincisi (in-flight damping = 0.6875) ile asim ve salinim engellenir.

**Ne sağlar:** Statik ve yavaş hedeflerde düşük titreşimli, öngörülebilir ve mekanik olarak nazik bir kilit sağlar.

#### `rl_controller.py` — Çevik takip kontrolcüsü

PPO politikasini Stable-Baselines3 veya PyTorch gerektirmeyen 11-64-64-2 NumPy MLP ile calistirir. Gozlem 11 boyuttadir: normalize hata (x, y), EMA ile filtrelenmis hata turevi (x, y), hiz (x, y), normalize mesafe ve son iki adimin normalize eylem gecmisi. Hata turevi olcegi 0.05, EMA katsayisi 0.4675'tir. Calisma zamaninda hiz gozlemi sifirlanir; politika konum hatasi, hata turevi ve eylem gecmisiyle kararli kalir.

Cikista bagimsiz acisal yumusak olu bolge (pan 0.490 deg, tilt 0.621 deg), Hermite taper (pan 0.400 deg, tilt 0.469 deg), 3 karelik (~90 ms) yon degistirme sonumu (katsayi 0.6637), tilt ekseninde 0.75 deg donanimsal tavan ve P ile uyumlu ivme/adim sinirlari uygulanir. P harmani yoktur; politika dogrudan motor adimi uretir.

**Ne sağlar:** Hızlı manevralarda taşıma gecikmesini eylem geçmişiyle telafi ederek kilidi korur; merkezde mikro titreşimi sönümler.

### 6.5 İletişim, telemetri ve arayüz

#### `protocol.py` — Ortak protokol sözleşmesi

PC tarafındaki `protocol.py` ile STM32 tarafındaki `protocol.h`, çerçeve uzunluğunu, başlık baytlarını, açı ölçeğini (0.1°) ve durum kodlarını tek bir sözleşmede tutar. Böylece bir taraftaki alan kayması checksum tutsa bile sessizce kabul edilmez.

**Ne sağlar:** Python ve C firmware arasındaki ikili protokolün tek kaynaktan üretilmesini ve tutarlı kalmasını sağlar.

#### `fcs_comm.py` — STM32 haberleşmesi

PC ile STM32 arasında 115200 Baud, tam çift yönlü, ikili paket protokolü kurar:

- **TX (PC → STM32), 7 bayt:** `0xAA`, pan ×10 (int16), tilt ×10 (int16), durum (uint8), checksum (uint8).
- **RX (STM32 → PC), 10 bayt:** `0x55`, mesafe (uint16), sinyal gücü (uint16), pan ×10 (int16), tilt ×10 (int16), checksum (uint8).

Port seçimi sırasıyla açıkça verilen port, ST-Link/USB VID `0483` ve tek USB seri port olarak yapılır. RX tarafında kalıcı tampon, 0x55 başlığına yeniden senkronizasyon ve checksum döngüsü vardır. 0.1° altı komutlar eksen bazlı artık biriktiriciyle korunur.

**Ne sağlar:** Gürültülü seri hatta paket bölünmesi veya bayt kaybı olsa bile komut ve telemetrinin güvenilir biçimde akmasını sağlar.

#### `telemetry.py` — Telemetri şeması 2.1

Her kare için 35 kolonlu CSV yazar: kare kimliği, zaman, dt, işlem gecikmesi, FPS, hedef güveni/kutusu, açısal hatalar, motor komutları, ham ve filtrelenmiş hız, Kalman inovasyonu, LiDAR verisi, FCS durumu, kilit sayacı, vuruş olasılığı, taret pozu, servo geri bildirimi ve devriye durumu. Oturum başında `meta.json` (donanım, `CONFIG`, sütun şeması, git commit), oturum sonunda özet (süre, FPS, tespit/kilit oranı); ayrıca `_events.csv` olay günlüğü üretir. Dosya tanıtıcıları açık tutulur, yazım 30 karelik tamponla yapılır.

**Ne sağlar:** Sahadaki her kararın ve sensör değerinin sonradan analizle incelenebilmesini sağlar.

#### `fcs_hud.py` ve `presenter.py` — Taktik arayüz

`fcs_hud.py` düşük seviyeli çizim ilkellerini (köşe kutusu, nişangah, kilit bari, LEAD oku, vuruş olasılığı, olay günlüğü, mod bannerı) sağlar. `presenter.py` bu ilkelleri doğru sırayla çağırır: hedef kutusu → takip çizgisi → LEAD → nişangah → telemetri → kilit göstergesi → banner/footer. E-stop, angajman ve arama durumlarında renk ve metinler değişir.

**Ne sağlar:** Operatörün tek bakışta hedef, kilit, menzil, mod ve atış iznini görmesini sağlar.

#### `patrol_generator.py` — Otonom arama

Hedef kaybolduğunda 1.5 saniye son konumu korur, ardından 20°/s hızla merkeze döner. Merkeze yaklaşınca 8 saniyelik, pan ±45° ve tilt ±16° genliğinde 1:2 frekanslı Lissajous figure-8 taramasına geçer. Merkeze dönüşte 0.95° birikimli adım eşiği kullanılır; böylece servo ölü bölgesinde takılma olmaz.

**Ne sağlar:** Hedef yokken hava sahasını sürekli, yumuşak ve mekanik olarak güvenli biçimde tarar.

#### `config.py` — Tek doğruluk kaynağı

Optik, kontrol, aktüatör, Kalman, balistik, RL, LiDAR, volan, montaj, kilit, hedef seçici, devriye ve HUD parametrelerini donmuş dataclass'lar halinde tutar. `FCSState` numaralandırması STM32 durum baytıyla birebir eşleşir.

**Ne sağlar:** Kod, test, Webots ve dokümantasyonun aynı sayısal değerleri kullanmasını sağlar; parametre sürüklenmesini önler.

### 6.6 RL eğitimi ve analiz araçları

#### `turret_fcs/rl/turret_env.py` — Gymnasium ortamı

Gerçek plant kısıtlarını modeller: 1.61° ölçülmüş dişli boşluğu, 0.10° servo ölü bölgesi, pan 1.50° / tilt 0.75° adım sınırları, 35-65 ms servo zaman sabiti, 1-3 kare taşıma gecikmesi, 25-45 ms FPS dalgalanması, C270 optik modeli, YOLO kutu gürültüsü ve harmonik jitter. Hedef yörüngeleri statik, doğrusal, zigzag, kaçış ve dairesel olarak rastgeleleştirilir. Gözlem 7 boyutlu gecikmeli sensör verisi + 4 boyutlu taze eylem geçmişi = 11 boyuttur.

**Ne sağlar:** Politikayı gerçek donanıma yakın bir plant üzerinde, gecikme ve gürültüye dayanıklı olarak eğitir.

#### `turret_fcs/rl/train_rl.py` — PPO eğitimi

250.000 adımlık PPO eğitimini çalıştırır; `models/` altına SB3 `.zip` modelini ve bağımsız NumPy `.npz` politika dosyasını üretir. `best_params.json` varsa `lambda_u`, `mu_du` ve öğrenme oranı oradan okunur. Ödül; hata azalışını pozitif, hata karesini, kontrol eforunu, eylem değişimini ve yön değiştirmeyi negatif ağırlıklandırır; bullseye ve sakinlik bonusu ekler.

**Ne sağlar:** RL modunun dağıtılan politikasını yeniden üretilebilir biçimde eğitir.

#### `turret_fcs/analysis/` ve `turret_basket/analysis/` — Doğrulama ve görselleştirme

| Kategori | Betik | Ne üretir? |
| :--- | :--- | :--- |
| **Karşılaştırma** | `turret_fcs/analysis/comparison/plot_tuned_system.py` | 20° basamak, faz portresi, Bode, dinamik takip, Lissajous, balistik ve kapalı çevrim kilit figürleri |
| **Karşılaştırma** | `turret_fcs/analysis/comparison/generate_benchmark_plots.py` | TurretEnv üzerinde P ve PPO RL karşılaştırma paneli |
| **Karşılaştırma** | `turret_fcs/analysis/comparison/generate_readme_diagrams.py` | Mimari, akış ve donanım SVG şemalarını üretir |
| **Karşılaştırma** | `turret_fcs/analysis/comparison/optimize_selector_evt.py` | EVT ile hedef seçici güven eşiği optimizasyonu |
| **Karşılaştırma** | `turret_fcs/analysis/comparison/optimize_ballistics_roc.py` | Vuruş olasılığı ROC / Youden J eşik optimizasyonu |
| **Sistem** | `turret_fcs/analysis/system/python/system_identification.py` | Sentetik FOPDT/SOPDT model uydurma ve kök-yer eğrisi |
| **Sistem** | `turret_fcs/analysis/system/python/system_identification_real.py` | Canlı telemetriden gerçek backlash ve gecikme kestirimi |
| **P kontrol** | `turret_fcs/analysis/controllers/p/optimize_p_controller.py` | 5 parametreli Tip-1 P Optuna TPE optimizasyonu |
| **P kontrol** | `turret_fcs/analysis/controllers/p/p_profile_sweep.py` | 6 hedef profili × 8 P kazancı Webots taraması |
| **RL** | `turret_fcs/analysis/controllers/rl/optimize_rl_wrapper.py` | RL sarmalayıcı taper ve yön değiştirme sönümü optimizasyonu |
| **RL** | `turret_fcs/analysis/controllers/rl/export_rl_policy.py` | SB3 `.zip` modelini bağımsız `.npz` NumPy MLP'ye aktarır |
| **RL** | `turret_fcs/analysis/controllers/rl/rl_diagnose.py` | Üretim RL yolu ile ham politikayı aynı tohumlarda karşılaştırır |
| **Kalman** | `turret_fcs/analysis/kalman/optimize_kalman.py` | Süreç/ölçüm kovaryansı ve ego zaman sabiti optimizasyonu |
| **Kalman** | `turret_fcs/analysis/kalman/generate_kalman_plots.py` | Kalman konum/hız kestirimi, inovasyon dağılımı ve doğrulama tablosu |
| **Telemetri** | `turret_fcs/analysis/logs/inspect_logs.py` | Son telemetri CSV'sinde kilit, hata, salınım ve kesintisiz kilit serisi raporu |
| **Telemetri** | `turret_fcs/analysis/logs/consolidate_telemetry_logs.py` | Parçalı logları birleştirme ve KPI özeti |
| **Devriye** | `turret_fcs/analysis/patrol/plot_patrol_log.py` | Lissajous devriye yörüngesi ve servo adımları |
| **Basket Analiz** | `turret_basket/analysis/plot_optuna_results.py` | Cup tracker Optuna TPE denemeleri karşılaştırma grafiği |

### 6.7 Webots dijital ikizi

| Modül | Ne işe yarar? |
| :--- | :--- |
| `turret_fcs.py` | Üretim `FCSManager`, `EngagementManager`, HUD ve telemetri kodunu Webots'ta çalıştırır |
| `webots_hardware_interface.py` | Motor, enkoder, kamera ve LiDAR cihazlarını üretim arayüzüne bağlar; 90.0 ms taşıma gecikmesi, 30.0 ms servo tau, 1.61° backlash ve 0.1° kuantalama uygular |
| `webots_flywheel_launcher.py` | Volan spin-up, boşta dönüş ve solenoid tetiklemesini simüle eder |
| `webots_virtual_ballistics.py` | Sanal mermi uçuşu ve namlu ofsetli isabet değerlendirmesi üretir |
| `webots_telemetry_logger.py` | Webots oturumunu CSV olarak kaydeder |
| `webots_tuning.py` | Yalnızca Webots için Kp ve maksimum çıkış aşırı yüklemesi yapar; üretim `CONFIG` değerini değiştirmez |

### 6.8 STM32 gömülü yazılımı

| Dosya | Ne işe yarar? |
| :--- | :--- |
| `Core/Inc/protocol.h` | PC-STM32 çerçeve uzunluğu, başlık, ölçek ve durum kodlarının C tarafındaki tek kaynağı |
| `Core/Src/main.c` | 100 Hz ana döngü, IWDG beslemesi, heartbeat, B1 mandallı E-stop, servo park ve durum yönlendirmesi |
| `Core/Src/fcs_comm.c` | 8 bayt komut ayrıştırma (hız dahil), 10 bayt telemetri, checksum, E-stop çift paket doğrulaması, kayan pencere yeniden senkronizasyonu |
| `Core/Src/servo_driver.c` | 50 Hz PWM üretimi, açı sınırları, zaman tabanlı 45°/s hız sınırı, yumuşak merkez dönüşü |
| `Core/Src/luna_read.c` | TF-Luna UART çerçeve ayrıştırma ve mesafe/sinyal güncellemesi |
| `Core/Src/flywheel_solenoid.c` | Volan enable ve 80 ms / 150 ms non-blocking solenoid durum makinesi |

---

## 7. Kontrol ve Kestirim Teorisi

### 7.1 Tip-1 P ve ayrık integratör

STM32 servo sürücüsü her komutu mevcut açıya ekler:

```text
theta(k) = theta(k-1) + delta_theta(k)
```

Bu, z-düzleminde `H(z) = z / (z - 1)` transfer fonksiyonuna sahip bir integratördür. Oransal kontrol `delta_theta(k) = Kp * e(k)` ile birleştiğinde sistem Tip-1 olur; basamak hedefte kalıcı durum hatası ölü bölge bandı içinde kalır. Bu nedenle harici Ki integrali kullanılmaz. Hareketli hedefte kalan rampa hatası, Kalman lead'i ve atış anındaki slew ileri beslemesiyle karşılanır.

### 7.2 Yumuşak ölü bölge ve ivme sınırı

```text
u = 0,                               |e| <= db
u = sign(e) * Kp * (|e| - db),       |e| > db
```

Ölü bölgeden çıkışta komut sıfırdan başlar; basamak darbesi oluşmaz. Komut ayrıca ivme sınırına tabidir:

```text
u(k) = clip(u_desired, u(k-1) - a * dt^2, u(k-1) + a * dt^2)
```

Pan için `a = 556.58°/s²`, tilt için `a = 211.81°/s²` kullanılır. Bu, sabit bir hız tavanı koymadan kalkış ve yön değişimlerini yumuşatır; asimetrik frenleme (`asymmetric_braking`) ile hedefe yaklaşırken sıfır aşım sağlar.

### 7.3 CV Kalman ve ego-hareket telafisi

Durum `X = [x, y, vx, vy]^T`, ölçüm `Z = [x, y]^T` şeklindedir. Kestirim adımı:

```text
X_pred = F * X + ego_delta
P_pred = F * P * F^T + Q
```

Ego kayması, son motor komutunun birinci derece servo yanıtından üretilir:

```text
alpha_m = dt / (tau + dt)
ego_step = (1 - alpha_m) * ego_step + alpha_m * last_cmd
```

`tau = 0.0300 s` değerindedir. Böylece taretin kendi dönüşü hedef hızına karışmaz. Ölçüm güncellemesi Joseph formuyla yapılır.

100 px üstü inovasyon normunda veya %99.9 Ki-kare inovasyon kapısında (`chi2 > 13.8155`) ölçüm reddedilir; durum yalnızca `F` ile ilerletilir. Aynı yeni bölgede 3 tutarlı ölçüm oluşursa ilk aday kareden başlanarak tüm adaylar ileri sarılır (`predict + correct`), böylece yeni hedefin hızı sıfırdan öğrenilmez.

### 7.4 Lead ve balistik süper-elevasyon

```text
t_flight = clip(d / v0 + t_offset, 0.03 s, 0.40 s)
lead_x = vx * t_flight
lead_y = vy * t_flight
theta_gravity = arctan((z_drop + camera_to_muzzle_z) / d)
```

`v0 = 10.0 m/s`, `t_offset = 0.090 s`, `g = 9.80665 m/s²`, `camera_to_muzzle_z = 0.0570 m` (CAD v10 namlu paralaksı) değerlerindedir. Aerodinamik sürükleme (`drag_enabled: bool = True`, m = 1.2 g dart, Cd = 0.60) modeliyle düşüş (`z_drop`) hesaplanır. Hedefin ölçülen güncel noktası lead kadar ileri taşınır; filtrenin konum gecikmesi ikinci kez eklenmez.

### 7.5 Vuruş olasılığı

Hedefin ölümcül açısal yarıçapı `theta_target_lethal`, toplam belirsizlik `sigma_total` olmak üzere:

```text
theta_target_lethal = 0.60 * theta_target
sigma_total^2 = sigma_weapon^2 + (0.35 * omega * t_total)^2
nc = (angular_error / sigma_total)^2
x_cutoff = (theta_target_lethal / sigma_total)^2
p_ballistic = ncx2.cdf(x_cutoff, df=2, nc=nc)
p_hit = 100 * p_ballistic * s_tracking * s_gate
```

`sigma_weapon = 1.25°`, `t_total = t_flight + 0.0900 s` sistem gecikmesidir. `s_tracking`, kilit oturana kadar 0.15-1.0 arasında ilerler; `s_gate`, hata dinamik kapı dışına çıktığında üstel olarak söner. Atış için minimum vuruş olasılığı %25'tir.

### 7.6 PPO RL formülasyonu

Gözlem:

```text
[err_x, err_y, d_err_x, d_err_y, vx, vy, dist,
 act_pan_t-1, act_tilt_t-1, act_pan_t-2, act_tilt_t-2]
```

Ödül:

```text
r = 2.0 * (e_prev^2 - e^2)
    - 0.05 * e^2
    - 0.0433 * |u|^2
    - 0.0338 * |du|^2
    - reversal_penalty
    + bullseye
    + alive_bonus
```

Hata azalışı pozitif; hata karesi, kontrol eforu, eylem değişimi ve yön değişimi negatif ağırlıklıdır. Kilit yakınında efor/jerk cezası 1.8 katına çıkar. `bullseye`, hata 0.05 normalize birimin altına indiğinde `0.50 * calm_factor` olarak verilir; `alive_bonus` merkeze yakın sakin duruşu ödüllendirir. Eğitimde 1.61° backlash, 0.10° servo ölü bölgesi, pan 1.50° / tilt 0.75° adım sınırı, 1-3 kare gecikme ve 2-5 px YOLO gürültüsü kullanılır.

---

## 8. Güvenlik ve Fail-Safe Mimarisi

| Katman | Koruma | Davranış |
| :--- | :--- | :--- |
| **STM32 donanım** | Bağımsız IWDG, nom. 800 ms | Firmware kilitlenirse MCU reset atar; pinler tekrar 0 V'a çekilir |
| **STM32 yazılım** | 300 ms heartbeat | PC komutu kesilirse volan ve solenoid anında kapatılır |
| **STM32 yazılım** | 2000 ms servo park | PC kalıcı olarak yoksa servolar 45°/s ile merkeze indirilir |
| **STM32 protokol** | E-stop çift paket | E-stop yalnızca üst üste iki geçerli pakette kabul edilir |
| **STM32 UART** | Kayan pencere | Checksum hatasında tampon bir sonraki `0xAA` başlığına kaydırılır; geçerli paket kaybı önlenir |
| **STM32 buton** | B1 mandallı E-stop | Butona basınca PC komutları yok sayılır, launcher kesilir, servolar parka alınır; tekrar basınca açılır |
| **STM32 servo** | Zaman tabanlı 45°/s | PC FPS'i artsa bile taret hızı sabit kalır |
| **STM32 servo** | Açı limitleri | Pan 15° - 165°, tilt 65° - 115° yazılımsal olarak sınırlanır |
| **STM32 ateşleme** | Volan hazır kapısı | Volan enerjili değilse `Launcher_TriggerFire()` atışı reddeder |
| **STM32 ateşleme** | 80 ms / 150 ms zamanlayıcı | Çift dart ve solenoid aşırı ısınması engellenir |
| **Elektriksel** | Pull-down + izolasyon | Açılış/reset anında fırlatıcı hatları 0 V; solenoid optokuplörle izole |
| **PC** | Mandallı E-stop | `[Space]` motorları merkeze indirir ve tüm komutları kilitler |
| **PC** | Vuruş olasılığı kapısı | Vuruş olasılığı %25 altındaysa `[Y]` atışı reddeder |
| **PC** | Kamera bayat kare | 350 ms yeni kare gelmezse döngü güvenli biçimde durur |
| **PC** | Fiziksel ateşleme izolasyonu | `ENABLE_PHYSICAL_FIRE = 0` iken solenoid enerjilenmez |

### Çok katmanlı durdurma felsefesi

1. **Yazılımsal kapı:** Kilit, vuruş olasılığı, menzil ve sekans uygun değilse atış başlamaz.
2. **Gömülü kapı:** STM32, volan hazır değilse veya heartbeat yoksa atışı reddeder.
3. **Elektriksel kapı:** Pull-down ve optokuplör, MCU hatasında bile fırlatıcıyı pasif tutar.
4. **Operatör kapı:** Fiziksel atış varsayılan olarak kapalıdır; laboratuvar testinde bilinçli olarak açılır.

---

## 9. Performans ve Doğrulama

### 9.1 Gerçek donanım telemetrisi

Aşağıdaki değerler, P ve RL modlarının birlikte kullanıldığı 4.383 karelik saha oturumundan (`logs/telemetry_test_20261025_054723.csv`) hesaplanmıştır. Oturum 146.06 saniye, ortalama 37.9 FPS'dir.

| Metrik | Oturum toplamı |
| :--- | :---: |
| Kare sayısı | 4.383 |
| Hedef tespit oranı | %76.2 (3.338 kare) |
| Genel kilit oranı | %42.3 |
| Tespit sırasında kilit oranı | %55.5 |
| Ortalama vuruş olasılığı | %71.5 |
| En uzun kesintisiz kilit | 322 kare (10.71 s) |

### 9.2 Mod bazında karşılaştırma

| Metrik | P Kontrolcü | RL Kontrolcü |
| :--- | ---: | ---: |
| Kare sayısı | 2.616 | 1.767 |
| Hedef tespit karesi | 2.074 | 1.264 |
| Kilit oranı (tespit sırasında) | %52.1 | %59.7 |
| Pan RMS hatası | 3.66° | 3.37° |
| Tilt RMS hatası | 3.31° | 2.14° |
| Bileşke RMS hatası | 4.94° | 3.99° |
| Ortalama mutlak hata | 3.50° | 2.71° |
| Medyan hata | 1.96° | 1.71° |
| En uzun kesintisiz kilit | 322 kare (10.71 s) | 137 kare (4.55 s) |
| Pan komut yön değişimi | %0.19 | %0.55 |

> Bu tablo aynı saha oturumunun P ve RL bölümlerini gösterir; hedef profilleri, operatör müdahaleleri ve angajman anları bölümler arasında aynı değildir. Kontrollü kontrolcü karşılaştırması için `turret_fcs/analysis/comparison/generate_benchmark_plots.py` kullanılmalıdır.

### 9.3 P ve PPO RL benzetim karşılaştırması

![P Kontrolcü vs PPO RL Benchmark](turret_fcs/analysis/figures/python/python_fig11_p_vs_rl_final_benchmark.png)

Benzetim, 1-3 kare arasinda rastgelelestirilmis tasima gecikmesi ve dinamik eyleyici lag'i altinda dort tohumla calistirilir. Guncel figurde FCS kilit kapisi (<= 1.70 deg) altinda P kontrolcu %36.8, PPO RL %50.2 kilit orani uretir (RL %13.4 onde). Genis takip kapisinda (<= 2.95 deg) basari P icin %60.4, RL icin %64.0'tur. Pan RMS hatasi P icin 4.02 deg, RL icin 4.06 deg; tilt RMS hatasi P icin 1.97 deg, RL icin 1.74 deg'dir (RL tilt ekseninde %12.0 iyilesme saglar).

### 9.4 Kontrol ve kararlılık figürleri (Python & MATLAB)

Tüm analiz grafikleri `turret_fcs/analysis/figures/` altında `python/` ve `matlab/` olarak gruplanmıştır.

#### Python doğrulama figürleri (`turret_fcs/analysis/figures/python/`)

| Figür | Gösterdiği |
| :--- | :--- |
| python_fig1_tuned_step_and_control_effort.png | 20 deg basamak yaniti; 0.49 s yerlesme, %11.1 asim, ivme sinirli kalkis ve asimetrik frenleme |
| `python_fig2_tuned_phase_plane_portrait.png` | Faz düzlemi; yumuşak ölü bölgeye asimptotik yakınlaşma |
| python_fig3_tuned_bode_stability.png | Acik cevrim Bode; 6.97 rad/s gecis, 42.3 deg faz marji, 6.0 dB kazanc marji |
| `python_fig4_tuned_dynamic_tracking.png` | Sinüzoidal hedefte pan/tilt dinamik takip ve hata |
| `python_fig5_tuned_lissajous_coverage.png` | Figure-8 devriye yörüngesi ve 3B tarama hacmi |
| `python_fig6_tuned_ballistic_characteristics.png` | Mesafeye göre uçuş süresi ve süper-elevasyon |
| `python_fig7_sim_still_lock.png` | Sabit hedefte kapalı çevrim kilit ve motor sessizliği |
| `python_fig8_sim_moving_tracking.png` | Manevralı hedefte takip ve kilit kararlılığı |
| `python_fig9_system_identification.png` | Sentetik FOPDT/SOPDT sistem tanımlama ve kök-yer eğrisi |
| `python_fig10_kalman_tracking_performance.png` | Kalman konum/hız doğruluğu, inovasyon dağılımı ve KPI tablosu |
| `python_fig11_p_vs_rl_final_benchmark.png` | Kapalı çevrim TurretEnv üzerinde P ve PPO RL karşılaştırması |
| `python_fig12_selector_evt_optimization.png` | EVT ile hedef güven skoru analizi ve eşik tespiti |
| `python_fig13_system_id_real_hardware.png` | Gerçek telemetriden 1.61° dişli boşluğu ve 66.7 ms gecikme kestirimi |
| `python_fig14_ballistics_roc_optimization.png` | Vuruş olasılığı ROC eğrisi ve Youden J optimal kesme noktası |
| `python_fig15_patrol_kinematic_telemetry.png` | Kinematik devriye telemetrisi ve yumuşak ivmelenme analizi |
| `python_fig16_robustness_monte_carlo.png` | Monte Carlo gecikme ve ölçüm gürültüsü dayanıklılık analizi |
| `python_fig17_pipeline_latency_breakdown.png` | Uçtan uca boru hattı gecikme dağılımı (Kamera, YOLO, FCS, STM32) |
| `python_fig18_real_tau_comparison.png` | Gerçek donanım servo zaman sabiti (tau) ve frekans cevabı karşılaştırması |

#### MATLAB & Simulink figürleri (`turret_fcs/analysis/figures/matlab/`)

| Figür | Gösterdiği |
| :--- | :--- |
| `matlab_fig1_system_id_result.png` | FOPDT/SOPDT sistem tanımlama ve ölçüm uyumu |
| `matlab_fig2_stability_root_locus.png` | S-düzlemi kök-yer eğrisi ve kritik kazanç |
| `matlab_fig3_stability_bode_nyquist.png` | Bode kararlılık payları ve Nyquist kritik nokta analizi |
| `matlab_fig4_stability_step_response.png` | Kapalı çevrim adım tepkisi ve yumuşak ölü bölge davranışı |
| `matlab_fig5_simulink_simulation_result.png` | Simulink dijital ikiz kapalı çevrim doğrulaması |
| `matlab_fig6_hil_test_result.png` | Hardware-in-the-Loop canlı STM32 takip testi |

![20 Derece Basamak Yanıtı](turret_fcs/analysis/figures/python/python_fig1_tuned_step_and_control_effort.png)

![Faz Düzlemi Portresi](turret_fcs/analysis/figures/python/python_fig2_tuned_phase_plane_portrait.png)

![Bode Kararlılık Analizi](turret_fcs/analysis/figures/python/python_fig3_tuned_bode_stability.png)

![Dinamik Takip](turret_fcs/analysis/figures/python/python_fig4_tuned_dynamic_tracking.png)

![Lissajous Devriye](turret_fcs/analysis/figures/python/python_fig5_tuned_lissajous_coverage.png)

![Balistik Karakteristik](turret_fcs/analysis/figures/python/python_fig6_tuned_ballistic_characteristics.png)

![Sabit Hedef Kilit Kararlılığı](turret_fcs/analysis/figures/python/python_fig7_sim_still_lock.png)

![Manevralı Hedef Takibi](turret_fcs/analysis/figures/python/python_fig8_sim_moving_tracking.png)

![Sistem Tanımlama](turret_fcs/analysis/figures/python/python_fig9_system_identification.png)

![Kalman Doğrulama](turret_fcs/analysis/figures/python/python_fig10_kalman_tracking_performance.png)

![P vs RL Final Benchmark](turret_fcs/analysis/figures/python/python_fig11_p_vs_rl_final_benchmark.png)

![Hedef Güven Eşiği EVT Optimizasyonu](turret_fcs/analysis/figures/python/python_fig12_selector_evt_optimization.png)

![Gerçek Donanım Sistem Tanılaması](turret_fcs/analysis/figures/python/python_fig13_system_id_real_hardware.png)

![Balistik ROC Eğrisi](turret_fcs/analysis/figures/python/python_fig14_ballistics_roc_optimization.png)

![Kinematik Devriye Telemetrisi](turret_fcs/analysis/figures/python/python_fig15_patrol_kinematic_telemetry.png)

![Monte Carlo Dayanıklılık Analizi](turret_fcs/analysis/figures/python/python_fig16_robustness_monte_carlo.png)

![Boru Hattı Gecikme Dağılımı](turret_fcs/analysis/figures/python/python_fig17_pipeline_latency_breakdown.png)

![Gerçek Zaman Sabiti Karşılaştırması](turret_fcs/analysis/figures/python/python_fig18_real_tau_comparison.png)

![MATLAB Sistem Tanımlama](turret_fcs/analysis/figures/matlab/matlab_fig1_system_id_result.png)

![MATLAB Kök-Yer Eğrisi](turret_fcs/analysis/figures/matlab/matlab_fig2_stability_root_locus.png)

![MATLAB Bode ve Nyquist Analizi](turret_fcs/analysis/figures/matlab/matlab_fig3_stability_bode_nyquist.png)

![MATLAB Adım Tepkisi](turret_fcs/analysis/figures/matlab/matlab_fig4_stability_step_response.png)

![MATLAB Simulink Dijital İkiz Doğrulaması](turret_fcs/analysis/figures/matlab/matlab_fig5_simulink_simulation_result.png)

![MATLAB HIL Testi](turret_fcs/analysis/figures/matlab/matlab_fig6_hil_test_result.png)

---

## 10. Hızlı Başlangıç

### 10.1 Gereksinimler

- Windows 10/11 x64
- Python 3.10, 3.11 veya 3.12
- STM32F401RE Nucleo-64 ve ST-Link
- Logitech C270 (veya uyumlu UVC kamera)
- Benewake TF-Luna ToF LiDAR
- 12 V adaptör, XL4015 6 V buck, servo/volan/solenoid güç katı

### 10.2 Otomatik kurulum

```bash
git clone https://github.com/brkaysenturk/turret-fcs.git
cd turret-fcs
setup_env.bat
```

`setup_env.bat` sırasıyla uygun Python'u seçer, gerekirse `venv` ortamını yeniden kurar, bağımlılıkları CPU-only PyTorch indeksiyle yükler, paketi `pip install --no-deps -e .` ile bağlar ve temel importları doğrular.

### 10.3 Testler

```bash
venv\Scripts\python.exe -m pytest -q
```

51 test; P kontrolcü, Kalman aday geri oynatma, kilit/vuruş olasılığı, LiDAR kapılama, RL sarmalayıcı, Webots tuning, ağırlık yöneticisi, `CONFIG` tutarlılığı ve `turret_basket` (RK4 balistik, RLTracker, RLBallistics, BasketEngagement, BasketManager, UART comm, HUD UI ve FSM) sözleşmelerini donanım olmadan doğrular.

### 10.4 Sistemi başlatma

**Hava Savunma Tareti (Turret FCS):**

```bash
turret_fcs\main.bat
```

veya:

```bash
venv\Scripts\python.exe -m turret_fcs.main
```

**Bardak Basketi Görevi (Cup Basket Challenge):**

```bash
turret_basket\main.bat
```

veya:

```bash
venv\Scripts\python.exe -m turret_basket.main
```

İlk çalıştırmada `weights/best.pt` ve `weights/yolov8n.pt` yoksa GitHub Releases üzerinden otomatik indirilir. STM32 bulunamazsa sistem çevrimdışı devam eder; kamera bulunamazsa HUD uyarısı verilir ve görüntü döngüsü başlamaz.

### 10.5 Webots dijital ikizi

1. Webots R2025a ile `webots/worlds/turret_fcs.wbt` (Hava Savunma) veya `webots/worlds/turret_basket.wbt` (Bardak Basketi) dosyasını açın.
2. Başlangıç modunu seçin:

   ```text
   WEBOTS_CONTROL_MODE=P     # hassas kilit
   WEBOTS_CONTROL_MODE=RL    # çevik takip
   ```

3. Hedef profilini `webots/balloon_profile.json` içindeki `active_profile` ile veya `WEBOTS_BALLOON_PROFILE` ortam değişkeniyle seçin (0-7).
4. Gözlem gecikmesini `WEBOTS_OBS_DELAY_S` (varsayılan 0.096 s / 3 kare), kutu titremesini `WEBOTS_BBOX_JITTER` (varsayılan 3.0 px) ile ayarlayabilirsiniz.
5. P kazancı için `webots/tuning_params.json` kullanılır; bu dosya üretim `CONFIG` değerini değiştirmez.

### 10.6 Model ağırlıkları

```bash
venv\Scripts\python.exe -m turret_fcs.weights
```

Komut `best.pt` ve `yolov8n.pt` dosyalarını SHA256 doğrulamasıyla `weights/` altına indirir.

---

## 11. Operatör Kullanımı

### 11.1 Tuşlar

| Tuş | İşlev | Davranış |
| :---: | :--- | :--- |
| `ESC` / `Q` | Güvenli çıkış | Telemetriyi kapatır, dosyaları yazar, portları serbest bırakır |
| `SPACE` | Mandallı E-stop | Devriyeyi kapatır, FCS'yi sıfırlar, motorları merkeze indirir; tekrar basılınca devre dışı kalır |
| `M` | Mod değişimi | P (hassas kilit) ↔ RL (çevik takip) |
| `P` | Devriye | Figure-8 arama modunu açar/kapatır |
| `Y` | Atış onayı | Kilit, menzil ve minimum %25 vuruş olasılığı sağlanınca angajman başlatır |
| `N` | Kilidi bırak | FCS'yi sıfırlar, angajmanı iptal eder, aramaya döner |

### 11.2 Tipik oturum

1. `turret_fcs\main.bat` ile sistemi açın; HUD'de kamera, STM32, LiDAR ve telemetri durumunu kontrol edin.
2. `P` ile devriye taramasını açın veya hedefi manuel olarak kadraja alın.
3. Hedef kutuda belirdiğinde HUD'de takip çizgisi, LEAD ibresi ve vuruş olasılığı görünür.
4. Kilit barı dolduğunda `TARGET LOCKED` mesajı gelir; `[Y]` ile atışı onaylayın.
5. Atış sonrası EVAL aşaması tamamlanana kadar bekleyin; gerekirse `N` ile hedefi bırakın.
6. Acil durumda `SPACE`; çıkışta `Q`.

### 11.3 HUD'de okunacaklar

- **Sol üst:** durum metni, LiDAR mesafesi, sinyal gücü ve açısal hata.
- **Sağ üst:** FPS göstergesi (kilit/atış anında gizlenir).
- **Üst orta:** `[P CONTROLLER]` veya `[RL CONTROLLER]` bannerı ve devriye durumu.
- **Merkez:** nişangah, kilit barı, `HIT PROBABILITY: %xx`.
- **Hedef üzerinde:** `TARGET` köşe kutusu, LEAD noktası ve takip çizgisi.
- **Alt:** operatör komut lejandı ve olay günlüğü.

---

## 12. Telemetri ve Analiz

### 12.1 Üretilen dosyalar

| Dosya | İçerik |
| :--- | :--- |
| `logs/telemetry_test_YYYYMMDD_HHMMSS.csv` | 35 kolonlu kare bazlı telemetri (şema 2.1) |
| `logs/telemetry_test_YYYYMMDD_HHMMSS.meta.json` | Donanım, `CONFIG`, sütun şeması, git commit ve oturum özeti |
| `logs/telemetry_test_YYYYMMDD_HHMMSS_events.csv` | Zaman damgalı taktiksel olaylar |

### 12.2 Analiz komutları

```bash
# Son telemetri kaydını incele
venv\Scripts\python.exe turret_fcs\analysis\logs\inspect_logs.py

# Belirli bir kaydı incele
venv\Scripts\python.exe turret_fcs\analysis\logs\inspect_logs.py logs\telemetry_test_20261025_054723.csv

# Kontrol ve kararlılık figürlerini üret
venv\Scripts\python.exe turret_fcs\analysis\comparison\plot_tuned_system.py

# Sistem tanımlama figürünü üret
venv\Scripts\python.exe turret_fcs\analysis\system\python\system_identification.py

# Kalman doğrulama figürünü üret
venv\Scripts\python.exe turret_fcs\analysis\kalman\generate_kalman_plots.py

# P vs RL benzetim benchmark'ını üret
venv\Scripts\python.exe turret_fcs\analysis\comparison\generate_benchmark_plots.py

# Mimari, akış ve donanım SVG şemalarını üret
venv\Scripts\python.exe turret_fcs\analysis\comparison\generate_readme_diagrams.py
```

### 12.3 RL politikasını yeniden eğitme

```bash
# 250.000 adımlık PPO eğitimi
venv\Scripts\python.exe turret_fcs\rl\train_rl.py

# SB3 modelini bağımsız NumPy MLP'ye aktar
venv\Scripts\python.exe turret_fcs\analysis\controllers\rl\export_rl_policy.py
```

Eğitim çıktısı `turret_fcs/rl/models/` altına yazılır. Üretim çıkarımı `.npz` dosyasını kullanır; PyTorch veya Stable-Baselines3 kurulu olmasa da RL modu çalışır.

---

## 13. Dizin Yapısı

```text
turret-multimission/
├── .github/workflows/ci.yml          # Python 3.10-3.12 test ve ruff matrisi
├── logs/                             # Telemetri CSV, meta.json ve olay günlükleri
├── tests/                            # 51 Pytest birim/entegrasyon testi
├── turret_fcs/                       # Görev 1: Hava Savunma Atış Kontrol Sistemi
│   ├── main.bat                      # Gerçek zamanlı FCS başlatıcı
│   ├── config.py                     # FCS SSOT konfigürasyonu
│   ├── main.py                       # Ana çalışma döngüsü
│   ├── fcs_manager.py                # FCS karar çekirdeği
│   ├── p_controller.py               # Tip-1 P kontrolcü
│   ├── rl_controller.py              # PPO RL kontrolcü
│   ├── kalman_filter.py              # CV Kalman + aday geri oynatma
│   ├── lock_manager.py               # Dinamik kilit + vuruş olasılığı
│   ├── engagement.py                 # 5 aşamalı angajman FSM
│   ├── target_selector.py            # Hedef seçimi ve önceliklendirme
│   ├── camera_thread.py              # Sıfır gecikmeli kamera thread'i
│   ├── fcs_comm.py                   # STM32 115200 Baud ikili protokol
│   ├── protocol.py                   # PC-STM32 ortak sözleşme
│   ├── fcs_hud.py                    # MIL-STD taktik HUD
│   ├── presenter.py                  # HUD sunum katmanı
│   ├── telemetry.py                  # Telemetri şeması 2.1
│   ├── patrol_generator.py           # Figure-8 devriye yörüngesi
│   ├── weights.py                    # YOLO ağırlık yöneticisi
│   ├── analysis/                     # Doğrulama, benchmark, log ve figür araçları
│   ├── docs/                         # Dokümantasyon, CAD ve varlıklar
│   ├── rl/                           # Gymnasium ortamı, eğitim motoru ve modeller
│   ├── stm32/                        # STM32F401RE CubeIDE firmware projesi
│   └── yolo/                         # Veri toplama ve etiketleme araçları
├── turret_basket/                    # Görev 2: Bardak İçi Dart Basket Görevi (Cup Basket Challenge)
│   ├── main.bat                      # Basket görevi başlatıcı
│   ├── config.py                     # Basket SSOT konfigürasyonu
│   ├── main.py                       # Basket ana döngüsü
│   ├── basket_manager.py             # Basket karar çekirdeği
│   ├── rl_tracker.py                 # Cup Mouth Tracker (1. aşama yapay zeka)
│   ├── rl_ballistics.py              # Neural & RK4 Balistik süper-elevasyon (2. aşama)
│   ├── cup_tracker.py                # Bardak takip arayüzü
│   ├── engagement.py                 # Basket atış FSM
│   ├── basket_comm.py                # Basket STM32 UART iletişimi
│   ├── basket_hud.py                 # Basket taktik HUD
│   ├── presenter.py                  # Basket sunum katmanı
│   ├── analysis/                     # Optuna TPE analiz ve görselleştirme araçları
│   ├── docs/                         # Basket dokümantasyonu (README, firmware, reproducibility)
│   ├── rl/                           # Kupa Gymnasium ortamı ve modeller
│   ├── stm32/                        # Basket için optimize edilmiş STM32 firmware
│   └── yolo/                         # Bardak tespit modeli (CupDetector)
├── webots/                           # Webots R2025a Dijital İkiz
│   ├── worlds/
│   │   ├── turret_fcs.wbt            # Hava savunma simülasyon dünyası
│   │   └── turret_basket.wbt         # Bardak basketi simülasyon dünyası
│   ├── controllers/
│   │   ├── webots_turret_fcs/        # Hava savunma dijital ikiz denetleyicisi
│   │   ├── webots_turret_basket/     # Bardak basketi temas fiziği denetleyicisi
│   │   └── balloon_mover/            # Dinamik hedef balon hareketlendiricisi
│   ├── balloon_profile.json          # 8 hedef manevra profili
│   └── tuning_params.json            # Webots-only P kazanç ayarları
├── weights/                          # YOLO model ağırlıkları (best.pt, yolov8n.pt)
├── setup_env.bat                     # Sanal ortam ve paket kurulumu
├── pyproject.toml                    # Paket ve pytest yapılandırması
├── CITATION.cff                      # Akademik atıf üstverisi
└── LICENSE                           # MIT
```
---

## 14. Dokümantasyon

- [FCS Firmware kılavuzu](turret_fcs/docs/firmware.md): STM32CubeIDE kurulumu, pinout, flaşlama ve güvenlik bekçileri.
- [FCS Tekrar üretilebilirlik kılavuzu](turret_fcs/docs/reproducibility.md): testler, RL eğitimi, benchmark ve figür üretimi.
- [FCS Analiz araçları](turret_fcs/analysis/README.md): betikler, dizin hiyerarşisi ve ürettikleri çıktılar.
- [Basket Görevi kılavuzu](turret_basket/docs/README.md): Cup Basket Challenge görevi mimarisi ve çalıştırma.
- [Basket Firmware kılavuzu](turret_basket/docs/firmware.md): Basket STM32 yapılandırması ve pinout.
- [Basket Tekrar üretilebilirlik kılavuzu](turret_basket/docs/reproducibility.md): Basket ortamı, Optuna TPE eğitimi ve doğrulama.


---

## 15. Lisans, Atıf ve Güvenlik

Bu proje [MIT Lisansı](LICENSE) kapsamında lisanslanmıştır. Yıldız Teknik Üniversitesi Kontrol ve Otomasyon Mühendisliği Bölümü Bitirme Çalışması ve Tasarım Projesi kapsamında geliştirilmiştir.

```bibtex
@misc{turret_fcs_2026,
  author = {Senturk, Berkay},
  title = {Autonomous Air Defense Turret Fire Control System (Turret FCS)},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/brkaysenturk/turret-fcs}}
}
```

### Güvenlik ve etik kullanım

- Sistem yalnızca akademik araştırma, kontrol teorisi doğrulaması ve güvenli sünger dart hedefleri için tasarlanmıştır.
- Gömülü yazılımda fiziksel ateşleme varsayılan olarak kapalıdır (`ENABLE_PHYSICAL_FIRE = 0`).
- İnsanlara, canlı hedeflere veya yetkisiz nesnelere karşı kullanılamaz.
- Donanım testleri E-stop, 300 ms heartbeat ve 800 ms IWDG koruması altında yapılmalıdır.
- Sistem bir güvenlik sertifikasyonu değildir; askeri/operasyonel kullanım için ayrı bir emniyet ve uygunluk süreci gerekir.
