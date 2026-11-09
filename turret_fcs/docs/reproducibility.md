# Tekrar Üretilebilirlik Kılavuzu (Reproducibility Guide)

Bu doküman; Turret FCS projesinin testlerinin, pekiştirmeli öğrenme model eğitiminin (RL training), simülasyon benchmark'larının ve akademik analiz figürlerinin bağımsız araştırmacılar tarafından sıfırdan nasıl tekrar üretilebileceğini adım adım açıklamaktadır.

---

## 1. Ortam Kurulumu ve Bağımlılıklar

Proje Python 3.10, 3.11 ve 3.12 sürümleri ile test edilmiştir.

```bash
# Depoyu klonlayın
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

Donanıma (kamera veya STM32) ihtiyaç duymadan çalışan 52 birim, entegrasyon, ağırlık ve SSOT tutarlılık testi:

```bash
pytest -v
```

Bu komut; Tip-1 P kontrolcünün adım sınırlarını, CV Kalman filtresinin kovaryans kararlılığını, 2B Gauss kilit olasılığını, menzil kapılama histerezisini, kesintisiz devriye ve sahte tespit bağışıklığını, model ağırlık yöneticisini, `turret_basket` görev modüllerini ve `CONFIG` parametrelerinin tüm kod tabanında tutarlı şekilde okunduğunu doğrular.

---

## 3. Model Ağırlıklarının İndirilmesi

Projede kullanılan derin öğrenme modelleri GitHub Releases üzerinden dağıtılır:

```bash
# Model ağırlıklarını (best.pt ve yolov8n.pt) otomatik indirmek ve SHA256 doğrulaması yapmak için:
python -m turret_fcs.weights
```

Dosyalar `weights/` dizini altına kaydedilir.

---

## 4. PPO RL Modelinin Yeniden Eğitilmesi

9-boyutlu Gymnasium taret simülasyon ortamında (`TurretEnv`) sıfırdan model eğitmek için:

```bash
# 250.000 adımlık PPO eğitimini başlatın
python turret_fcs/rl/train_rl.py
```

Eğitim parametreleri:

- **Algoritma:** PPO (MlpPolicy: [64, 64])
- **Öğrenme Oranı (Learning Rate):** 3e-4
- **Gamma:** 0.99
- **GAE Lambda:** 0.95
- **Clip Range:** 0.2
- **Adım Sayısı:** 250.000 adım

Eğitim tamamlandığında üretilen model `turret_fcs/rl/models/rl_v2-1.zip` olarak kaydedilir. Bu modeli saf NumPy MLP (`.npz`) olarak dışa aktarmak için:

```bash
python turret_fcs/analysis/controllers/rl/export_rl_policy.py
```

---

## 5. P Kontrolcü vs PPO RL Benchmark Karşılaştırması

İki kontrolcünün dinamik hedef yörüngesi üzerindeki başarımını karşılaştırmak ve 5 panelli akademik benchmark grafiğini (`turret_fcs/analysis/figures/python/python_fig11_p_vs_rl_final_benchmark.png`) üretmek için:

```bash
python turret_fcs/analysis/comparison/generate_benchmark_plots.py
```

Bu betik; TurretEnv üzerinde Zikzak, Kaçış ve Doğrusal Kaçış senaryolarında her iki kontrolcüyü aynı tohumlarla koşturur; pan/tilt hata zaman serilerini, kontrol eforunu, hata CDF dağılımını ve akademik metrik tablosunu üretir.

---

## 6. Akademik Analiz Figürlerinin Üretilmesi

Tüm analitik modelleri, sistem tanıma modellerini, Kalman kestirim doğrulamalarını ve faz uzayı grafiklerini üretmek için:

```bash
# 1. 8 Panelli Dinamik Kararlılık Figürleri (python_fig1 - python_fig8)
python turret_fcs/analysis/comparison/plot_tuned_system.py

# 2. Sistem Tanıma ve Kök-Yer Eğrisi Figürü (python_fig9)
python turret_fcs/analysis/system/python/system_identification.py

# 3. Kalman Filtresi Hız ve Zemin Gerçeği Karşılaştırma Figürü (python_fig10)
python turret_fcs/analysis/kalman/generate_kalman_plots.py

# 4. P vs RL Benchmark Görseli (python_fig11)
python turret_fcs/analysis/comparison/generate_benchmark_plots.py

# 5. Mimari, Akış ve Donanım SVG Şemaları
python turret_fcs/analysis/comparison/generate_readme_diagrams.py
```
