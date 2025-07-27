# 2D Otonom Robot Simülasyonu - A* Yol Planlama Projesi

## 📋 Proje Özeti
Bu proje, ROS 2 tabanlı bir simülasyon ortamında, 2D bir labirent içerisinde A* algoritması kullanarak yol planlama yapan ve bu yolu takip eden otonom bir robot uygulamasıdır.

## 📝 Proje Durumu ve Yapılacaklar Listesi

### 1. Proje Altyapısı Kurulumu
- [x] ROS 2 workspace oluşturma
- [x] Gerekli paketlerin kurulumu
  - [x] geometry_msgs
  - [x] nav_msgs
  - [x] tf2_ros
- [x] Proje paket yapısının oluşturulması
  - [x] CMakeLists.txt düzenleme
  - [x] package.xml hazırlama
  - [x] Klasör yapısının oluşturulması

### 2. Harita ve Görselleştirme
- [ ] 2D labirent haritasının tasarlanması
  - [ ] Harita boyutlarının belirlenmesi
  - [ ] Engellerin yerleştirilmesi
- [ ] OccupancyGrid yapısının oluşturulması
- [ ] RViz konfigürasyonunun hazırlanması
  - [ ] Harita görselleştirme ayarları
  - [ ] Robot modeli görselleştirme ayarları

### 3. A* Algoritması Implementasyonu
- [ ] A* algoritması temel yapısının kodlanması
  - [ ] Node sınıfının oluşturulması
  - [ ] Heuristik fonksiyonun implementasyonu
  - [ ] Yol bulma algoritmasının kodlanması
- [ ] ROS 2 node yapısına entegrasyon
  - [ ] Gerekli topic'lerin tanımlanması
  - [ ] Service yapısının kurulması

### 4. Robot Kontrolcüsü
- [ ] Robot state publisher node'unun oluşturulması
- [ ] Hareket kontrolcüsünün implementasyonu
  - [ ] Velocity commands (cmd_vel) yapısı
  - [ ] Pozisyon takip sistemi
- [ ] Engel tespit sisteminin eklenmesi

### 5. Simülasyon Entegrasyonu
- [ ] Launch file hazırlama
  - [ ] Node'ların başlatılma sırası
  - [ ] Parametre yapılandırması
- [ ] Test senaryolarının hazırlanması
- [ ] Hata durumu yönetimi

## 🎯 Temel Özellikler
- 2D grid (matris) tabanlı labirent ortamı
- A* algoritması ile optimal yol planlama
- Dinamik engel tespiti ve yeniden rota planlama
- ROS 2 tabanlı robot kontrolü ve simülasyonu
- Gerçek zamanlı görselleştirme (RViz desteği)

## 🔧 Sistem Mimarisi

### Ana Modüller ve Sorumlulukları
| Modül | Açıklama | Durum |
|-------|-----------|--------|
| `occupancy_map_publisher` | 2D labirent haritasını OccupancyGrid formatında yayınlayan modül | 🟡 Başlanacak |
| `a_star_planner` | A* algoritması implementasyonu ve yol planlama | 🔴 Başlanmadı |
| `robot_controller` | Planlanan rotayı takip eden robot kontrolcüsü | 🔴 Başlanmadı |
| `robot_state_simulator` | Robot pozisyonunu simüle eden modül (odom yayını) | 🔴 Başlanmadı |
| `rviz_config` | RViz görselleştirme konfigürasyonu | 🔴 Başlanmadı |

### ROS 2 Topic ve Mesaj Yapısı
- `geometry_msgs/Pose`: Robot ve hedef pozisyonları
  - `/robot_pose`: Robotun anlık pozisyonu
  - `/goal_pose`: Hedef pozisyon
- `geometry_msgs/Twist`: Robot hareket komutları
  - `/cmd_vel`: Hareket kontrol komutları
- `nav_msgs/OccupancyGrid`: Labirent haritası
  - `/map`: Güncel harita verisi
- `nav_msgs/Path`: Planlanan rota
  - `/planned_path`: A* ile hesaplanan yol
- `tf2_ros`: Robot pozisyon transformasyonları

## 📁 Proje Yapısı
```
2d_robot_sim/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── simulation_launch.py
├── maps/
│   └── labirent_map.yaml
├── src/
│   ├── a_star_planner.py
│   ├── robot_controller.py
│   ├── robot_state_simulator.py
│   └── occupancy_map_publisher.py
├── config/
│   └── rviz/
│       └── robot_visualization.rviz
└── test/
    └── test_a_star.py
```

## 🔄 Çalışma Prensibi ve Test Adımları

1. **Harita Yayını ve Doğrulama**
   - [ ] Haritanın doğru formatta yayınlandığının kontrolü
   - [ ] Engellerin doğru konumlandığının teyidi
   - [ ] RViz'de harita görselleştirmesinin kontrolü

2. **Yol Planlama Testleri**
   - [ ] Basit rotalar için A* algoritması testi
   - [ ] Karmaşık engelli rotalar için test
   - [ ] Performans ve optimizasyon kontrolü

3. **Robot Kontrol Testleri**
   - [ ] Temel hareket komutlarının testi
   - [ ] Rota takip doğruluğu kontrolü
   - [ ] Hız ve ivme limitlerinin testi

4. **Dinamik Engel Yönetimi**
   - [ ] Engel tespitinin doğrulanması
   - [ ] Yeniden rota planlamanın test edilmesi
   - [ ] Tepki süresi optimizasyonu

## 📊 İlerleme Takibi
- 🔴 Başlanmadı
- 🟡 Devam Ediyor
- 🟢 Tamamlandı

Her modül veya özellik geliştirildikçe, ilgili durumu güncellenecek ve gerekli notlar eklenecektir.