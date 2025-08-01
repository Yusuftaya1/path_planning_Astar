# 2D Otonom Robot Simülasyonu - A* Yol Planlama Projesi

## 📋 Proje Özeti
Bu proje, ROS 2 tabanlı bir simülasyon ortamında, 2D bir labirent içerisinde A* algoritması kullanarak yol planlama yapan ve bu yolu takip eden otonom bir robot uygulamasıdır. Prim's algoritması ile oluşturulan labirentte, A* algoritması kullanılarak optimal yol planlaması yapılmaktadır.

## 📝 Proje Durumu ve Yapılacaklar Listesi

### 1. Proje Altyapısı Kurulumu ✅
- [x] ROS 2 workspace oluşturma
- [x] Gerekli paketlerin kurulumu
  - [x] nav_msgs
  - [x] geometry_msgs
  - [x] tf2_ros
- [x] Proje paket yapısının oluşturulması
  - [x] CMakeLists.txt düzenleme
  - [x] package.xml hazırlama
  - [x] Klasör yapısının oluşturulması

### 2. Harita ve Görselleştirme ✅
- [x] 2D labirent haritasının tasarlanması
  - [x] Harita boyutlarının belirlenmesi (50x50 grid)
  - [x] Prim's algoritması ile labirent oluşturma
  - [x] Başlangıç ve bitiş noktalarının belirlenmesi
- [x] OccupancyGrid yapısının oluşturulması
  - [x] Grid hücre değerlerinin ayarlanması (0: boş, 1: duvar, 50: başlangıç, 25: hedef)
  - [x] Harita koordinat sisteminin ayarlanması
- [x] RViz konfigürasyonunun hazırlanması
  - [x] Harita görselleştirme ayarları
  - [x] Yol görselleştirme ayarları

### 3. A* Algoritması Implementasyonu ✅
- [x] A* algoritması temel yapısının kodlanması
  - [x] GridNode sınıfının oluşturulması
  - [x] Manhattan mesafe heuristiği implementasyonu
  - [x] Yol bulma algoritmasının kodlanması
- [x] ROS 2 node yapısına entegrasyon
  - [x] Map subscriber implementasyonu
  - [x] Path publisher implementasyonu
  - [x] Koordinat dönüşümlerinin implementasyonu

### 4. Robot Kontrolcüsü 🔄
- [ ] Robot state publisher node'unun oluşturulması
- [ ] Hareket kontrolcüsünün implementasyonu
  - [ ] Velocity commands (cmd_vel) yapısı
  - [ ] Pozisyon takip sistemi
- [ ] Engel tespit sisteminin eklenmesi

### 5. Simülasyon Entegrasyonu 🔄
- [x] Launch file hazırlama
  - [x] Node'ların başlatılma sırası
  - [x] Parametre yapılandırması
- [ ] Test senaryolarının hazırlanması
- [ ] Hata durumu yönetimi

## 🎯 Temel Özellikler
- 50x50 grid tabanlı labirent ortamı (0.1m/hücre çözünürlük)
- Prim's algoritması ile çözülebilir labirent oluşturma
- Manhattan mesafe heuristiği kullanan A* yol planlama
- ROS 2 tabanlı modüler yapı
- Gerçek zamanlı görselleştirme (RViz desteği)

## 🔧 Sistem Mimarisi

### Ana Modüller ve Sorumlulukları
| Modül | Açıklama | Durum |
|-------|-----------|--------|
| `occupancy_map_publisher` | Prim's algoritması ile labirent oluşturan ve OccupancyGrid formatında yayınlayan modül | ✅ Tamamlandı |
| `a_star_planner` | A* algoritması implementasyonu ve yol planlama | ✅ Tamamlandı |
| `robot_controller` | Planlanan rotayı takip eden robot kontrolcüsü | 🔄 Başlanacak |
| `robot_state_simulator` | Robot pozisyonunu simüle eden modül (odom yayını) | 🔄 Başlanacak |

### ROS 2 Topic ve Mesaj Yapısı
- `nav_msgs/OccupancyGrid`
  - `/map`: Labirent haritası (50x50 grid)
- `nav_msgs/Path`
  - `/planned_path`: A* ile hesaplanan yol
- `geometry_msgs/PoseStamped` (Planlandı)
  - `/robot_pose`: Robot pozisyonu
  - `/goal_pose`: Hedef pozisyon
- `geometry_msgs/Twist` (Planlandı)
  - `/cmd_vel`: Robot hareket komutları

## 📁 Proje Yapısı
```
path_planning_robot/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── path_planning_robot/
│       ├── occupancy_map_publisher.hpp
│       └── a_star_planner.hpp
├── src/
│   ├── occupancy_map_publisher.cpp
│   └── a_star_planner.cpp
├── launch/
│   └── map_publisher.launch.py
└── config/
    └── rviz/
        └── map_view.rviz
```

## 🔄 Çalışma Prensibi ve Test Adımları

1. **Harita Yayını ve Doğrulama** ✅
   - [x] Haritanın doğru formatta yayınlandığının kontrolü
   - [x] Engellerin doğru konumlandığının teyidi
   - [x] RViz'de harita görselleştirmesinin kontrolü

2. **Yol Planlama Testleri** ✅
   - [x] Başlangıç ve hedef noktaları arası yol planı
   - [x] Manhattan mesafe heuristiği doğrulaması
   - [x] Koordinat dönüşümlerinin kontrolü

3. **Robot Kontrol Testleri** 🔄
   - [ ] Temel hareket komutlarının testi
   - [ ] Rota takip doğruluğu kontrolü
   - [ ] Hız ve ivme limitlerinin testi

4. **Dinamik Engel Yönetimi** 🔄
   - [ ] Engel tespitinin doğrulanması
   - [ ] Yeniden rota planlamanın test edilmesi
   - [ ] Tepki süresi optimizasyonu

## 📊 İlerleme Takibi
- ✅ Tamamlandı
- 🔄 Devam Ediyor
- ⏳ Başlanmadı

## 🔜 Sonraki Adımlar
1. Robot kontrolcüsü node'unun implementasyonu
2. Robot state simulator'ün geliştirilmesi
3. Test senaryolarının hazırlanması
4. Hata durumu yönetiminin eklenmesi

Her modül veya özellik geliştirildikçe, ilgili durumu güncellenecek ve gerekli notlar eklenecektir.