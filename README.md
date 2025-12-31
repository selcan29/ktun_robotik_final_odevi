KTÜN Robotik Giriş Dersi - Final Projesi
Otonom Süpürge Robotu: QR Doğrulama ve Oda Bazlı Temizlik

Öğrenci Adı: Selcan Özdemir

Ders: Robotik Giriş

Dönem: 2025-2026 Güz

1. Proje Özeti
Bu proje, ROS (Robot Operating System) Noetic üzerinde TurtleBot3 Waffle Pi robotu kullanılarak geliştirilmiş otonom bir temizlik senaryosunu içerir. Robot; önceden haritalandırılmış (SLAM) bir ev ortamında lokalizasyon (AMCL) yaparak belirlenen odaları ziyaret eder. Her oda girişinde kamera ile QR kod okuyarak doğru odada olduğunu doğrular ve oda içi temizlik rotasını (waypoint) tamamlar. Görev sonunda bir temizlik raporu oluşturur.

2. Özellikler
    SLAM & Navigasyon: Gmapping ile harita çıkarma ve MoveBase ile otonom sürüş.

    QR Kod Doğrulama: pyzbar kütüphanesi ile oda girişlerinde görsel doğrulama.

    Hata Yönetimi (Recovery): QR bulunamadığında robotun çevresini tarama (sağa/sola dönme) ve tekrar deneme yeteneği.

    Görev Raporlama: Görev sonunda terminale ve .txt dosyasına "BAŞARILI/ATLANDI" raporu yazma.

    Akıllı Sensör: Aynı QR kodunu sürekli okuyup log kirliliği yapmayı engelleyen hafızalı okuyucu sistemi.

3. Kurulum ve Gereksinimler
Projeyi çalıştırmadan önce aşağıdaki paketlerin yüklü olduğundan emin olun:
Sistem Paketleri (Ubuntu 20.04 / ROS Noetic):
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3-msgs
sudo apt-get install ros-noetic-turtlebot3-navigation
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-map-server

Python Kütüphaneleri (QR Okuma İçin): Bu proje pyzbar kullanır. Sistem kütüphanesi olan libzbar0 mutlaka yüklenmelidir:
sudo apt-get install libzbar0
pip3 install pyzbar opencv-python

Çalışma Alanı Kurulumu:
cd ~/catkin_ws/src
# Proje klasörünü buraya kopyalayın (final_odev)
cd ~/catkin_ws
catkin_make
source devel/setup.bash

4. Çalıştırma Adımları
Proje, simülasyon ortamı ve görev mantığı olmak üzere iki aşamada çalıştırılır.
Adım 1: Simülasyonu ve Navigasyonu Başlatma

Bu komut Gazebo'yu, RViz'i, Harita Sunucusunu ve Kamera arayüzünü tek seferde açar.
Bash

roslaunch final_odev setup.launch

    ÖNEMLİ: RViz açıldıktan sonra üst menüdeki "2D Pose Estimate" butonunu kullanarak robotun haritadaki konumunu işaretleyin. (AMCL'in robotu tanıması için şarttır).

Adım 2: Görevi Başlatma
Simülasyon hazır olduğunda ve robot konumlandırıldığında, temizlik görevini başlatmak için yeni bir terminalde:

roslaunch final_odev mission.launch
Bu komut şunları tetikler:

    QR Okuyucu Node'u (Kamera taramaya başlar).

    Task Manager Node'u (Robot harekete geçer).

5. Yapılandırma (Koordinatları Değiştirme)

Robotun gideceği odalar, giriş noktaları ve temizlik rotaları config/mission.yaml dosyasında tutulmaktadır. Koordinatları değiştirmek isterseniz bu dosyayı düzenleyebilirsiniz:
YAML

salon:
  qr_text: "ROOM=SALON"
  entry:
    x: -0.73  # Giriş Koordinatı
    y: 1.11
    z: -0.04
    w: 0.99
  cleaning_points:
    - {x: -1.09, y: 3.54, z: 0.0, w: 1.0} # Temizlik noktaları

6. Dosya Yapısı
Plaintext

final_odev/
├── config/
│   └── mission.yaml        # Görev ve koordinat listesi
├── launch/
│   ├── setup.launch        # Gazebo, Navigasyon, RViz başlatıcı
│   ├── mission.launch      # QR Reader ve Task Manager başlatıcı
│   └── start_world.launch  # Sadece dünyayı başlatan yardımcı dosya
├── maps/
│   ├── map.yaml            # Harita veri dosyası
│   └── map.pgm             # Harita görseli
├── src_scripts/
│   ├── qr_reader.py        # Kamera okuma ve QR işleme kodu
│   └── task_manager.py     # Ana görev yöneticisi (Navigasyon + Karar mekanizması)
├── world/
│   └── final_house.world   # QR kodların yerleştirildiği Gazebo dünyası
├── temizlik_raporu.txt     # (Otomatik oluşur) Görev sonu raporu
├── CMakeLists.txt
└── package.xml

7. Hata Giderme

    "[ERROR] map_server could not open...": Harita dosyası yolu bulunamıyor. setup.launch içindeki map_file argümanını kontrol edin.

    Robot hareket etmiyor: RViz üzerinden "2D Pose Estimate" yapıldığından emin olun.

    QR Okunmuyor: Robot açısı QR koda tam bakmıyor olabilir. Task Manager otomatik olarak sağa/sola dönüp (Retry Behavior) tekrar deneyecektir.
    
8. Demo Video ve Sonuçlar
Projenin çalıştığını kanıtlayan ekran kaydı proje klasörüne eklenmiştir.

* **Video Dosyası:** `demo.mp4`
* **Video İçeriği:**
  1. Başlatma komutları (`setup.launch` ve `mission.launch`)
  2. Robotun Salon -> Mutfak -> Koridor -> Yatak Odası rotasını izlemesi.
  3. QR kod doğrulama anları ve hata yönetimi.
  4. Görev sonunda oluşan `temizlik_raporu.txt` dosyasının gösterimi.

**Örnek Rapor Çıktısı:**
```text
KTUN ROBOTIK FINAL - TEMIZLIK RAPORU
------------------------------------
ODA: SALON 	 -> SUCCESS
ODA: MUTFAK 	 -> SUCCESS
ODA: KORIDOR 	 -> SUCCESS
ODA: YATAK_ODASI -> SUCCESS
------------------------------------
