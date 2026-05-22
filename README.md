# Multi-Sensor Data Acquisition & Statistical Analysis over BLE with ESP32

Bu proje; ESP32 mikrodenetleyicisi üzerinde **I2C haberleşme protokolü** kullanılarak 3 farklı gelişmiş sensörden eş zamanlı veri toplayan, sinyal gürültüsünü engellemek adına **hareketli medyan filtresi** uygulayan ve toplanan verilerin istatistiksel analizlerini çıkartarak **NimBLE (Bluetooth Low Energy)** üzerinden canlı yayınlayan kararlı ve optimize edilmiş bir gömülü yazılım (firmware) uygulamasıdır.

## 🚀 Proje Mimarisi ve Çalışma Adımları

Sistem, ana döngü içerisinde **1 Hz (saniyede 1 örnek)** frekansında çalışır ve her döngüde sırasıyla aşağıdaki adımları koşturur:

1. **Veri Toplama (I2C):** `Wire` kütüphanesi yardımıyla MPL3115A2, ICG-20660L ve BME280 sensörlerinin ham register verileri okunur ve fiziksel anlamlı büyüklüklere (Pa, °C, g, d/s, %) dönüştürülür.
2. **Sinyal Filtreleme:** Ham veriler, ani sıçramaları (spike) ve gürültüyü baskılamak için **7 elemanlı bir Hareketli Medyan Filtresine (Moving Median Filter)** sokulur. Filtreleme işlemi için `Insertion Sort` algoritması kullanılır.
3. **Dairesel Arabellek (Circular Buffer):** Filtrelenmiş veriler, bellek optimizasyonu sağlamak adına her kanal için ayrı tanımlanmış **32 eleman kapasiteli dairesel tamponlara (Circular Buffer - FIFO)** yazılır. Tampon dolduğunda en eski veri otomatik olarak ezilir.
4. **Anlık İzleme:** Her saniye, tampon yapısını bozmadan (`buffer_peek_latest`) sensörlerden alınan en son filtrelenmiş veriler Seri Ekran (Serial Monitor) üzerinden formatlı bir şekilde yazdırılır.
5. **İstatistiksel Analiz ve BLE Aktarımı:** Her 30 saniyede bir (`BLE_SEND_PERIYOT`), mevcut tamponların anlık durum kopya görüntüleri (**Snapshot**) alınır. Bu veriler üzerinden;
   * **Minimum & Maximum** değerler,
   * **Medyan (Ortanca)** değer,
   * **Standart Sapma (Standard Deviation)** hesaplanır.
   Hesaplanan bu metrikler Nordic UART Service (NUS) protokolü kullanılarak **NimBLE** üzerinden bağlı olan istemciye (örn: *Serial Bluetooth Terminal*) paketler halinde `Notify` edilir ve Seri Ekran'da raporlanır.

---

## 📑 Sensör & Kanal Haritası

Proje toplamda **11 farklı veri kanalını** eş zamanlı olarak yönetmektedir:

| Kanal No | Sensör | Ölçülen Parametre | I2C Adresi |
| :---: | :---: | :--- | :---: |
| **K00** | MPL3115A2 | Basınç (Pa) | `0x60` |
| **K01** | MPL3115A2 | Sıcaklık (°C) | `0x60` |
| **K02** | ICG-20660L | İvmeölçer X Ekseni (g) | `0x69` |
| **K03** | ICG-20660L | İvmeölçer Y Ekseni (g) | `0x69` |
| **K04** | ICG-20660L | İvmeölçer Z Ekseni (g) | `0x69` |
| **K05** | ICG-20660L | Jiroskop X Ekseni (d/s) | `0x69` |
| **K06** | ICG-20660L | Jiroskop Y Ekseni (d/s) | `0x69` |
| **K07** | ICG-20660L | Jiroskop Z Ekseni (d/s) | `0x69` |
| **K08** | BME280 | Sıcaklık (°C) | `0x76` |
| **K09** | BME280 | Basınç (Pa) | `0x76` |
| **K10** | BME280 | Nem (%) | `0x76` |

---

## 🛠️ Donanım Bağlantıları (Pinout)

ESP32 için varsayılan I2C pin mimarisi kullanılmıştır:
* **SDA:** GPIO 21
* **SCL:** GPIO 22
* Tüm sensörler I2C hattı üzerinde paralel olarak bağlanmalı ve hat üzerinde uygun Pull-Up dirençlerinin bulunduğundan emin olunmalıdır.

---

## 💻 Öne Çıkan Yazılımsal Özellikler

* **NimBLE Entegrasyonu:** Standart BLE kütüphanelerine kıyasla **~%50 daha az RAM/Flash** tüketen ve daha kararlı bağlantı sunan `NimBLEDevice` mimarisi kullanılmıştır.
* **MTU Optimizasyonu:** BLE MTU değeri `256` olarak set edilmiştir. Bu sayede uzun istatistik string satırları paket bölünmesine uğramadan, tek bir BLE paketiyle (throughput optimizasyonu) hızlıca gönderilir.
* **Veri Tutarlılığı (Data Integrity):** BME280 sensör yapısı gereği `t_fine` hassas sıcaklık parametresine bağımlı olduğundan, kod mimarisinde her zaman önce sıcaklık hesaplanıp ardından basınç ve nem hesaplamasına geçilerek veri bozulmalarının önüne geçilmiştir.
* **Gelişmiş Bellek Yönetimi:** İstatistikler hesaplanırken ana dairesel arabellek kilitlenmez veya tüketilmez. `buffer_snapshot` fonksiyonu ile verilerin anlık gölgesi çıkarılarak analiz edilir, böylece kesintisiz veri toplama sürdürülür.

---

## 📥 Kurulum ve Çalıştırma

1. Bilgisayarınızda Arduino IDE veya PlatformIO'nun kurulu olduğundan emin olun.
2. `NimBLE-Arduino` kütüphanesini kütüphane yöneticisinden projenize dahil edin.
3. `son.ino` dosyasını kartınıza yükleyin.
4. Seri Port ekranını açıp baud rate değerini `115200` olarak ayarlayın.
5. Mobil cihazınızdan **Serial Bluetooth Terminal** benzeri bir BLE uygulamasını açarak **"ESP32-SEN"** isimli cihaza bağlanın ve 30 saniyelik periyotlarla gelen istatistiksel analiz raporlarını izleyin.

---

### 📝 Lisans
Bu proje open-source olup, eğitim ve geliştirme amaçlı serbestçe kullanılabilir, değiştirilebilir.
