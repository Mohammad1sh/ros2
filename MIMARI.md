# SISTEM MIMARISI — Doosan H2515 + Zimpara Uc Elemani (Kapi Esigi Capak Alma)

Bu dokuman sistemin tamamini anlatir: hangi makine ne yapar, hangi paket
nerede calisir, hesaplamalar nasil yapilir, 25N nereden gelir.

---

## 1. BUYUK RESIM — Uc Makine, Tek Gorev

```
+------------------------+        zenoh koprusu        +------------------------+
|  LAPTOP (WSL Ubuntu)   |<---------------------------->|  MINI PC (gercek uc)   |
|  "BEYIN + SIMULASYON"  |     TCP 7447, DDS<->DDS      |  "GOZ + EL"            |
|                        |                              |                        |
|  * Gazebo simulasyonu  |   Topikler:                  |  * Kamera + YOLO       |
|  * akilli_dinleyici    |   mission_start  (START)     |  * Load cell (kuvvet)  |
|    (gorev beyni)       |   detections     (capaklar)  |  * Servo kutu kapagi   |
|  * Poz tablolari (IK)  |   mission_status (kuvvet)    |  * Zimpara rolesi      |
|                        |   servo_command  (kutu ac)   |  * Arayuz (GUI+START)  |
|                        |   sander_only    (role)      |                        |
+------------------------+                              +------------------------+
            |
            |  (gelecek: ayni beyin, adapterle)
            v
+------------------------+
|  GERCEK DOOSAN H2515   |
|  "KASLAR"              |
|  dsr_bringup2 real mod |
+------------------------+
```

Tek beyin ilkesi: gorevi YALNIZCA laptoptaki `akilli_dinleyici.py` yurutur.
Mini PC duyulari saglar (goruntu, kuvvet) ve elleri oynatir (servo, role);
karar vermez. Bu, "iki beyin" doneminde yasanan cakismalari (kamera erken
acilmasi, görev yarida kesilmesi) kokten cozdu.

---

## 2. MAKINE MAKINE PAKETLER

### 2.1 Laptop (Windows 11 + WSL2 Ubuntu 22.04, ROS2 Humble)

| Bilesen | Ne is yapar |
|---|---|
| **ROS2 Humble** | Robotik ara katman: dugumler (node) topic/servis ile konusur |
| **CycloneDDS** | ROS2'nin haberlesme tasiyicisi (RMW). `ROS_DOMAIN_ID=42` |
| **Gazebo Ignition Fortress** | Fizik simulatoru: kol, arac sasesi, temaslar, yercekimi |
| **ign_ros2_control** | Gazebo icindeki kolu ROS2 kontrolculerine baglar (/gz ad alani) |
| **forward_command_controller** | 7 eklemli pozisyon kontrolcusu (6 eklem + servo_joint). Gain 2.5 |
| **velocity_controller** | Zimpara diski donme hizi (sander_spin_joint) |
| **doosan-robot2 / dsr_description2** | Doosan'in resmi H2515 tanimi: URDF/xacro + STL gorseller |
| **end_effector_ros2 (BIZIM paket)** | Uc eleman URDF'i, arac modeli, launch, kontrolcu ayarlari |
| **akilli_dinleyici.py (BIZIM)** | GOREV BEYNI — asagida detayli |
| **poz_tablosu.json (BIZIM)** | Onceden cozulmus IK poz tablolari — asagida detayli |
| **gazebo_baslat.sh (BIZIM)** | Tek komutla her sey: temizlik -> Gazebo -> zenoh -> dinleyici (bekcili) |
| **zenoh-bridge-ros2dds** | Iki makinenin DDS'ini TCP uzerinden birbirine koprular |

### 2.2 Mini PC (gercek donanim tarafi)

| Bilesen | Ne is yapar |
|---|---|
| **vision_node** | Kamera goruntusu (1280px) + YOLO capak tespiti -> `/end_effector/detections` |
| **logic_node** | Donanim kapisi: load cell okur (kalibre Newton) -> `mission_status`; servo/role komutlarini uygular; **>50N ACIL DURUM** bekcisi |
| **GUI (arayuz)** | START butonu, canli log paneli (`/end_effector/log`), kamera goruntusu |
| **Load cell (HX711)** | Diskin esige uyguladigi normal kuvveti olcer |
| **Servo kutusu** | Kamera koruma kapagi: s=35 acik, s=160 kapali |
| **Zimpara rolesi** | Gercek zimpara motoru: 111 ac, 222 kapat |

### 2.3 Iletisim katmani
- Iki makine de `ROS_DOMAIN_ID=42` + CycloneDDS.
- **zenoh-bridge-ros2dds**: mini PC dinler (`-l tcp/0.0.0.0:7447`), laptop
  baglanir. Windows tarafinda portproxy ile WSL'e aktarilir.
- Kopru koparsa bekci dongusu 3 sn'de yeniden baslatir (iki tarafta da).

---

## 3. GOREV BEYNI — akilli_dinleyici.py adim adim

1. **Bekle**: `mission_start` (START butonu) gelene kadar park pozunda.
2. **Tarama pozuna git**: 81 pozluk dogrulanmis `park_to_scan` yolunu oynat.
   Esigin **25 cm** ustunde durur (kamera gorus genisligi 90 cm olur).
3. **Kamera kutusunu AC** (servo 35) — kol VARDIKTAN sonra. 6 sn isinma.
4. **15 sn tespit topla**: vision_node'dan gelen capak pikselleri birikir.
5. **Kutuyu kapat** (servo 160), tespitleri robot koordinatina cevir.
6. **Kumele**: her capak disk yaricapi (±5 cm) kadar aralik sayilir;
   kesisen araliklar TEK bolge olur (yakin capaklar icin in-kalk yok).
7. Her bolge icin: **HIGH hattan tasi -> INIS -> 25N kapisi** (elle/temasla
   25-50N bandinda 3 ardisik okuma) -> **role AC** -> **1 cm / 5 sn surunme**
   (bolge sonuna kadar) -> **role KAPAT -> kalkis**.
8. Bolgeler arasi boslukta disk YUKARIDA tasinir (bosluga surtmez).
9. **Tespit yoksa**: demo yok — kol sadece geri cekilir, park eder.
10. **Park**: ayni dogrulanmis koridordan geri; parkta eklem hatasi
    <0.02 radyan olana dek pozisyon kilidi.
11. **ACIL DURUM** her an: role kapat, kutu kapat, guvenli park.

---

## 4. HESAPLAMALAR

### 4.1 Ters kinematik (IK) — NEDEN CANLI DEGIL, TABLODAN?
Canli IK denendi ve basarisiz oldu: cozucu bazen kolu araca sarar, kamerayi
yanlis yone bakar. Cozum: **cevrimdisi cok-tohumlu IK + kalite filtresi**.
Her hedef nokta icin IK bircok baslangictan cozulur, sadece sunlari GECEN
cozumler tabloya girer:
- pozisyon hatasi < 1 cm
- takim tam asagi bakiyor (dik zimpara)
- eklem limitleri icinde
- onceki poza gore surekli (siçrama yok, maks eklem farki siniri)

Sonuc `poz_tablosu.json`:
- **LOW hatti**: z=0.145, 156 poz (~4 mm arayla) — zimpara yuksekligi
- **HIGH hatti**: z=0.30, 36 poz — guvenli tasima yuksekligi
- **park_to_scan**: 81 poz — parktan taramaya dogrulanmis koridor
Calisma aninda IK YOK: sadece tablo okunur. Bu yuzden hareketler garantili.

### 4.2 Piksel -> robot koordinati
Tarama yuksekligi 25 cm'de kamera 90 cm genislik gorur, goruntu 1280 px:
```
PX2M = 0.90 / 1280 = 0.000703 m/piksel   (1 px ~ 0.7 mm)
robot_y = TARAMA_Y + (piksel_x - 640) * PX2M * EKSEN_ISARETI
```
x esik cizgisine sabitlenir (capaklar esik uzerinde). EKSEN_ISARETI ayna
duzeltmesidir; gercek kurulumda tek testle dogrulanir (KALIBRASYON.md #2).

### 4.3 Kumeleme (bolgelestirme)
Her tespit y'si `[y-5cm, y+5cm]` araligina genisletilir (disk yaricapi),
kesisen araliklar birlestirilir. Bolgenin basi/sonu disk MERKEZI ilk/son
capaga denk gelecek sekilde daraltilir. Boylece 10 cm'lik disk icinde kalan
komsu capaklar icin kol tekrar tekrar inip kalkmaz.

### 4.4 Surunme hizi
Istenen kural: **5 saniyede 1 cm** => 2 mm/s. Duvar saatiyle (gercek zaman)
uygulanir; simulasyon yavaslasa bile gercek role suresi dogru kalir.

---

## 5. NIYE 25N?

25N ~ 2.5 kg'lik bir bastirma kuvvetidir (elle orta siddette bastirmak gibi).
Uc gerekceyle secildi:

1. **Kesme icin alt sinir**: Zimpara diski capagi ancak yeterli normal
   kuvvetle asindirir. Cok az kuvvette disk yuzeyde kayar, capagi parlatir
   ama almaz. Kucuk capli diskler icin tipik etkin calisma bandi
   10-40N civaridir; 25N bu bandin guvenli ortasidir.
2. **Gercek temasin kaniti**: Kuvvet esigi ayni zamanda "disk gercekten
   esige degdi" onayidir. 25N gorulmeden role acilmaz — zimpara HAVADA
   asla donmez. (Sistemde 3 ardisik olcum istenir: sensor sicramasi eleme.)
3. **Guvenlik bandina mesafe**: Mini PC 50N ustunu ACIL DURUM sayar
   (saci ezme / motoru zorlama / koruma durdurmasi riski). 25N, alarm
   esiginin tam yarisi: normal calisma ile tehlike arasinda genis marj.
   Gecerli temas bandi: **25-50N**. Ustu: uyari, sonra acil durum.

Ozet cumle (sunum icin): "25N, diskin kesmeye basladigi minimum kuvvetin
uzerinde, sacin zarar gordugu kuvvetin ise cok altinda; ayrica temasin
fiziksel kanitidir — bu yuzden zimpara rolesi ancak 25N goruldugunde acilir."

---

## 6. GERCEK ROBOTA GECIS

Degisen TEK katman kol surucusudur; beyin ve mini PC aynen kalir.

| Simulasyonda | Gercekte |
|---|---|
| /gz pozisyon kontrolcusune komut akitma | dsr_msgs2 servisleri: movej / movesj (dsr_bringup2 real mod) |
| Sabit z'ye inis + 25N bekleme | **Kuvvet korumali inis**: kucuk adimlarla in, 25N okununca DUR (`real_kol_surucu.py inis_25N`) |
| Tablo z degerleri (0.145) | Touch-off ile olculur, tabloya islenir |
| Sim disk donmesi (velocity ctrl) | Zaten gercek role (degisiklik yok) |

Hazir dosyalar: `real_kol_surucu.py` (adaptor iskeleti, rad->derece donusumu
dahil), `KALIBRASYON.md` (kol basinda yarim gunluk kontrol listesi).
Tahmin: kol basinda ~1 gun (baglanti + kalibrasyon + dusuk hizli testler).

---

## 7. GUVENLIK KATMANLARI (icten disa)

1. 25-50N temas bandi (laptop): role ancak bandda acilir, ustunde uyari
2. >50N acil durum (mini PC): kol yeni komut almaz, role+kutu kapanir, park
3. Doosan koruma durdurmasi (kontrolcu): beklenmedik dirençte motor durur
4. Fiziksel e-stop butonu: her seyin ustunde
