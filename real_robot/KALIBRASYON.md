# GERCEK ROBOT KALIBRASYON LISTESI (kol basinda, sirayla)

Tahmini toplam sure: yarim gun. Her adimin sonucunu bu dosyaya isle.

## EMULATORDE DOGRULANDI (2026-07-22 — sahada TEKRAR GEREKMEZ)
Doosan'in resmi DRCF emulatorunde uctan uca test edildi (bkz. EMULATOR_TEST.md):
- [x] dsr servis adlari: /dsr01/motion/move_joint + move_spline_joint
- [x] Aci birimi DERECE — radyan->derece cevrimi dogru (hata 0.000 rad)
- [x] /dsr01/joint_states eklem adlari joint_1..joint_6 (beyin ISIM bazli esler,
      sira karisik gelse bile sorun degil — emulator karisik sirada yayinliyordu)
- [x] movej bloklamali calisiyor; parcali yol (play_path) + iptal calisiyor
- [x] Asamali inis (5+5cm) + 25N mantigi + dara alma Doosan protokolunde kosuyor
- [x] TAM GOREV: mini PC arayuzundeki GERCEK START butonu -> tarama -> tespit ->
      inis -> temas -> role ACIK zimpara -> kalkis -> park (RViz'de izlendi)
- [x] MINI PC UZERINDE PROVA (2026-07-23, laptopsuz): GUI START -> tarama ->
      kamera kapagi GERCEK servoyla acildi/kapandi ({"camera"} semasi fiziksel
      dogrulandi) -> tespit yok -> guvenli park; role hic acilmadi, watchdog sessiz
- [x] ROLE WATCHDOG tezgahta olculdu: yayin kesilince 12.0 sn'de otomatik SANDER_OFF

## 00. SAHA KURULUMU
Mini PC'de zaten var: ros2-end-effector calisma alani, vision/can/logic/GUI.

SAHADA CALISTIRMA (iki pencere):
1. `~/minipc_baslat.sh`  (GUI + vision + can + logic — bugunku gibi; zenoh bos calisir, zarari yok)
2. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch end_effector_ros2 gercek_robot.launch.py robot_ip:=<FIRMANIN_IP> sensorler:=false`
   (RMW on eki SART: mini PC yigini cyclonedds'te — minipc_baslat.sh:17; ayni RMW olmazsa beyin dugumleri goremez)
   (`sensorler:=false` SART: vision/can'i minipc_baslat zaten acti — cift dugum olmasin.
    Beyin GERCEK_ROBOT=1 ile launch icinden kalkar; her sey yerel DDS.)

## 0. Baglanti (kol basinda)
TEK CALISTIRMA KOMUTU launch'tir — real_kol_surucu.py ELLE CALISTIRILMAZ,
beyin onu kendi icinde otomatik kullanir. (Toplantida istenen "tek launch" bu.)
- [ ] Doosan kontrolcu IP: ________ (firma verecek)
- [ ] Mini PC ayni agda, ping atiyor
- [ ] `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch end_effector_ros2 gercek_robot.launch.py robot_ip:=<IP> sensorler:=false`
- [ ] Poz akisi test: ikinci terminalde `ros2 topic hz /dsr01/joint_states` -> ~100Hz
- [ ] JOG testi: "jog" = Doosan'in EL KUMANDASI (teach pendant/tablet) uzerindeki
      yon tuslariyla kolu elle oynatmak. Kumandayla kolu oynatirken ikinci
      terminalde `ros2 topic echo /dsr01/joint_states --field position` acik olsun:
      sayilar kolla birlikte ANLIK degisiyorsa baglanti canli demektir.
- [ ] ILK HAREKET dusuk hizla: hiz sabiti real_robot/real_kol_surucu.py dosyasinin
      BASINDA `VEL_TASIMA` (tasima hizi, derece/sn; su an 25). Ilk gun bunu 10'a
      indir, START ile tek gorev dene; sorun yoksa 25'e geri al. "Dusuk tut" = bu sayi.

## 1. Esik z yuksekligi (touch-off) -> TABLO YENIDEN URETIMI
NIYE TABLO? Beyin sahada CANLI hesap (IK) YAPMAZ — bilincli karar: canli IK
dallari simde kolu savuruyordu, koku buydu. Kolun ugrayacagi TUM pozlar onceden
tablo_uret.py ile hesaplanip poz_tablosu.json'a yazilir; beyin gorevde SADECE bu
tablodan okur (deterministik, surprizsiz). Tablo simdeki esik olculerine gore
uretildi; sahadaki esik baska yerde olacagi icin BIR KEZ yeniden uretilir (~5 dk).
- [ ] Jog ile diski esige DEGENE kadar indir (load cell ~5N gosterir)
- [ ] TCP z NEREDEN OKUNUR: Doosan el kumandasinin durum ekraninda TCP/pozisyon
      satiri X,Y,Z (mm) gosterir. Oradan: z_gercek = ________ m
- [ ] Ayni ekrandan esik cizgisi: diski esigin BIR ucuna getir (x,y oku), sonra
      OBUR ucuna getir (y oku) -> x=______ y=[______,______]
- [ ] Bu uc olcuyle tablo_uret.py calistirilir -> poz_tablosu.json yeniden uretilir.
      (O gun komutu bana yazdirirsin; laptop gerekmiyorsa mini PC Claude'a yaptirilir.)

## 2. Kamera SOL/SAG yonu robotla ayni mi? (AXIS_SIGN)
AMAC: goruntude SOLDA gorunen capak icin kol GERCEKTEN sola gitmeli. Kamera ters
monteliyse kol capagin TERS tarafina iner — 1 dakikalik testle anlasilir:
- [ ] Kol tarama pozunda, kamera acik, GUI'de canli goruntu var
- [ ] Esigin SOL ucuna YOLO'nun capak sanacagi bir isaret koy (tebesir izi veya
      koyu bant; tespit cikmazsa farkli isaret dene)
- [ ] GUI'de tespit kutusu SOL tarafta mi? Dinleyici logundaki "y=..." degeri
      sol tarafa mi dusuyor? Kol o yone mi gidiyor?
- [ ] TERS ise: akilli_dinleyici.py'de AXIS_SIGN = +1.0 yap (su an -1.0, tek satir)

## 3. "1 piksel kac metre?" (PX2M / VIEW_W_M dogrulama)
VIEW_W_M = kameranin 25cm yukseklikten gordugu alanin GERCEK genisligi (su an
0.90m VARSAYIM). Capagin y konumu bununla hesaplanir; yanlissa kol capagin tam
ustune degil YANINA iner. Test:
- [ ] Tarama yuksekligi gercekten ~25cm mi? Metreyle olc: ________ cm
- [ ] Esige CETVELLE tam 20cm arayla iki isaret koy
- [ ] Gorevi role KAPALI baslat; dinleyici logu her tespit icin "capak: ... y=..."
      yazar. Iki isaretin y farkini logdan oku: ________ cm
- [ ] 20cm'den sapma >%10 ise: VIEW_W_M_yeni = 0.90 x (20 / olculen_cm)
      -> akilli_dinleyici.py basindaki VIEW_W_M'ye yaz

## 4. Load cell
- [ ] Bos durumda okuma ~0N mi? 
- [ ] Uzerine bilinen agirlik koy (or. 2 kg = 19.6N): okuma ________ N
- [ ] 25-50N bandi elle bastirarak GUI panelinde dogrulaniyor mu?

## 5. Guvenlik (EN SON, HERKES UZAKTA)
- [ ] Fiziksel e-stop butonu deneniyor — kol aninda duruyor mu?
- [ ] GUI EMERGENCY: kol her seyi kesip parka donuyor mu? (emulatorde dogrulandi;
      sahada gercek kolla tekrar)
- [ ] GUI STOP: kol OLDUGU YERDE duruyor mu (parka gitmeden)?
- [ ] Mini PC >50N acil durumu: load cell'e sert bastir -> EMERGENCY tetikleniyor mu?
- [ ] Doosan koruma durdurmasi: dusuk hizda kasitli hafif temas testi

## 6. Ilk tam gorev
- [ ] Capak yerine tebesir/bant isareti kullan (disk donmeden, role kapali)
- [ ] START -> tarama -> tespit -> inis (25N'de durdu mu?) -> surunme yolu dogru mu?
- [ ] Her sey dogruysa role acik gercek zimpara denemesi
