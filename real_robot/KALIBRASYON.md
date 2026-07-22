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

## 00. SAHA KURULUMU — SAHADA LAPTOP YOK, HER SEY MINI PC'DE
Mini PC'de zaten var: ros2-end-effector calisma alani, vision/can/logic/GUI.
Sahadan ONCE mini PC'ye eklenecek iki sey:
- [ ] Repo guncellemesi: `cd ~/ros2-end-effector && git pull`
      (beynin GERCEK modu, real_robot/, gercek_robot.launch.py, poz_tablosu.json)
- [ ] Doosan resmi surucusu (laptopta gitignore'lu — repo'da YOK, ayri kurulur):
      `cd ~/ros2-end-effector/src`
      `git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git`
      `cd .. && rosdep install -i --from-path src/doosan-robot2 -y`
      `colcon build --packages-up-to dsr_bringup2 dsr_controller2 && source install/setup.bash`
- [ ] Dogrulama (robot olmadan): `ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py`
      hatasiz aciliyor mu (mode virtual, docker yoksa surucu baglanti bekler — normal)

SAHADA CALISTIRMA (iki pencere):
1. `~/minipc_baslat.sh`  (GUI + vision + can + logic — bugunku gibi; zenoh bos calisir, zarari yok)
2. `ros2 launch end_effector_ros2 gercek_robot.launch.py robot_ip:=<FIRMANIN_IP> sensorler:=false`
   (`sensorler:=false` SART: vision/can'i minipc_baslat zaten acti — cift dugum olmasin.
    Beyin GERCEK_ROBOT=1 ile launch icinden kalkar; laptop/zenoh YOK, her sey yerel DDS.)

MINI PC CLAUDE'A VERILECEK PROMPT (kopyala-yapistir):
"~/ros2-end-effector reposunu git pull yap. src/ altina DoosanRobotics/doosan-robot2
humble dalini klonla, rosdep ile bagimliliklari kur, `colcon build --packages-up-to
dsr_bringup2 dsr_controller2` ile derle, install/setup.bash'i .bashrc'ye ekli oldugunu
dogrula. Sonra `ros2 launch end_effector_ros2 gercek_robot.launch.py` --show-args ile
launch'in gorundugunu kanitla. Robot IP'si henuz yok; baglanti deneme. Bitince derleme
ciktisindaki hata/uyari ozetini raporla."

## 0. Baglanti (kol basinda)
- [ ] Doosan kontrolcu IP: ________ (firma verecek)
- [ ] Mini PC ayni agda, ping atiyor
- [ ] `ros2 launch end_effector_ros2 gercek_robot.launch.py robot_ip:=<IP> sensorler:=false`
- [ ] /dsr01/joint_states akiyor mu: `ros2 topic hz /dsr01/joint_states` (~100Hz beklenir)
- [ ] Kolu elle jog yap — pozlar canli degisiyor mu?
- [ ] ILK HAREKET: bos alanda, dusuk hizda tek movej — beklenen yere gitti mi?
      (real_kol_surucu.py VEL_TASIMA=25 derece/sn — ilk gun DUSUK TUT)

## 1. Esik z yuksekligi (touch-off) — SONRASINDA TABLOLAR YENIDEN URETILIR
- [ ] Kolu jog ile disk esige DEGENE kadar indir (load cell ~5N gosterir)
- [ ] O andaki TCP z degerini oku: z_gercek = ________ m
- [ ] Esik cizgisinin x'i ve y-araligi da olculur: x=______ y=[______,______]
- [ ] tablo_uret.py bu olculerle calistirilip poz_tablosu.json YENIDEN uretilir
      (sim degerleri: cizgi x=-0.595, ust z=0.439 — sahada FARKLI olacak)

## 2. Kamera-robot eksen esleme (AXIS_SIGN testi)
- [ ] Kol tarama pozunda, kamera acik
- [ ] Esigin SOL ucuna bir kagit parcasi koy
- [ ] Sistem capagi solda gosterip kolu SOLA mi goturuyor?
- [ ] Ters gidiyorsa: akilli_dinleyici.py'de AXIS_SIGN = +1.0 yap (su an -1.0)

## 3. Piksel-metre olcegi (PX2M dogrulama)
- [ ] Tarama yuksekligi tam 25 cm mi? Metre ile olc: ________ cm
- [ ] Esik uzerine 20 cm arayla iki isaret koy
- [ ] Tespit piksellerinden hesaplanan mesafe: ________ cm
- [ ] Sapma >%10 ise: VIEW_W_M degerini olculen genislikle degistir

## 4. Load cell
- [ ] Bos durumda okuma ~0N mi? (beyin iniste OTOMATIK dara alir; yine de kontrol)
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
