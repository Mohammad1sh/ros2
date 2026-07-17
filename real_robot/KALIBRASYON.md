# GERCEK ROBOT KALIBRASYON LISTESI (kol basinda, sirayla)

Tahmini toplam sure: yarim gun. Her adimin sonucunu bu dosyaya isle.

## 0. Baglanti
- [ ] Doosan kontrolcu IP: ________ (fabrika: 192.168.137.100)
- [ ] Laptop ayni agda, ping atiyor
- [ ] `ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<IP> model:=h2515`
- [ ] RViz'de gercek kolun pozu canli goruluyor (kolu elle jog yap, RViz takip etsin)
- [ ] ILK HAREKET: bos alanda, dusuk hizda (vel=5) tek movej — beklenen yere gitti mi?

## 1. Esik z yuksekligi (touch-off)
- [ ] Kolu jog ile disk esige DEGENE kadar indir (load cell ~5N gosterir)
- [ ] O andaki TCP z degerini oku: z_gercek = ________ m
- [ ] Sim degeri z=0.145 idi -> fark: ________ m
- [ ] poz_tablosu.json LOW satirlarina bu farki isle (veya inis_25N zaten
      temasla durdugu icin sadece HIGH yuksekligini guncelle)

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
- [ ] Bos durumda okuma ~0N mi? (ofset kaymasi varsa mini PC'de tare)
- [ ] Uzerine bilinen agirlik koy (or. 2 kg = 19.6N): okuma ________ N
- [ ] 25-50N bandi elle bastirarak GUI panelinde dogrulaniyor mu?

## 5. Guvenlik (EN SON, HERKES UZAKTA)
- [ ] Fiziksel e-stop butonu deneniyor — kol aninda duruyor mu?
- [ ] Mini PC >50N acil durumu: load cell'e sert bastir -> kol yeni komut
      almayi kesiyor mu? (dinleyici log: EMERGENCY)
- [ ] Doosan koruma durdurmasi: dusuk hizda kasitli hafif temas testi

## 6. Ilk tam gorev
- [ ] Capak yerine tebesir/bant isareti kullan (disk donmeden, role kapali)
- [ ] START -> tarama -> tespit -> inis (25N'de durdu mu?) -> surunme yolu dogru mu?
- [ ] Her sey dogruysa role acik gercek zimpara denemesi
