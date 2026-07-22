# Doosan Emülatörüyle Sahasız Doğrulama (GERCEK mod testi)

**Amaç:** Sahaya gitmeden `GERCEK_ROBOT=1` zincirini (RealKol → /dsr01 servisleri →
Doosan DRCF protokolü) resmi emülatörde uçtan uca test etmek.

**Durum: 2026-07-22'de TAMAMEN GEÇTİ.**
- Faz A: bringup `mode:=virtual` + DRCF konteyneri + `/dsr01/motion/move_joint`,
  `move_spline_joint`, `/dsr01/joint_states` (isimler `joint_1..6`, ~95 Hz) ✓
- Faz B: RealKol `movej` / `play_path` (16 pozluk koridor) / `surun_noktalar` /
  iptal — hepsi 0.000 rad hata ile ✓
- Faz C: beyin `GERCEK_ROBOT=1` + sentetik mini PC ile TAM GÖREV:
  START → tarama → 2 çapak → bölge başına aşamalı iniş → 25N temas → röle AÇIK
  5cm zımpara → kalkış → park. Olay sırası birebir spek ✓

## Bilinen tuzak (kök neden bulundu)
`dsr_hardware2` DRCF'e **20 × 0.5 sn = 10 sn** bağlanmayı dener; emülatör
konteynerinin İLK açılışı ~40 sn sürer → sürücü "missing state interfaces" ile
çöker. Ayrıca orijinal `run_drcf.sh` çalışan konteyneri ÖLDÜRÜP yeniden kurar
(yarış her başlatmada tekrarlar).

**Çözüm:** `run_drcf.sh`'ye "çalışıyorsa yeniden kullan" yaması + konteyneri
önceden ısıtma. Yama gitignore'lu dosyalarda (src/doosan-robot2 + install)
olduğundan repoda DEĞİL — `zz_emu_hazirla.sh` her çağrıldığında güvenle yeniden
uygular (idempotent). **Saha etkisi SIFIR:** `mode:=real`'de run_drcf hiç çalışmaz.

## Nasıl çalıştırılır (Desktop/ros2-end-effector-kod-2026-07-15 içinden)
Ön koşul: Docker Desktop AÇIK + WSL entegrasyonu Ubuntu-22.04 için etkin.

1. `zz_emu_hazirla.sh`  — run_drcf yaması + konteyner ön-ısıtma (DRCF_HAZIR bekler)
2. `zz_emu_taze.sh`     — temiz oturum: konteyner restart + bringup virtual + aktivasyon
3. `zz_emu_b_kos.sh`    — RealKol ilkel testleri (log: ~/emu_b.log, `FAZ_B_SONUC`)
4. `zz_emu_c_kos.sh`    — beyin + sentetik mini PC tam görev (log: ~/emu_c_sentetik.log,
   `FAZ_C_SONUC: GECTI` beklenir)
5. `zz_emu_temizle.sh`  — her şeyi kapat (sim yığınından ÖNCE şart — çift beyin olmasın)

Not: Test ortasında süreç öldürülürse sürücü↔DRCF oturumu asılı kalabilir
(movej cevapsız kalır). Çare: `zz_emu_taze.sh` (30 sn'de taze oturum).

## Sahada geçerli kalan farklar (emülatörün sınadıkları / sınayamadıkları)
- Sınadı: servis adları, DERECE dönüşümü, bloklamalı movej, parçalı yol + iptal,
  aşamalı iniş mantığı, dara/kuvvet callback'leri, röle-servo senkronu, park.
- Sınamadı: gerçek load cell değerleri, kamera/YOLO, CAN, gerçek çarpışma/kuvvet
  fiziği, ağ (zenoh yok — hepsi tek makinede). KALIBRASYON.md'deki SAHA maddeleri geçerli.
