#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
REAL KOL SURUCU — Doosan H2515 gercek robot adaptoru (ISKELET)
================================================================
Simulasyondaki akilli_dinleyici.py'nin KOL KOMUT KATMANININ gercek karsiligi.
Gorev beyni (tespit -> kumeleme -> 25N kapisi -> 1cm/5sn surunme -> park)
AYNEN kalir; sadece send/move_seg/glide/play_path fonksiyonlarinin ici
bu siniftaki cagrilarla degisir.

KURULUM (kol basinda, ~1 gun):
  1) Doosan kontrolcu IP'sini ogren (fabrika varsayilani: 192.168.137.100)
  2) Laptopta gercek surucuyu baslat:
       ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
           mode:=real host:=<ROBOT_IP> model:=h2515 name:=dsr01
     (paket: doosan-robot2, humble dali — launch adi kurulumda dogrulanacak)
  3) Asagidaki TODO'lari kol basinda tek tek test ederek isaretle.
  4) KALIBRASYON.md'deki olcumleri yap, poz tablosuna isle.

KRITIK NOTLAR:
  * dsr servisleri ACI (DERECE) kullanir — poz_tablosu.json RADYAN tutar.
    Bu sinif donusumu kendisi yapar (rad2deg).
  * Poz tablosu satirlari 7 deger icerir (6 eklem + servo_joint).
    Gercek kolda ilk 6 kullanilir; kamera kutusu servosu ve zimpara rolesi
    zaten mini PC uzerinden suruluyor (servo_command / sander_only).
  * Simde "komut akitma" vardi; gercekte her hareket BLOKLAYAN servis
    cagrisidir (movej/movesj). Gorev beyni icin fark yok.
"""
import math, time

try:
    import rclpy
    from dsr_msgs2.srv import MoveJoint, MoveSplineJoint   # doosan-robot2 paketi
    DSR_VAR = True
except ImportError:
    DSR_VAR = False   # laptopta kol yokken de dosya derlenebilsin

R2D = 180.0 / math.pi
NS = '/dsr01'          # dsr_bringup2 name:=dsr01 ile eslesir

# Guvenli hiz/ivme tavanlari (derece/sn, derece/sn^2) — ILK TESTTE DUSUK TUT!
VEL_TRANSIT = 20.0     # bolgeler arasi tasima
ACC_TRANSIT = 20.0
VEL_CREEP   = 5.0      # inis / surunme
ACC_CREEP   = 5.0


class RealKol:
    """akilli_dinleyici'nin kullandigi arayuz: ayni imzalar, gercek servisler."""

    def __init__(self, node):
        self.n = node
        self.cli_movej  = node.create_client(MoveJoint,       NS + '/motion/move_joint')
        self.cli_movesj = node.create_client(MoveSplineJoint, NS + '/motion/move_spline_joint')
        for c in (self.cli_movej, self.cli_movesj):
            if not c.wait_for_service(timeout_sec=5.0):
                raise RuntimeError('dsr servisi yok — dsr_bringup2 real modda calisiyor mu?')

    # ---- temel: tek poza git (bloklar) ----------------------------------
    def movej(self, q_rad7, vel=VEL_TRANSIT, acc=ACC_TRANSIT, t=0.0):
        req = MoveJoint.Request()
        req.pos = [q * R2D for q in q_rad7[:6]]     # ilk 6 eklem, DERECE
        req.vel = float(vel); req.acc = float(acc); req.time = float(t)
        req.radius = 0.0; req.mode = 0              # mode=0: mutlak  TODO dogrula
        req.blend_type = 0; req.sync_type = 0       # senkron (bitene dek bekle)
        fut = self.cli_movej.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60.0)
        return fut.result() is not None and fut.result().success

    # ---- yol oynat: park_to_scan gibi coklu-poz yollar ------------------
    def play_path(self, path_rad, vel=VEL_TRANSIT, acc=ACC_TRANSIT):
        """Spline ile tum ara pozlardan gecer (simdeki play_path karsiligi)."""
        req = MoveSplineJoint.Request()
        req.pos = []
        for q in path_rad:
            req.pos.extend([v * R2D for v in q[:6]])
        req.pos_cnt = len(path_rad)                 # TODO alan adini dogrula
        req.vel = float(vel); req.acc = float(acc)
        req.time = 0.0; req.mode = 0; req.sync_type = 0
        fut = self.cli_movesj.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=180.0)
        return fut.result() is not None and fut.result().success

    # ---- KUVVET KORUMALI INIS (gercekte olmazsa olmaz) ------------------
    def inis_25N(self, q_high, q_low, oku_kuvvet, esik=25.0, adim=12):
        """HIGH pozundan LOW pozuna dogru KUCUK adimlarla in; her adimda
        load cell'i oku. esik'e ulasinca DUR (o an temas saglandi).
        Simdeki 'sabit z'ye in + 25N bekle' yerine gercekte bu kullanilir —
        sac esigin gercek yuksekligi tablodakinden farkliysa bile robot
        bastirmaz, temasta durur.  >50N mini PC acil durumu ayrica bekcidir."""
        for i in range(1, adim + 1):
            a = i / adim
            q = [h + (l - h) * a for h, l in zip(q_high[:6], q_low[:6])]
            if not self.movej(q, vel=VEL_CREEP, acc=ACC_CREEP):
                return False
            f = oku_kuvvet()
            if f >= esik:
                return True                          # temas — inisi bitir
        return oku_kuvvet() >= esik                  # tabana indik: temas var mi?

    # ---- 1cm/5sn surunme zimpara ----------------------------------------
    def surun(self, low_tablosu, y0, y1, oku_acil, hiz_mps=0.002):
        """LOW tablosu uzerinde y0->y1 pozlarini sirayla gez; her adimda
        adim_mesafesi/hiz kadar sure ver (t parametresi) => 2mm/s = 1cm/5sn."""
        yol = [p for p in low_tablosu if min(y0, y1) - 1e-6 <= p['y'] <= max(y0, y1) + 1e-6]
        yol.sort(key=lambda p: p['y'], reverse=(y1 < y0))
        onceki_y = y0
        for p in yol:
            if oku_acil():
                return False
            dt = abs(p['y'] - onceki_y) / hiz_mps
            if not self.movej(p['q'], vel=VEL_CREEP, acc=ACC_CREEP, t=max(dt, 0.5)):
                return False
            onceki_y = p['y']
        return True


# =========================================================================
# ENTEGRASYON NOTU — akilli_dinleyici.py'de degisecek yerler (kol basinda):
#   USE_REAL = True ise:
#     kol = RealKol(n)
#     play_path(T['park_to_scan'])  -> kol.play_path(...)
#     move_seg(a, b, sure)          -> kol.movej(b)
#     inis + bekle_25N ikilisi      -> kol.inis_25N(q_high, q_low, lambda: state['force'])
#         (bekle_25N yine cagrilabilir: inis_25N zaten temasla dondugu icin
#          band kontrolu 25-50N'i aninda gecer)
#     surunme glide(...)            -> kol.surun(LOW, ya, y_end, lambda: state['emergency'])
#     to_park()                     -> kol.play_path(ters park_to_scan)
#   Zimpara rolesi/servo/log AYNEN kalir (mini PC'ye gidiyor, degismez).
#   ACIL DURUM: state['emergency'] True olunca hicbir yeni movej gonderilmez;
#   ayrica Doosan'in kendi koruma durdurmasi ve fiziksel e-stop her zaman ustundur.
# =========================================================================
