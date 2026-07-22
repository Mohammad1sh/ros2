#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
REAL KOL SURUCU v2 — Doosan H2515 gercek robot adaptoru
========================================================
akilli_dinleyici GERCEK_ROBOT=1 ile calisinca hareket ilkellerini bu sinifa
yonlendirir. Gorev beyni (kumeleme, 5+5cm inis, 25N, park) HIC degismez.

Kullanim zinciri:
  gercek_robot.launch.py -> dsr_bringup2 (mode:=real host:=<IP>)
                         -> akilli_dinleyici (GERCEK_ROBOT=1) -> RealKol

SAHA NOTLARI (kol basinda dogrulanacak — grep 'SAHA:'):
  SAHA: servis adlari /dsr01/motion/move_joint & move_spline_joint
  SAHA: dsr aci birimi DERECE (tablolar radyan -> burada cevrilir)
  SAHA: /dsr01/joint_states eklem adlari 'joint_1'..'joint_6' mi?
"""
import math, time

R2D = 180.0 / math.pi
NS  = '/dsr01'
VEL_TASIMA = 25.0     # derece/sn  (ilk testte dusuk tut!)
ACC_TASIMA = 25.0
VEL_INIS   = 6.0
ACC_INIS   = 6.0


class RealKol:
    def __init__(self, node, spin_once):
        """node: rclpy node, spin_once: callback isleme fonksiyonu"""
        import rclpy
        from dsr_msgs2.srv import MoveJoint, MoveSplineJoint
        self.rclpy = rclpy
        self.n = node
        self.spin = spin_once
        self.cli_j  = node.create_client(MoveJoint,       NS + '/motion/move_joint')
        self.cli_sj = node.create_client(MoveSplineJoint, NS + '/motion/move_spline_joint')
        for ad, c in (('move_joint', self.cli_j), ('move_spline_joint', self.cli_sj)):
            if not c.wait_for_service(timeout_sec=8.0):
                raise RuntimeError(f'dsr servisi yok: {ad} — dsr_bringup2 real modda mi?')

    # ── temel bloklayan hareket ──────────────────────────────────────────
    def movej(self, q_rad, vel=VEL_TASIMA, acc=ACC_TASIMA, t=0.0, zaman_asimi=30.0):
        from dsr_msgs2.srv import MoveJoint
        req = MoveJoint.Request()
        req.pos = [float(v) * R2D for v in list(q_rad)[:6]]
        req.vel = float(vel); req.acc = float(acc); req.time = float(t)
        req.radius = 0.0; req.mode = 0; req.blend_type = 0; req.sync_type = 0
        fut = self.cli_j.call_async(req)
        self.rclpy.spin_until_future_complete(self.n, fut, timeout_sec=zaman_asimi)
        r = fut.result()
        return bool(r and getattr(r, 'success', True))

    # ── yol: kisa parcalara bolunmus movej zinciri (iptal edilebilir) ────
    def play_path(self, path, iptal, vel=VEL_TASIMA):
        """Uzun spline yerine ~4 pozda bir movej: her parca arasi iptal bakilir.
        (Tek buyuk spline yazilim tarafindan durdurulamazdi.)"""
        adim = max(1, len(path) // 20)          # ~20 parca
        for i in range(adim, len(path), adim):
            if iptal(): return False
            if not self.movej(path[i], vel=vel): return False
        if iptal(): return False
        return self.movej(path[-1], vel=vel)

    # ── surunme: nokta listesi, her noktaya sabit surede (1cm/5sn kurali) ─
    def surun_noktalar(self, noktalar, sn_per_nokta, iptal):
        for q in noktalar:
            if iptal(): return False
            if not self.movej(q, vel=VEL_INIS, acc=ACC_INIS,
                              t=max(sn_per_nokta, 0.3)):
                return False
        return True

    # ── asamali inis (5+5cm) — kuvvet callback'iyle ──────────────────────
    def inis_asamali(self, poz_fn, dara_fn, kuvvet_fn, iptal, log,
                     esik=25.0, tepki=10.0):
        """poz_fn(a): a karisim derinligindeki eklem pozu (0=HIGH,1=LOW,>1 alt)
        Donus: ('TEMAS'|'BOS'|'IPTAL', a)"""
        a_ust, a_alt = None, None   # sinifin disindan gecirilecek — basitlik icin sabit oranlar:
        a_ust = 1 - 0.05 / 0.151
        a_alt = 1 + 0.05 / 0.151
        if not self.movej(poz_fn(a_ust), vel=VEL_TASIMA): return ('IPTAL', a_ust)
        dara = dara_fn()
        log(f'gercek inis: dara={dara:.1f}, kuvvet izlenerek iniliyor')
        def kademe(a0, a1, n=10):
            for k in range(1, n + 1):
                if iptal(): return ('IPTAL', a0)
                a = a0 + (a1 - a0) * k / n
                if not self.movej(poz_fn(a), vel=VEL_INIS, acc=ACC_INIS):
                    return ('IPTAL', a)
                d = kuvvet_fn() - dara
                if d >= esik:
                    log(f'TEMAS ✓ net +{d:.1f}N (a={a:.2f})')
                    return ('TEMAS', a)
            return None
        r = kademe(a_ust, 1.0)
        if r: return r
        d = kuvvet_fn() - dara
        if d >= tepki:
            r = kademe(1.0, a_alt)
            if r: return r
            d = kuvvet_fn() - dara
            return (('TEMAS' if d >= tepki else 'BOS'), a_alt)
        log('5cm tepkisiz — +5cm daha')
        r = kademe(1.0, a_alt)
        if r: return r
        d = kuvvet_fn() - dara
        return (('TEMAS' if d >= tepki else 'BOS'), a_alt)
