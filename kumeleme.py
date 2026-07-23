#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KUTU-BAZLI CAPAK ESLESTIRME (saf python — ROS'suz, birim-test edilebilir).

Kural (kullanici tasarimi, 2026-07-23):
  * SABIT MESAFE YOK (3-5cm kalkti). Yeni tespitin MERKEZI, kumenin referans
    kutusunun BUYUTME oraniyla (orn. %30) genisletilmis haline dusuyorsa
    AYNI CAPAKTIR.
  * Referans kutu = kume uyelerinin MEDYAN merkez/boyutu (tek uc deger kaydirmaz;
    her kare sonunda tazelenir).
  * Birden cok kume uyarsa merkezi EN YAKIN olani secilir.
  * Ayni karede ayni kumeye iki kutu dusse de kume o kare icin BIR kez sayilir
    (n = kac FARKLI karede goruldu — kalicilik esigi bunun uzerinden).
  * Kareler ARDISIK olmak zorunda degil (1., 6., 8., 13. kare gecerli sayim).

Girdi : dets = [kare, kare, ...]   kare = [{'x':cx[,'y':cy,'w':w,'h':h,...]}, ...]
        (eski sema {'x'} tek basina da calisir — kutu yoksa varsayilan_wh kullanilir)
Cikti : [{'cx','cy','w','h','n','uye'}]  n=kac karede, uye=toplam tespit sayisi
"""


def _medyan(v):
    s = sorted(v)
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def kumele(dets, buyutme=0.30, varsayilan_wh=53.0):
    kumeler = []
    for ki, kare in enumerate(dets):
        dokunulan = False
        for b in kare:
            try:
                cx = float(b['x'])
            except (KeyError, TypeError, ValueError):
                continue
            try:
                cy = float(b.get('y') or 0.0)
            except (TypeError, ValueError):
                cy = 0.0
            try:
                w = float(b.get('w') or 0.0) or float(varsayilan_wh)
                h = float(b.get('h') or 0.0) or float(varsayilan_wh)
            except (TypeError, ValueError):
                w = h = float(varsayilan_wh)

            en_iyi, en_yakin = None, None
            for k in kumeler:
                yari_w = k['w'] * (1.0 + buyutme) / 2.0
                yari_h = k['h'] * (1.0 + buyutme) / 2.0
                if abs(cx - k['cx']) <= yari_w and abs(cy - k['cy']) <= yari_h:
                    d = (cx - k['cx']) ** 2 + (cy - k['cy']) ** 2
                    if en_yakin is None or d < en_yakin:
                        en_iyi, en_yakin = k, d
            if en_iyi is None:
                kumeler.append({'cx': cx, 'cy': cy, 'w': w, 'h': h,
                                '_cx': [cx], '_cy': [cy], '_w': [w], '_h': [h],
                                'kareler': {ki}})
            else:
                en_iyi['_cx'].append(cx); en_iyi['_cy'].append(cy)
                en_iyi['_w'].append(w);   en_iyi['_h'].append(h)
                en_iyi['kareler'].add(ki)
            dokunulan = True
        if dokunulan:                       # kare bitti: referanslari tazele
            for k in kumeler:
                k['cx'] = _medyan(k['_cx']); k['cy'] = _medyan(k['_cy'])
                k['w'] = _medyan(k['_w']);   k['h'] = _medyan(k['_h'])
    return [{'cx': k['cx'], 'cy': k['cy'], 'w': k['w'], 'h': k['h'],
             'n': len(k['kareler']), 'uye': len(k['_cx'])} for k in kumeler]
