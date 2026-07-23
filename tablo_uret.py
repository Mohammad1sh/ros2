#!/usr/bin/env python3
"""POZ TABLOLARINI ESIGIN GERCEK KONUMUNA GORE URET.
SAHA KULLANIMI (mini PC'de, ROS source edilmis kabukta):
  1) Asagidaki X, Z_ESIK, Y0, Y1 satirlarini SAHADA OLCULEN degerlerle degistir
     (KALIBRASYON.md bolum 1: el kumandasindan okunan TCP degerleri)
  2) python3 tablo_uret.py  ->  poz_tablosu.json yeniden yazilir (~5 dk)
  3) NOT: HIGH-LOW araligi 0.151 m SABIT kalmali (asamali inisin "5cm" orani buna bagli)
Sim olcumleri: x=-0.595, ust yuzey z=0.439, y=-0.75..+0.55
  LOW  = disk esige DEGER      -> tool z = Z_ESIK + 0.010
  HIGH = 15 cm guvenli tasima  -> tool z = Z_ESIK + 0.161
  SCAN = esigin 25 cm ustu     -> tool z = Z_ESIK + 0.250 (kamera 90cm gorur)
  KORIDOR = park -> scan, KARTEZYEN duz cizgi (araca girmez)
  UYARI: koridor/arac-kutusu denetimi SIM aracina gore; sahada bilgi amaclidir."""
import json, os, sys, math
import numpy as np
sys.path.insert(0, os.path.expanduser('~/ros2-end-effector/src/end_effector_ros2'))
from end_effector_ros2 import gazebo_bridge as gb

P = os.path.expanduser('~/ros2-end-effector/poz_tablosu.json')
T = json.load(open(P))
R = gb._R_down_yaw(0.0)
X, Z_ESIK = -0.595, 0.439
Z_LOW, Z_HIGH, Z_SCAN = Z_ESIK + 0.010, Z_ESIK + 0.161, Z_ESIK + 0.250
Y0, Y1 = -0.70, 0.50
YM = (Y0 + Y1) / 2

def coz(x, y, z, tohum):
    s = gb.ik_solve(np.array([x, y, z]), R, current=tohum)
    if s is None: return None
    q = s[0] if isinstance(s, (tuple, list)) and len(s) == 3 else s
    p = gb.forward_kinematics(q)[:3, 3]
    if np.linalg.norm(p - np.array([x, y, z])) > 0.008: return None
    return list(map(float, q))

def hat(z, adim, etiket):
    """y ekseninde tarama; her poz oncekinden TOHUMLANIR (sureklilik)."""
    tohum = list(T['park'])
    ilk = None
    for t in range(40):                       # merkezden iyi bir baslangic bul
        ilk = coz(X, YM, z, tohum)
        if ilk: break
        tohum = list(np.array(tohum) + np.random.uniform(-0.3, 0.3, 6))
    assert ilk, f'{etiket}: baslangic cozulemedi'
    orta, sag, sol = ilk, [], []
    tohum = ilk
    y = YM + adim
    while y <= Y1 + 1e-9:
        q = coz(X, y, z, tohum)
        if q is None: break
        if max(abs(np.array(q) - np.array(tohum))) > 0.35: break
        sag.append({'y': round(y, 4), 'j': q}); tohum = q; y += adim
    tohum = ilk; y = YM - adim
    while y >= Y0 - 1e-9:
        q = coz(X, y, z, tohum)
        if q is None: break
        if max(abs(np.array(q) - np.array(tohum))) > 0.35: break
        sol.insert(0, {'y': round(y, 4), 'j': q}); tohum = q; y -= adim
    tab = sol + [{'y': round(YM, 4), 'j': orta}] + sag
    d = [float(np.max(np.abs(np.array(tab[i+1]['j']) - np.array(tab[i]['j'])))) for i in range(len(tab)-1)]
    print(f'{etiket}: {len(tab)} poz  y {tab[0]["y"]:+.2f}..{tab[-1]["y"]:+.2f}  ardisik MAX Δ={max(d):.3f} rad')
    assert max(d) < 0.35, f'{etiket} sureksiz'
    return tab

print('LOW  uretiliyor (disk esikte)...');  LOW  = hat(Z_LOW,  0.004, 'LOW ')
print('HIGH uretiliyor (tasima)...');       HIGH = hat(Z_HIGH, 0.020, 'HIGH')

# HIGH<->LOW ayni y farki (dusey inis purussuz mu)
def at(tab, y):
    y = max(tab[0]['y'], min(tab[-1]['y'], y))
    for i in range(len(tab)-1):
        if tab[i]['y'] <= y <= tab[i+1]['y']:
            a = (y-tab[i]['y'])/max(tab[i+1]['y']-tab[i]['y'],1e-9)
            return np.array(tab[i]['j'])*(1-a)+np.array(tab[i+1]['j'])*a
    return np.array(tab[0]['j'] if y < tab[0]['y'] else tab[-1]['j'])
hl = max(float(np.max(np.abs(at(HIGH, p['y']) - np.array(p['j'])))) for p in LOW)
print(f'HIGH<->LOW ayni y MAX Δ = {hl:.3f} rad')
assert hl < 0.8, 'dusey blend buyuk'

SCAN = coz(X, YM, Z_SCAN, at(HIGH, YM))
assert SCAN, 'scan cozulemedi'
print(f'SCAN pozu ✓ (z={Z_SCAN:.3f})')

# KORIDOR: park -> scan, kartezyen duz cizgi (araca girmeden yandan yaklasir)
p_park = gb.forward_kinematics(T['park'])[:3, 3]
p_scan = np.array([X, YM, Z_SCAN])
print(f'park ucu {p_park.round(3)} -> scan {p_scan.round(3)}')
def coz2(x, y, z, tohum, tol=0.02):
    s = gb.ik_solve(np.array([x, y, z]), R, current=tohum)
    if s is None: return None
    q = s[0] if isinstance(s, (tuple, list)) and len(s) == 3 else s
    if np.linalg.norm(gb.forward_kinematics(q)[:3, 3] - np.array([x, y, z])) > tol: return None
    if max(abs(np.array(q) - np.array(tohum))) > 0.45: return None
    return list(map(float, q))

# YUKARIDAN yaklas: park -> tepe -> esik ustu -> scan  (araca girmez)
ARA = [p_park,
       np.array([-0.30, YM, 0.85]),      # aracin disinda, yuksekte
       np.array([X,     YM, 0.85]),      # esigin tam ustu, yuksekte
       p_scan]
KOR, tohum = [list(T['park'])], list(T['park'])
for i in range(len(ARA)-1):
    A, B = ARA[i], ARA[i+1]
    n = max(8, int(np.linalg.norm(B-A) / 0.02))
    for k in range(1, n+1):
        h = A + (B-A) * (k/n)
        q = coz2(h[0], h[1], h[2], tohum)
        if q is None:                     # IK tutmazsa eklem-uzayi ara adim
            continue
        KOR.append(q); tohum = q
if max(abs(np.array(KOR[-1]) - np.array(SCAN))) > 0.05:
    KOR.append(SCAN)
# kalan buyuk sicramalari BOL (yumusak gecis)
def yogunlastir(yol, adim=0.10):
    out = [list(yol[0])]
    for q in yol[1:]:
        a0, a1 = np.array(out[-1]), np.array(q)
        n = max(1, int(np.ceil(float(np.max(np.abs(a1-a0))) / adim)))
        for k in range(1, n+1): out.append(list(a0 + (a1-a0)*(k/n)))
    return out
KOR = yogunlastir(KOR)
d = [float(np.max(np.abs(np.array(KOR[i+1]) - np.array(KOR[i])))) for i in range(len(KOR)-1)]
print(f'KORIDOR: {len(KOR)} poz  MAX Δ={max(d):.3f} rad')
assert max(d) < 0.15, 'koridor sicramali'
# ARACA GIRIYOR MU? (tool ucu arac kutusunun icinde kalmamali)
ARC = dict(x=(-2.35,-0.59), y=(-2.13,2.56), z=(0.01,1.17))
ihlal = 0
for i, q in enumerate(KOR[:-12]):        # son yaklasma haric
    p = gb.forward_kinematics(q)[:3, 3]
    if ARC['x'][0] < p[0] < ARC['x'][1]-0.02 and ARC['y'][0] < p[1] < ARC['y'][1] and ARC['z'][0] < p[2] < ARC['z'][1]:
        ihlal += 1
print(f'koridor arac ihlali: {ihlal} poz (0 olmali)')

T['low'], T['high'], T['scan'], T['park_to_scan'] = LOW, HIGH, SCAN, KOR
json.dump(T, open(P, 'w'))
print('\n✓ poz_tablosu.json YENIDEN URETILDI (esik: x=-0.595, z=0.439)')
print(f'  LOW z={Z_LOW:.3f}  HIGH z={Z_HIGH:.3f}  SCAN z={Z_SCAN:.3f}')
