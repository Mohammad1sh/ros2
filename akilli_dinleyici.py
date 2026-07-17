#!/usr/bin/env python3
"""AKILLI DINLEYICI — gercek tespit-gudumlu zimparalama (IK'siz, poz tablosuyla).

Akis (mini PC arayuzunde START'a basilinca):
  1. Kol tarama pozuna gider (esigin 25cm ustu)
  2. Kamera penceresi boyunca gercek YOLO tespitleri toplanir
  3. Tespitler esik cizgisine izdusurulur, bosluklara gore bolgelere ayrilir
  4. Her bolge: diskin MERKEZI bolge BASINA iner -> 5sn/1cm ilerleyerek
     zimparalar -> bolge sonunda KALKAR -> sonraki bolgeye gecer
  5. Bitince (veya hic tespit yoksa) geri cekilir ve PARK'a doner
EMERGENCY STOP: aninda kalk + park.
"""
import json, math, os, time
import rclpy
from std_msgs.msg import Bool, String, Float64MultiArray

WS = os.path.expanduser('~/ros2-end-effector')
T = json.load(open(os.path.join(WS, 'poz_tablosu.json')))

# ── Ayarlar ──
FRAME_W      = 1280          # vision_node kare genisligi (px)
VIEW_W_M     = 0.90          # 25cm yukseklikte gorus genisligi (m)
PX2M         = VIEW_W_M / FRAME_W
AXIS_SIGN    = -1.0          # goruntu-x -> robot-y isareti (gerekirse +1 yap)
SCAN_Y       = -0.129        # tarama pozunun y'si (goruntu merkezi buraya bakar)
CAM_WINDOW_S = 15.0          # tespit toplama suresi (kamera acilma gecikmesi dahil)
GAP_M        = 0.10          # bu boslugtan buyukse KALK-GEC (disk capi)
CREEP_MPS    = 0.01 / 5.0    # 1 cm / 5 sn
TRAVERSE_S   = 0.12          # tablo pozlari arasi gecis suresi (hizli hareket)
SPIN         = 80.0
RATE         = 20

LOW  = T['low'];  HIGH = T['high']
Y_MIN, Y_MAX = LOW[0]['y'], LOW[-1]['y']

rclpy.init()
n = rclpy.create_node('akilli_dinleyici')
pub_j = n.create_publisher(Float64MultiArray, '/gz/dsr_position_controller/commands', 10)
pub_z = n.create_publisher(Float64MultiArray, '/gz/zimpara_velocity_controller/commands', 10)

state = {'emergency': False, 'start': False, 'dets': []}
n.create_subscription(Bool, '/end_effector/mission_start',
                      lambda m: state.__setitem__('start', state['start'] or m.data), 10)
n.create_subscription(Bool, '/end_effector/emergency_stop',
                      lambda m: state.__setitem__('emergency', state['emergency'] or m.data), 10)
def on_det(m):
    try:
        d = json.loads(m.data)
        if d.get('burrs'):
            state['dets'].append(d['burrs'])
    except Exception:
        pass
n.create_subscription(String, '/end_effector/detections', on_det, 10)

from rosgraph_msgs.msg import Clock
sim = {'t': None}
n.create_subscription(Clock, '/clock', lambda m: sim.__setitem__('t', m.clock.sec + m.clock.nanosec*1e-9), 10)
def now():
    rclpy.spin_once(n, timeout_sec=0.0)
    return sim['t'] if sim['t'] is not None else time.time()

def send(q, spin=0.0):
    m = Float64MultiArray(); m.data = [float(v) for v in q] + [0.0]
    pub_j.publish(m)
    z = Float64MultiArray(); z.data = [float(spin)]
    pub_z.publish(z)

def move_seg(q0, q1, dur, spin=0.0):
    """iki poz arasi kosinuslu, sim-saatli, surekli yayinla"""
    t0 = now(); import numpy as np
    q0 = np.array(q0); q1 = np.array(q1)
    while True:
        if state['emergency']: return False
        t = now() - t0
        a = min(1.0, t / max(dur, 1e-3))
        a = 0.5 - 0.5 * math.cos(math.pi * a)
        send((1 - a) * q0 + a * q1, spin)
        if t >= dur: return True
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.01)

def play_path(path, per_seg=None, spin=0.0):
    for k in range(1, len(path)):
        if not move_seg(path[k-1], path[k], per_seg or TRAVERSE_S, spin):
            return False
    return True

def table_at(table, y):
    """tabloda y'ye karsilik gelen eklem pozu (komsu interp)"""
    import numpy as np
    y = max(Y_MIN, min(Y_MAX, y))
    for i in range(len(table) - 1):
        if table[i]['y'] <= y <= table[i+1]['y']:
            a = (y - table[i]['y']) / max(table[i+1]['y'] - table[i]['y'], 1e-9)
            return list((1-a) * np.array(table[i]['j']) + a * np.array(table[i+1]['j']))
    return list(table[0]['j'] if y < table[0]['y'] else table[-1]['j'])

def glide(table, y0, y1, speed, spin=0.0):
    """hat boyunca y0->y1, verilen hizla (m/s)"""
    dist = abs(y1 - y0); dur = dist / max(speed, 1e-6)
    t0 = now()
    while True:
        if state['emergency']: return False
        t = now() - t0
        a = min(1.0, t / max(dur, 1e-3))
        y = y0 + (y1 - y0) * a
        send(table_at(table, y), spin)
        if a >= 1.0: return True
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.01)

def to_park():
    # guvenli donus: yuksek hatta scan tarafina -> scan -> park (ayna)
    send(T['scan']); time.sleep(0.3)
    path = list(reversed(T['park_to_scan']))
    play_path(path, per_seg=0.10)
    # parki kilitle
    for _ in range(40):
        send(T['park']); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.05)

def cluster(dets):
    """tespit karelerinden esik-y bolgeleri cikar"""
    ys = []
    for frame in dets:
        for b in frame:
            dy = (b['x'] - FRAME_W / 2) * PX2M * AXIS_SIGN
            ys.append(max(Y_MIN, min(Y_MAX, SCAN_Y + dy)))
    if not ys: return []
    ys.sort()
    groups = [[ys[0]]]
    for y in ys[1:]:
        if y - groups[-1][-1] > GAP_M:
            groups.append([y])
        else:
            groups[-1].append(y)
    return [(g[0], g[-1]) for g in groups]

print('╔════════════════════════════════════════════════╗')
print('║ AKILLI DINLEYICI HAZIR — mini PC\'de START\'a bas ║')
print('╚════════════════════════════════════════════════╝', flush=True)

while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
    if state['emergency']:
        print('EMERGENCY -> park', flush=True)
        state['emergency'] = False; state['start'] = False
        to_park(); continue
    if not state['start']:
        continue
    state['start'] = False
    state['dets'] = []
    print('START -> tarama pozuna gidiliyor (esik +25cm)...', flush=True)
    if not play_path(T['park_to_scan'], per_seg=0.12):
        to_park(); continue

    print(f'Kamera penceresi: {CAM_WINDOW_S:.0f} sn tespit toplaniyor...', flush=True)
    t0 = now()
    while now() - t0 < CAM_WINDOW_S:
        if state['emergency']: break
        send(T['scan'])
        rclpy.spin_once(n, timeout_sec=0.05)
    if state['emergency']:
        state['emergency'] = False; to_park(); continue

    bolgeler = cluster(state['dets'])
    print(f'{sum(len(f) for f in state["dets"])} tespit -> {len(bolgeler)} bolge: '
          + ', '.join(f'[{a:+.2f}..{b:+.2f}]' for a, b in bolgeler), flush=True)

    if not bolgeler:
        print('Tespit YOK -> geri cekiliyor, park.', flush=True)
        to_park(); continue

    ok = True
    cur_y = None
    for (ya, yb) in bolgeler:
        y_end = yb + 0.01   # kullanici kurali: baslangictan itibaren 5sn/1cm; min 1cm
        # yuksek hatta bolge basina git
        entry = table_at(HIGH, ya)
        if cur_y is None:
            ok = move_seg(T['scan'], entry, 2.0)
        else:
            ok = glide(HIGH, cur_y, ya, 0.10)
        if not ok: break
        print(f'  bolge [{ya:+.2f}..{y_end:+.2f}] -> inis', flush=True)
        ok = move_seg(entry, table_at(LOW, ya), 2.0)          # inis
        if not ok: break
        print(f'  ZIMPARA: {abs(y_end-ya)*100:.0f} cm @ 1cm/5sn', flush=True)
        ok = glide(LOW, ya, y_end, CREEP_MPS, spin=SPIN)       # surunme zimpara
        send(table_at(LOW, y_end), 0.0)
        if not ok: break
        ok = move_seg(table_at(LOW, y_end), table_at(HIGH, y_end), 1.5)  # kalk
        if not ok: break
        cur_y = y_end
    print('Bolgeler bitti -> park.', flush=True)
    to_park()
    print('HAZIR — tekrar START bekleniyor.', flush=True)
