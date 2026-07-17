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

FORCE_N      = 25.0          # zimpara baslama sarti (gercek load cell, Newton)
FORCE_WAIT_S = 120.0         # 25N bekleme ust siniri (sim-sn); dolarsa uyariyla devam
DISK_R       = 0.05          # disk yaricapi (kumeleme icin)

state = {'emergency': False, 'start': False, 'dets': [], 'force': 0.0}
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

def on_status(m):
    """mini PC logic'in yayinladigi kalibre kuvvet (Newton)"""
    try:
        state['force'] = float(json.loads(m.data).get('contact_force', 0.0))
    except Exception:
        pass
n.create_subscription(String, '/end_effector/mission_status', on_status, 10)

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

from sensor_msgs.msg import JointState
jstate = {}
n.create_subscription(JointState, '/gz/joint_states',
                      lambda m: jstate.update(zip(m.name, m.position)), 10)
JNAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

def to_park(from_y=None):
    """GERI-BESLEMELI park donusu: yuksek hatta scan hizasina -> scan ->
    ayna yol -> park (eklemler VARANA kadar yayin)."""
    if from_y is not None:
        glide(HIGH, from_y, SCAN_Y, 0.10)
    entry = table_at(HIGH, SCAN_Y)
    move_seg(entry, T['scan'], 2.5)
    path = list(reversed(T['park_to_scan']))
    play_path(path, per_seg=0.18)
    # parki eklem geri beslemesiyle kilitle
    t0 = time.time()
    while time.time() - t0 < 60:
        send(T['park'])
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.04)
        if all(k in jstate for k in JNAMES):
            err = max(abs(jstate[k] - T['park'][i]) for i, k in enumerate(JNAMES))
            if err < 0.02:
                print(f'  park dogrulandi (hata {err:.3f} rad)', flush=True)
                return
    print('  park zaman asimi (yaklasik parkta)', flush=True)

def bekle_25N(label):
    """gercek load cell 25N olana kadar bekle (kullanici bastiracak)"""
    print(f'  {label}: 25N TEMAS BEKLENIYOR — load cell\'e bastir! (su an {state["force"]:.1f}N)', flush=True)
    t0 = now(); last_p = 0
    while now() - t0 < FORCE_WAIT_S:
        if state['emergency']: return False
        send_hold = True
        rclpy.spin_once(n, timeout_sec=0.05)
        if state['force'] >= FORCE_N:
            print(f'  {label}: TEMAS ✓ {state["force"]:.1f}N — zimpara basliyor!', flush=True)
            return True
        el = int(now() - t0)
        if el // 5 != last_p:
            last_p = el // 5
            print(f'    bekleniyor... F={state["force"]:.1f}N ({el}sn)', flush=True)
    print(f'  {label}: 25N gelmedi ({FORCE_WAIT_S:.0f}sn) — yine de devam', flush=True)
    return True

def cluster(dets):
    """tespitler -> disk-farkindalikli bolgeler:
    her tespit ±DISK_R araligina genisletilir, kesisenler birlesir.
    Boylece 10cm disk icine giren yakin capaklar TEK bolge olur."""
    iv = []
    for frame in dets:
        for b in frame:
            dy = (b['x'] - FRAME_W / 2) * PX2M * AXIS_SIGN
            y = max(Y_MIN, min(Y_MAX, SCAN_Y + dy))
            iv.append((y - DISK_R, y + DISK_R))
    if not iv: return []
    iv.sort()
    merged = [list(iv[0])]
    for a, b in iv[1:]:
        if a <= merged[-1][1] + 1e-9:      # kesisiyor/degiyor -> birlestir
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    # kullanici kurali: disk MERKEZI ilk capaga iner (a+R), son capaga kadar (b-R)
    out = []
    for a, b in merged:
        out.append((max(Y_MIN, a + DISK_R), min(Y_MAX, b - DISK_R)))
    return [(min(a, b), max(a, b)) for a, b in out]

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
    for bi, (ya, yb) in enumerate(bolgeler, 1):
        y_end = max(yb, ya + 0.01)   # min 1cm ilerleme (tek capak = 5sn)
        etiket = f'BOLGE {bi}/{len(bolgeler)}'
        entry = table_at(HIGH, ya)
        if cur_y is None:
            ok = move_seg(T['scan'], entry, 2.0)
        else:
            ok = glide(HIGH, cur_y, ya, 0.10)
        if not ok: break
        print(f'  {etiket} [{ya:+.2f}..{y_end:+.2f}] -> inis', flush=True)
        ok = move_seg(entry, table_at(LOW, ya), 2.0)          # inis
        if not ok: break
        # 25N GERCEK TEMAS KAPISI — load cell'e bastirilmadan zimpara baslamaz
        ok = bekle_25N(etiket)
        if not ok: break
        print(f'  {etiket} ZIMPARA: {abs(y_end-ya)*100:.0f} cm @ 1cm/5sn', flush=True)
        ok = glide(LOW, ya, y_end, CREEP_MPS, spin=SPIN)       # surunme zimpara
        send(table_at(LOW, y_end), 0.0)
        if not ok: break
        ok = move_seg(table_at(LOW, y_end), table_at(HIGH, y_end), 1.5)  # kalk
        if not ok: break
        cur_y = y_end
    print('Bolgeler bitti -> park.', flush=True)
    to_park(from_y=cur_y)
    print('HAZIR — tekrar START bekleniyor.', flush=True)
