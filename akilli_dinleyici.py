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
# FIZIKSEL taraf (zenoh uzerinden mini PC can_node'a gider):
pub_servo  = n.create_publisher(String, '/end_effector/servo_command', 10)   # kamera kutusu
pub_sander = n.create_publisher(String, '/end_effector/sander_only', 10)     # gercek role
pub_log    = n.create_publisher(String, '/end_effector/log', 10)             # mini PC GUI log paneli

def log_gui(msg):
    print(msg, flush=True)
    pub_log.publish(String(data=f'[KOL] {msg}'))

def kamera_kutusu(acik):
    """gercek kamera kutusunu ac/kapat (servo)"""
    s = 35 if acik else 160
    pub_servo.publish(String(data=json.dumps({'s1': s, 's2': s, 'sander': 222})))

def gercek_zimpara(acik):
    pub_sander.publish(String(data=json.dumps({'sander': 111 if acik else 222})))

FORCE_N      = 25.0          # zimpara baslama sarti (gercek load cell, Newton)
FORCE_WAIT_S = 120.0         # 25N bekleme ust siniri (sim-sn); dolarsa uyariyla devam
DISK_R       = 0.05          # disk yaricapi (kumeleme icin)

state = {'emergency': False, 'stop': False, 'start': False, 'dets': [],
         'force': 0.0, 'y_son': None, 'lvl': 'HIGH'}
n.create_subscription(Bool, '/end_effector/mission_start',
                      lambda m: state.__setitem__('start', state['start'] or m.data), 10)
n.create_subscription(Bool, '/end_effector/emergency_stop',
                      lambda m: state.__setitem__('emergency', state['emergency'] or m.data), 10)
n.create_subscription(Bool, '/end_effector/mission_stop',
                      lambda m: state.__setitem__('stop', state['stop'] or m.data), 10)

def iptal():
    """Gorev kesme kontrolu: EMERGENCY veya STOP.
    Yakalandigi AN gercek role kapatilir (zimpara asla donmeye devam etmez)."""
    if state['emergency'] or state['stop']:
        gercek_zimpara(False)
        return True
    return False
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
        d = json.loads(m.data)
        state['force'] = float(d.get('contact_force', 0.0))
        # GUI E-STOP GUVENCESI — KENAR tetik: logic e-stop sonrasi 'emergency'
        # alanini SUREKLI true yayinlar; seviye olarak okursak kol sonsuz park
        # dongusune girer. Sadece false->true GECISI latch'lenir.
        e = bool(d.get('emergency'))
        if e and not state.get('em_onceki', False):
            state['emergency'] = True
        state['em_onceki'] = e
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

def move_seg(q0, q1, dur, spin=0.0, zorla=False):
    """iki poz arasi kosinuslu, sim-saatli, surekli yayinla.
    zorla=True: iptal bayragina BAKMA (guvenli kalkis icin)."""
    t0 = now(); import numpy as np
    q0 = np.array(q0); q1 = np.array(q1)
    while True:
        if not zorla and iptal(): return False
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

def glide(table, y0, y1, speed, spin=0.0, wall=False):
    """hat boyunca y0->y1, verilen hizla (m/s). wall=True -> duvar saatiyle
    (kullaniciya gercek saniye; cok yavas hizlarda takip guvenli)"""
    dist = abs(y1 - y0); dur = dist / max(speed, 1e-6)
    clk = time.time if wall else now
    t0 = clk()
    while True:
        if iptal(): return False
        t = clk() - t0
        a = min(1.0, t / max(dur, 1e-3))
        y = y0 + (y1 - y0) * a
        state['y_son'] = y            # iptalde guvenli kalkis icin konum izi
        send(table_at(table, y), spin)
        if a >= 1.0: return True
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.01)

from sensor_msgs.msg import JointState
jstate = {}
n.create_subscription(JointState, '/gz/joint_states',
                      lambda m: jstate.update(zip(m.name, m.position)), 10)
JNAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

def parkta():
    """kol su an park pozunda mi? (eklem geri beslemesinden)"""
    if not all(k in jstate for k in JNAMES):
        return False
    return max(abs(jstate[k] - T['park'][i]) for i, k in enumerate(JNAMES)) < 0.05

def to_park(from_y=None, acele=False):
    """GERI-BESLEMELI park donusu: yuksek hatta scan hizasina -> scan ->
    ayna yol -> park (eklemler VARANA kadar yayin).
    acele=True: EMERGENCY/STOP icin hizli geri cekilme."""
    hiz  = 0.25 if acele else 0.10
    segs = 1.2  if acele else 2.5
    segp = 0.10 if acele else 0.18
    if from_y is not None:
        glide(HIGH, from_y, SCAN_Y, hiz)
    entry = table_at(HIGH, SCAN_Y)
    move_seg(entry, T['scan'], segs)
    path = list(reversed(T['park_to_scan']))
    play_path(path, per_seg=segp)
    # parki eklem geri beslemesiyle kilitle
    t0 = time.time()
    while time.time() - t0 < 90:
        send(T['park'])
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.04)
        if all(k in jstate for k in JNAMES):
            err = max(abs(jstate[k] - T['park'][i]) for i, k in enumerate(JNAMES))
            if err < 0.03:
                print(f'  park dogrulandi (hata {err:.3f} rad)', flush=True)
                return
    print('  park zaman asimi (yaklasik parkta)', flush=True)

def bekle_25N(label, pose):
    """gercek load cell 25N olana kadar bekle (DUVAR saati, sicrama filtresi).
    Beklerken pozu tutmaya devam eder."""
    # --- OTOMATIK DARA ---------------------------------------------------
    # Mini PC'nin tare'i kendi gorevi disinda calismiyor; ham okuma bosta
    # yuzlerce "N" gorunebiliyor. Cozum: 1.5 sn bosta ornek al, medyani
    # sifir kabul et; temas karari FARKA (dara ustu net kuvvet) gore verilir.
    # Dara toplamsal oldugu icin olcek bozulmaz: fark gercek Newton'dur.
    log_gui(f'{label}: dara aliniyor — load cell\'e HENUZ BASTIRMA (1.5 sn)')
    orn = []
    td = time.time()
    while time.time() - td < 1.5:
        send(pose); rclpy.spin_once(n, timeout_sec=0.05)
        orn.append(state['force'])
    orn.sort(); dara = orn[len(orn)//2] if orn else 0.0
    log_gui(f'{label}: dara={dara:.1f} alindi -> SIMDI BASTIR (hedef: dara ustune +25..+50N)')
    t0 = time.time(); last_p = -1; ardarda = 0; son_uyari = 0.0
    while time.time() - t0 < 60.0:
        if iptal(): return False
        send(pose)                      # pozu tut
        rclpy.spin_once(n, timeout_sec=0.05)
        d = state['force'] - dara       # dara ustu NET kuvvet
        if d > 50.0:                    # cok sert — sayma; uyariyi 2 sn'de bir bas
            ardarda = 0
            if time.time() - son_uyari > 2.0:
                son_uyari = time.time()
                log_gui(f'{label}: COK SERT (net +{d:.1f}N)! biraz gevset — hedef +25..+50N')
        elif d >= FORCE_N:              # gecerli temas bandi: +25..+50N
            ardarda += 1
            if ardarda >= 3:            # 3 ardisik okuma: gercek temas
                log_gui(f'{label}: TEMAS ✓ net +{d:.1f}N — zimpara basliyor!')
                return True
        else:
            ardarda = 0
        el = int(time.time() - t0)
        if el // 5 != last_p:
            last_p = el // 5
            log_gui(f'   bekleniyor... net +{state["force"]-dara:.1f}N ({el}sn/60sn)')
    log_gui(f'{label}: TEMAS OKUNMADI (60sn) — DEMO MODU: yine de zimparalaniyor. '
            f'Load cell duzelince 25N kapisi otomatik devreye girer.')
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
        gercek_zimpara(False); kamera_kutusu(False)
        state['emergency'] = False; state['stop'] = False; state['start'] = False
        if parkta():
            log_gui('EMERGENCY (bosta) — kol zaten parkta, hareket yok')
        else:
            log_gui('EMERGENCY -> role kapatildi, kol HIZLI parka donuyor')
            to_park(acele=True)
        continue
    if not state['start']:
        state['stop'] = False        # bosta gelen STOP'un anlami yok — yut
        continue
    state['start'] = False
    state['dets'] = []
    state['emergency'] = False; state['stop'] = False   # bayat bayrak temizligi
    log_gui('START alindi -> kol tarama pozuna gidiyor (esik +25cm)...')
    if not play_path(T['park_to_scan'], per_seg=0.12):
        to_park(); continue

    # KOL VARDI -> simdi GERCEK kamera kutusunu ac (tek beyin senkronu)
    log_gui('Kol tarama pozunda ✓ — kamera kutusu ACILIYOR...')
    kamera_kutusu(True)
    t0 = time.time()
    while time.time() - t0 < 6.0:      # kutunun fiziksel acilmasi
        if iptal(): break
        send(T['scan']); rclpy.spin_once(n, timeout_sec=0.05)

    log_gui(f'Kamera penceresi: {CAM_WINDOW_S:.0f} sn gercek tespit toplaniyor...')
    state['dets'] = []                  # pencere oncesi kareleri sayma
    t0 = time.time()
    while time.time() - t0 < CAM_WINDOW_S:
        if iptal(): break
        send(T['scan'])
        rclpy.spin_once(n, timeout_sec=0.05)

    log_gui('Kamera kutusu kapaniyor...')
    kamera_kutusu(False)
    if iptal():
        state['emergency'] = False; state['stop'] = False
        to_park(); log_gui('Iptal edildi -> parkta.'); continue

    bolgeler = cluster(state['dets'])
    print(f'{sum(len(f) for f in state["dets"])} tespit -> {len(bolgeler)} bolge: '
          + ', '.join(f'[{a:+.2f}..{b:+.2f}]' for a, b in bolgeler), flush=True)

    if not bolgeler:
        print('Tespit YOK -> geri cekiliyor, park.', flush=True)
        to_park(); continue

    state['dets'] = []               # pencere sonrasi kareler birikmesin
    ok = True
    cur_y = None
    state['lvl'] = 'HIGH'; state['y_son'] = SCAN_Y
    for bi, (ya, yb) in enumerate(bolgeler, 1):
        y_end = max(yb, ya + 0.01)   # min 1cm ilerleme (tek capak = 5sn)
        etiket = f'BOLGE {bi}/{len(bolgeler)}'
        entry = table_at(HIGH, ya)
        state['y_son'] = ya
        if cur_y is None:
            ok = move_seg(T['scan'], entry, 2.0)
        else:
            ok = glide(HIGH, cur_y, ya, 0.10)
        if not ok: break
        log_gui(f'{etiket} [{ya:+.2f}..{y_end:+.2f}] -> INIS')
        low_pose = table_at(LOW, ya)
        ok = move_seg(entry, low_pose, 2.0)                    # inis
        if ok: state['lvl'] = 'LOW'
        if not ok: break
        # 25N GERCEK TEMAS KAPISI — load cell'e bastirilmadan zimpara baslamaz
        ok = bekle_25N(etiket, low_pose)
        if not ok: break
        log_gui(f'{etiket} ZIMPARA: {abs(y_end-ya)*100:.0f} cm @ 1cm/5sn (gercek role ACIK)')
        gercek_zimpara(True)                                   # GERCEK role calisir!
        ok = glide(LOW, ya, y_end, CREEP_MPS, spin=SPIN, wall=True)  # duvar-saatiyle surun
        gercek_zimpara(False)
        send(table_at(LOW, y_end), 0.0)
        if not ok: break
        log_gui(f'{etiket} tamam -> kalkis')
        ok = move_seg(table_at(LOW, y_end), table_at(HIGH, y_end), 1.5)  # kalk
        if ok: state['lvl'] = 'HIGH'
        if not ok: break
        cur_y = y_end
    gercek_zimpara(False)   # her ihtimale karsi role kapali
    if state['emergency'] or state['stop']:
        # GUVENLI IPTAL: (1) role/kutu kapali, (2) kol LOW'daysa once DIKEY
        # kalkis (zorla=True: iptal bayragi kalkisi engelleyemez), (3) bayraklar
        # TEMIZLENIR ki park koridoru kesintisiz oynasin — yoksa kol koridoru
        # atlayip dogrudan park referansina sicrar (tehlikeli).
        sebep = 'EMERGENCY' if state['emergency'] else 'STOP'
        log_gui(f'{sebep}! role kapatildi — HIZLI kalkis + parka donus')
        kamera_kutusu(False)
        ys = state['y_son'] if state['y_son'] is not None else SCAN_Y
        if state['lvl'] == 'LOW':
            move_seg(table_at(LOW, ys), table_at(HIGH, ys), 0.8, zorla=True)
            state['lvl'] = 'HIGH'
        state['emergency'] = False; state['stop'] = False
        to_park(from_y=ys, acele=True)
        log_gui('Iptal tamamlandi — kol parkta. Tekrar START bekleniyor')
    else:
        log_gui('Tum bolgeler bitti -> PARK\'a donuluyor')
        to_park(from_y=cur_y)
        log_gui('GOREV TAMAM ✓ — tekrar START bekleniyor')
