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
import json, math, os, sys, time
import rclpy
from std_msgs.msg import Bool, String, Float64MultiArray

# ── GERCEK ROBOT MODU ────────────────────────────────────────────────────
# GERCEK_ROBOT=1 ortam degiskeniyle acilir (gercek_robot.launch.py ayarlar).
# Kapaliyken (sim) davranis BIREBIR eskisi gibidir.
GERCEK = os.environ.get('GERCEK_ROBOT', '0') == '1'

WS = os.path.expanduser('~/ros2-end-effector')
T = json.load(open(os.path.join(WS, 'poz_tablosu.json')))
sys.path.insert(0, WS)
import kumeleme              # kutu-bazli capak eslestirme (saf modul, birim testli)

# ── Ayarlar ──
FRAME_W      = 1280          # vision_node kare genisligi (px)
VIEW_W_M     = 0.72          # 25cm yukseklikte gorus genisligi (m) — TEZGAHTA OLCULDU
                             # (2026-07-23, metreyle ~72cm; eski 0.90 varsayimdi)
PX2M         = VIEW_W_M / FRAME_W
AXIS_SIGN    = -1.0          # goruntu-x -> robot-y isareti (gerekirse +1 yap)
SCAN_Y       = -0.100        # tarama pozunun y'si (yeni esik hattinin merkezi)
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
pub_regions = n.create_publisher(String, '/end_effector/regions', 10)        # GUI canli maske (aday/kesin/bolge)

def log_gui(msg):
    print(msg, flush=True)
    pub_log.publish(String(data=f'[KOL] {msg}'))

def kamera_kutusu(acik):
    """kamera kutusu — GERCEK servo + SIM kapagi (7. eksen) birlikte hareket eder.
    SEMA: mini PC can_node'un ONCELIKLI ve fiilen test edilmis formati
    {'camera': metre} (>0.01 = AC). Eski {'s1','s2'} semasi kodda kabul
    goruluyordu ama tezgahta kapagi FIZIKSEL acmadigi gozlendi (2026-07-22)."""
    pub_servo.publish(String(data=json.dumps(
        {'camera': 0.025 if acik else 0.0, 'sander': 222})))
    state['kutu_sim'] = 0.025 if acik else 0.0

def gercek_zimpara(acik):
    pub_sander.publish(String(data=json.dumps({'sander': 111 if acik else 222})))

FORCE_N      = 25.0          # zimpara baslama sarti (gercek load cell, Newton)
FORCE_WAIT_S = 120.0         # 25N bekleme ust siniri (sim-sn); dolarsa uyariyla devam
DISK_R       = 0.05          # disk yaricapi (kumeleme icin)
# ── TESPIT ESLESTIRME AYARLARI (kullanici tasarimi 2026-07-23 — kolay degisir) ──
KALICILIK_ORAN  = 0.25       # capak, penceredeki karelerin bu ORANINDA gorulmeli
                             # (orn. 15 kare x 0.25 = 4 kare; ardisik olmasi GEREKMEZ)
MIN_GOZLEM      = 3          # ...ama her durumda en az bu kadar karede
KUTU_BUYUTME    = 0.30       # ayni-capak kurali: kutu bu oranla buyutulur,
                             # yeni tespitin MERKEZI icine dusuyorsa ayni capak
VARSAYILAN_KUTU = 53.0       # px — eski {'x'} semasi kutu boyutu gondermezse
MIN_PAS      = 0.05          # tek capak bile en az 5 cm zimparalanir (25 sn)

state = {'emergency': False, 'stop': False, 'start': False, 'dets': [],
         'force': 0.0, 'y_son': None, 'lvl': 'HIGH', 'gorev': False,
         'kutu_sim': 0.0}
n.create_subscription(Bool, '/end_effector/mission_start',
                      lambda m: state.__setitem__('start', state['start'] or m.data), 10)
n.create_subscription(Bool, '/end_effector/emergency_stop',
                      lambda m: state.__setitem__('emergency', state['emergency'] or m.data), 10)
n.create_subscription(Bool, '/end_effector/mission_stop',
                      lambda m: state.__setitem__('stop', state['stop'] or m.data), 10)

def iptal():
    """Gorev kesme kontrolu: EMERGENCY veya STOP.
    Yakalandigi AN gercek role kapatilir (zimpara asla donmeye devam etmez).
    PARK MODUNDA kesme YOK: park donusu zaten en guvenli hareket; yeni gelen
    e-stop mesajlari donusu bastan baslatirsa kol sonsuz dongude kalir
    (olculdu: 'Park donusu kesildi' + 'EMERGENCY' tekrari)."""
    if state.get('park_modu'):
        gercek_zimpara(False)
        return False
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
        onceki = state.get('em_onceki')      # ILK mesaj taban sayilir (None)
        # GOREV SIRASINDA: seviye tetik (basis aninda yakala, kacirma).
        # BOSTA: kenar tetik (mini PC'nin takili kalmis 'true'su park dongusu yapmasin).
        if not e:
            state['em_hazir'] = True      # 'false' gorulmeden seviye tetigi ACILMAZ
        # Gorev sirasinda seviye tetik — AMA once bir kez 'false' gorulmeli;
        # yoksa mini PC'nin onceki gorevden takili kalan 'true'su yeni gorevi
        # daha ilk saniyede iptal ediyordu (olculdu: START -> aninda park).
        if e and ((state.get('gorev') and state.get('em_hazir'))
                  or (onceki is not None and not onceki)):
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

LAG_MAX   = 0.40    # rad — OLCULDU: 2 rad/s'de normal gecikme 0.27; bu esik
                    # sadece GERCEK blokajda devreye girer (komut kolu savurmaz)
BEKLE_MAX = 2.5     # sn  — bu kadar beklendiyse yine de ilerle (kilitlenme kacisi)

def send(q, spin=0.0):
    state['son_komut'] = list(q)
    if GERCEK:
        return          # gercek robot poz akisi kabul etmez; hareket movej'le
    # 7. eksen = kamera kutusu KAPAGI (prismatik 0-0.025m): sim'de de acilip kapanir
    m = Float64MultiArray(); m.data = [float(v) for v in q] + [state.get('kutu_sim', 0.0)]
    pub_j.publish(m)
    z = Float64MultiArray(); z.data = [float(spin)]
    pub_z.publish(z)

def gecikme():
    """son komut ile GERCEK eklem arasindaki en buyuk fark (rad)"""
    if GERCEK: return 0.0            # movej bloklar; gecikme kavrami yok
    q = state.get('son_komut')
    if not q or not all(k in jstate for k in JNAMES): return 0.0
    return max(abs(jstate[k] - q[i]) for i, k in enumerate(JNAMES))

def move_seg(q0, q1, dur, spin=0.0, zorla=False):
    if GERCEK:
        if not zorla and iptal(): return False
        return kol.movej(q1, t=max(float(dur), 0.5))
    """Iki poz arasi kosinuslu gecis — GECIKME KAPILI:
    kol komutun >LAG_MAX gerisine duserse sanal zaman DURUR, komut kolu
    beklemeye alir. Boylece komut kolun onune gecip onu savurmaz."""
    import numpy as np
    q0 = np.array(q0); q1 = np.array(q1)
    ts = 0.0; bekl = 0.0; kacis = False; t_son = time.time()
    while True:
        if not zorla and iptal(): return False
        wt = time.time(); dt = min(wt - t_son, 0.1); t_son = wt
        if kacis or gecikme() < LAG_MAX:               # kol yetisiyorsa ilerle
            ts += dt
        else:
            bekl += dt
            if bekl > BEKLE_MAX: kacis = True         # BLOKE: ilerle, takilma
        a = min(1.0, ts / max(dur, 1e-3))
        a_s = 0.5 - 0.5 * math.cos(math.pi * a)
        send((1 - a_s) * q0 + a_s * q1, spin)
        if a >= 1.0: return True
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.01)

def yol_basina_git(q0, sure=6.0):
    """Yolu oynatmadan ONCE kolu yolun BASINA getir. Kol baska yerdeyse
    yol adimlari kucuk oldugu icin kapi kilitlenirdi; bu onu onler."""
    if all(k in jstate for k in JNAMES):
        e = max(abs(jstate[k] - q0[i]) for i, k in enumerate(JNAMES))
        if e > 0.10:
            # OLCULEN: 0.8 rad/s guvenli rampa; VARANA KADAR bekle (yoksa yol
            # kol daha yoldayken baslar -> surekli gecikme -> surunerek ilerler)
            move_seg([jstate[k] for k in JNAMES], list(q0), max(0.8, e / 1.6), zorla=True)
            t0 = time.time()
            while time.time() - t0 < 5.0:
                send(list(q0)); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.02)
                if all(k in jstate for k in JNAMES) and \
                   max(abs(jstate[k] - q0[i]) for i, k in enumerate(JNAMES)) < 0.12:
                    break
    return True

import bisect

def play_path(path, hiz=1.3, spin=0.0, zorla=False, **_):
    if GERCEK:
        return kol.play_path(path, (lambda: False) if zorla else iptal)
    """Yolu TEK SUREKLI hareket olarak oynat.
    ESKI HATA: her ara pozda kosinus profiliyle DURUP kalkiyordu (161 kez
    dur-kalk = dakikalarca surme). Simdi tum yol tek yay: sadece basta
    hizlanma, sonda yavaslama. hiz = eklem uzayinda tepe hiz (rad/s)."""
    import numpy as np
    yol_basina_git(path[0])
    P = [np.array(q, dtype=float) for q in path]
    d = [0.0]
    for i in range(1, len(P)):
        d.append(d[-1] + float(np.max(np.abs(P[i] - P[i-1]))))
    toplam = d[-1]
    if toplam < 1e-6: return True
    sure = toplam / max(hiz, 0.1)
    ts = 0.0; bekl = 0.0; kacis = False; t_son = time.time()
    while True:
        if not zorla and iptal(): return False
        wt = time.time(); dt = min(wt - t_son, 0.1); t_son = wt
        if kacis or gecikme() < LAG_MAX:
            ts += dt
        else:
            bekl += dt
            if bekl > BEKLE_MAX: kacis = True
        a = min(1.0, ts / sure)
        s = (0.5 - 0.5 * math.cos(math.pi * a)) * toplam     # yumusak baslangic/bitis
        i = max(1, min(bisect.bisect_left(d, s), len(P) - 1))
        seg = d[i] - d[i-1]
        u = 0.0 if seg < 1e-9 else (s - d[i-1]) / seg
        send(P[i-1] + (P[i] - P[i-1]) * u, spin)
        if a >= 1.0: return True
        rclpy.spin_once(n, timeout_sec=0.005); time.sleep(0.008)

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
    if GERCEK:
        adim = 0.008 if speed < 0.01 else 0.02
        ys = [y0 + (y1 - y0) * k / max(1, int(abs(y1 - y0) / adim))
              for k in range(0, max(1, int(abs(y1 - y0) / adim)) + 1)]
        state['y_son'] = y1
        return kol.surun_noktalar([table_at(table, y) for y in ys],
                                  adim / max(speed, 1e-6), iptal)
    """hat boyunca y0->y1, verilen hizla (m/s). wall=True -> duvar saatiyle
    (kullaniciya gercek saniye; cok yavas hizlarda takip guvenli)"""
    dist = abs(y1 - y0); dur = dist / max(speed, 1e-6)
    ts = 0.0; bekl = 0.0; kacis = False; t_son = time.time()
    while True:
        if iptal(): return False
        wt = time.time(); dt = min(wt - t_son, 0.1); t_son = wt
        if kacis or gecikme() < LAG_MAX:               # GECIKME KAPISI (+kacis)
            ts += dt
        else:
            bekl += dt
            if bekl > BEKLE_MAX: kacis = True         # BLOKE: artik ilerle, takilma
        a = min(1.0, ts / max(dur, 1e-3))
        y = y0 + (y1 - y0) * a
        state['y_son'] = y            # iptalde guvenli kalkis icin konum izi
        send(table_at(table, y), spin)
        if a >= 1.0: return True
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.01)

from sensor_msgs.msg import JointState
jstate = {}
# SAHA: gercek robotta eklem durumu /dsr01/joint_states'ten gelir
_JS_TOPIC = '/dsr01/joint_states' if GERCEK else '/gz/joint_states'
n.create_subscription(JointState, _JS_TOPIC,
                      lambda m: jstate.update(zip(m.name, m.position)), 10)
JNAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

kol = None
if GERCEK:
    sys.path.insert(0, os.path.join(WS, 'real_robot'))
    from real_kol_surucu import RealKol
    kol = RealKol(n, lambda t=0.05: rclpy.spin_once(n, timeout_sec=t))
    print('>>> GERCEK ROBOT MODU: hareketler Doosan surucusune gidiyor', flush=True)

def parkta():
    """kol su an park pozunda mi? (eklem geri beslemesinden)"""
    if not all(k in jstate for k in JNAMES):
        return False
    return max(abs(jstate[k] - T['park'][i]) for i, k in enumerate(JNAMES)) < 0.05

def _en_yakin_koridor():
    """Kolun SU ANKI pozuna en yakin koridor adimi (indeks, fark).
    Acil durumda kol koridorun ortasinda olabilir; bastan oynatmak yerine
    en yakin noktadan GERI oynatmak hem hizli hem carpmasiz."""
    if not all(k in jstate for k in JNAMES): return None, 9.9
    cur = [jstate[k] for k in JNAMES]
    en_i, en_d = 0, 9.9
    for i, q in enumerate(T['park_to_scan']):
        d = max(abs(cur[j] - q[j]) for j in range(6))
        if d < en_d: en_d, en_i = d, i
    return en_i, en_d

def to_park(from_y=None, acele=False):
    """HER POZDAN guvenli park donusu:
      1) kol asagidaysa once DIKEY kalkis
      2) koridora en yakin adim bulunur, oradan GERI oynatilir
      3) koridordan uzaksa once tarama pozuna hizalanir
    acele=True: EMERGENCY icin daha hizli."""
    # koridor artik 161 poz (yogunlastirildi, MAX 0.12 rad/adim) — gecikme
    # kapisi guvenligi sagladigi icin adim suresi kisa tutulabilir
    # OLCULEN takip: 1.5 rad/s -> 0.21 rad gecikme, 2.0 -> 0.27 (kazanc 12).
    # Koridor adimi 0.12 rad: segp 0.08 = 1.5 rad/s (guvenli), 0.06 = 2.0 (acil).
    hiz  = 0.30 if acele else 0.22     # her sey bitince park HEMEN olsun
    segs = 0.9  if acele else 1.4
    segp = 0.06 if acele else 0.08
    if parkta():
        state['emergency'] = False; state['stop'] = False
        return                          # zaten parkta — kimildama, sicrama yok
    state['park_modu'] = True           # bu donus KESILMEZ
    try:
        _park_git(from_y, hiz, segs, acele)
    finally:
        state['park_modu'] = False
        state['emergency'] = False; state['stop'] = False   # yankilari yut

def _park_git(from_y, hiz, segs, acele):
    # 1) kol ASAGIDAYSA once dikey kalk (yatay hareket sasiyi sıyırmasin)
    if state.get('lvl') == 'LOW' and state.get('y_son') is not None:
        ys = state['y_son']
        move_seg([jstate[k] for k in JNAMES] if all(k in jstate for k in JNAMES)
                 else karisim(ys, 1.0), table_at(HIGH, ys), 0.7, zorla=True)
        state['lvl'] = 'HIGH'
    # 2) koridora EN YAKIN adimdan geri oyna (kisa yol = hizli donus)
    i, d = _en_yakin_koridor()
    if i is not None and d < 0.6:
        path = list(reversed(T['park_to_scan'][:i+1]))
    else:
        # 3) koridordan uzak: once HIGH hatti + tarama pozu
        if from_y is not None:
            glide(HIGH, from_y, SCAN_Y, hiz)
        move_seg([jstate[k] for k in JNAMES] if all(k in jstate for k in JNAMES)
                 else table_at(HIGH, SCAN_Y), T['scan'], segs, zorla=True)
        path = list(reversed(T['park_to_scan']))
    if not play_path(path, hiz=(2.2 if acele else 1.5)):
        # koridor sirasinda YENI acil durum: oldugun yerde SABITLEN (yanki yok)
        if all(k in jstate for k in JNAMES):
            dur = [jstate[k] for k in JNAMES]
            t0 = time.time()
            while time.time() - t0 < 3.0:
                send(dur); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
        log_gui('Park donusu kesildi — kol guvenli sekilde sabitlendi')
        return
    # parki eklem geri beslemesiyle kilitle
    t0 = time.time()
    while time.time() - t0 < 8:        # kisa kilit (kol zaten yolun sonunda)
        send(T['park'])
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.02)
        if all(k in jstate for k in JNAMES):
            err = max(abs(jstate[k] - T['park'][i]) for i, k in enumerate(JNAMES))
            if err < 0.08:
                log_gui(f'PARK ✓ ({time.time()-t0:.1f} sn)')
                return
    log_gui('PARK (yaklasik)')

def _kullanilmiyor_bekle_25N(label, pose):  # ESKI YONTEM — asamali_inis geldi, cagrilmiyor
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
    log_gui(f'{label}: dara={dara:.1f} alindi -> SIMDI BASTIR (+25N ustu yeterli)')
    t0 = time.time(); last_p = -1; ardarda = 0; son_uyari = 0.0
    while time.time() - t0 < 60.0:
        if iptal(): return False
        send(pose)                      # pozu tut
        rclpy.spin_once(n, timeout_sec=0.05)
        d = state['force'] - dara       # dara ustu NET kuvvet
        if d >= FORCE_N:                # +25N ustu HER basis temas sayilir
                                        # (hucre olcegi buyuk: normal basis ~+700N okunuyor)
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
    return False

SPAN = 0.151   # HIGH(z=0.600) ile LOW(z=0.449) arasi dusey mesafe (m)
               # esik ust yuzeyi z=0.439 -> LOW'da disk esige DEGER

def bekle_varis(q, tol=0.05, sure=6.0, etiket=''):
    if GERCEK:
        return True     # movej bloklayarak vardi; ayrica beklemeye gerek yok
    """Hedef pozu YAYINLAMAYA devam ederek eklemlerin gercekten VARMASINI bekle.
    Kritik gecislerde sart: komut ilerledi ama kol geride kaldiysa, bir sonraki
    hareket kolu capraz suruklerdi -> sasiye/yere carpma. (bolge gecisi bugu)"""
    t0 = time.time()
    while time.time() - t0 < sure:
        if iptal(): return False
        send(q)
        rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.02)
        if all(k in jstate for k in JNAMES):
            err = max(abs(jstate[k] - q[i]) for i, k in enumerate(JNAMES))
            if err < tol:
                return True
    if etiket: log_gui(f'{etiket}: varis zaman asimi (yaklasik hedefte)')
    return True

def karisim(y, a):
    """HIGH ve LOW hatlari arasinda (a=0 HIGH, a=1 LOW) eklem-uzayi karisimi.
    a>1 = LOW'un ALTINA dogru kucuk ekstrapolasyon (bosluk aramasi icin)."""
    import numpy as np
    qh = np.array(table_at(HIGH, y)); ql = np.array(table_at(LOW, y))
    return list(qh + (ql - qh) * a)

def asamali_inis(etiket, ya):
    """KULLANICI KURALI — temas mantigi inise gomulu (operator basisi beklenmez):
      1) Zimpara yuksekliginin 5cm USTUNE gel, dara al, kuvvet izlemeye basla
      2) 5cm in: bu sirada net >= 25N olursa TEMAS (inis durur, zimpara baslar)
      3) 5cm sonunda tepki varsa (net >= 10N) 25N'a kadar derinlesmeye devam
      4) Tepki yoksa +5cm daha in (toplam 10cm)
      5) Hala tepki yoksa -> BOSLUK: zimpara yapilmaz, geri cekilme
    Donus: ('TEMAS'|'BOS'|'IPTAL', a)"""
    if GERCEK:
        def dara_al():
            orn = []
            t0 = time.time()
            while time.time() - t0 < 0.6:
                rclpy.spin_once(n, timeout_sec=0.05); orn.append(state['force'])
            orn.sort(); return orn[len(orn)//2] if orn else 0.0
        def kuvvet():
            rclpy.spin_once(n, timeout_sec=0.02); return state['force']
        return kol.inis_asamali(lambda a: karisim(ya, a), dara_al, kuvvet,
                                iptal, lambda m: log_gui(f'{etiket}: {m}'),
                                esik=FORCE_N, tepki=10.0, span=SPAN)
    a_ust = 1 - 0.05 / SPAN            # LOW'un 5cm ustu
    a_alt = 1 + 0.05 / SPAN            # LOW'un 5cm alti
    hedef = karisim(ya, a_ust)
    if not move_seg(table_at(HIGH, ya), hedef, 1.5):
        return ('IPTAL', a_ust)
    bekle_varis(hedef, 0.04, 4.0)          # 5cm ustunde GERCEKTEN dur
    orn = []; t0 = time.time()
    while time.time() - t0 < 0.5:      # dara: bosta okuma (kisa)
        send(hedef); rclpy.spin_once(n, timeout_sec=0.03)
        orn.append(state['force'])
    orn.sort(); dara = orn[len(orn)//2] if orn else 0.0
    log_gui(f'{etiket}: iniyor (temas icin kuvvet izleniyor)')

    def in_ve_izle(a0, a1, sure):
        """a0'dan a1'e in; temas olursa TEMASIN OLDUGU DERINLIGI dondur."""
        t0 = time.time()
        while True:
            if iptal(): return ('IPTAL', a0)
            t = (time.time() - t0) / sure
            if t >= 1.0: return None
            a_su = a0 + (a1 - a0) * t
            send(karisim(ya, a_su))
            rclpy.spin_once(n, timeout_sec=0.03); time.sleep(0.02)
            if gecikme() > LAG_MAX:      # kol INIYEMIYOR = fiziksel engel
                log_gui(f'{etiket}: kol bloke ({gecikme():.2f} rad) — temas kabul, inis durdu')
                return ('TEMAS', a_su)
            d = state['force'] - dara
            if d >= FORCE_N:
                log_gui(f'{etiket}: TEMAS ✓ net +{d:.1f}N — inis durdu, zimpara basliyor')
                return ('TEMAS', a_su)

    r = in_ve_izle(a_ust, 1.0, 4.0)            # asama 1: ilk 5cm
    if r: return r
    d = state['force'] - dara
    if d >= 10.0:
        log_gui(f'{etiket}: tepki var (+{d:.1f}N) — 25N icin derinlesiliyor')
        r = in_ve_izle(1.0, a_alt, 4.0)
        if r: return r
        d = state['force'] - dara
        if d >= 10.0:
            log_gui(f'{etiket}: 25N tam olusmadi ama temas belirgin (+{d:.1f}N) — zimpara basliyor')
            return ('TEMAS', a_alt)
        return ('BOS', a_alt)
    log_gui(f'{etiket}: 5cm indi, load cell TEPKISIZ — +5cm daha araniyor')
    r = in_ve_izle(1.0, a_alt, 4.0)            # asama 2: toplam 10cm
    if r: return r
    d = state['force'] - dara
    if d >= 10.0:
        log_gui(f'{etiket}: gec tepki (+{d:.1f}N) — temas kabul, zimpara basliyor')
        return ('TEMAS', a_alt)
    log_gui(f'{etiket}: 10cm inildi, load cell degismedi — zimpara BOSLUKTA')
    return ('BOS', a_alt)

def glide_blend(y0, y1, a, speed, spin=0.0, wall=False):
    if GERCEK:
        adim = 0.008
        npts = max(1, int(abs(y1 - y0) / adim))
        ys = [y0 + (y1 - y0) * k / npts for k in range(npts + 1)]
        state['y_son'] = y1
        # kalp: zimpara surerken roleyi ~4sn'de bir TAZELE (mini PC watchdog'u
        # 12sn tazeleme gormezse otomatik keser — beyin olurse role acik kalmaz)
        return kol.surun_noktalar([karisim(y, a) for y in ys],
                                  adim / max(speed, 1e-6), iptal,
                                  kalp=lambda: gercek_zimpara(True))
    """TEMASIN OLDUGU derinlikte (a) y0->y1 surun. Eskiden zimpara her zaman
    LOW seviyesine zorlanirdi; temas LOW'un ustunde olustuysa bu ANI DALIS
    demekti (sasiye/yere carpmanin ikinci kaynagi)."""
    dist = abs(y1 - y0); dur = dist / max(speed, 1e-6)
    ts = 0.0; bekl = 0.0; kacis = False; t_son = time.time()
    while True:
        if iptal(): return False
        wt = time.time(); dt = min(wt - t_son, 0.1); t_son = wt
        if kacis or gecikme() < LAG_MAX:               # GECIKME KAPISI (+kacis)
            ts += dt
        else:
            bekl += dt
            if bekl > BEKLE_MAX: kacis = True         # BLOKE: artik ilerle, takilma
        p = min(1.0, ts / max(dur, 1e-3))
        y = y0 + (y1 - y0) * p
        state['y_son'] = y
        send(karisim(y, a), spin)
        if p >= 1.0: return True
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.01)

def _bolgeler_birlestir(ys):
    """Sirali y listesi -> bolgeler: bosluk <= GAP_M (disk capi) ayni iniste.
    (METRIK kalan tek kural — disk capi fiziksel sabittir.)"""
    if not ys:
        return []
    b = [[ys[0], ys[0]]]
    for y in ys[1:]:
        if y - b[-1][1] <= GAP_M:
            b[-1][1] = y
        else:
            b.append([y, y])
    return [(a, c) for a, c in b]


def _px_to_y(cx):
    return SCAN_Y + (cx - FRAME_W / 2) * PX2M * AXIS_SIGN


def _regions_yayinla(son=False):
    """GUI canli maskesi: aday (esik alti, soluk) + kesin capaklar + bolge
    seritleri. Pencere SIRASINDA ~0.5sn'de bir, sonunda 'kesin' olarak yayinlanir."""
    dets = state['dets']
    kumeler = kumeleme.kumele(dets, KUTU_BUYUTME, VARSAYILAN_KUTU)
    esik = max(MIN_GOZLEM, int(round(len(dets) * KALICILIK_ORAN)))
    adaylar, kesin, ys = [], [], []
    for k in kumeler:
        y = _px_to_y(k['cx'])
        rec = {'x': round(k['cx'], 1), 'y': round(k['cy'], 1),
               'w': round(k['w'], 1), 'h': round(k['h'], 1), 'n': k['n']}
        if k['n'] >= esik and (Y_MIN - 0.02) <= y <= (Y_MAX + 0.02):
            kesin.append(rec)
            ys.append(min(max(y, Y_MIN), Y_MAX))
        else:
            adaylar.append(rec)
    ys.sort()
    bolgeler = _bolgeler_birlestir(ys)
    bpx = []
    for a, b in bolgeler:
        xa = FRAME_W / 2 + (a - SCAN_Y) / (PX2M * AXIS_SIGN)
        xb = FRAME_W / 2 + (b - SCAN_Y) / (PX2M * AXIS_SIGN)
        bpx.append([round(min(xa, xb), 1), round(max(xa, xb), 1)])
    pub_regions.publish(String(data=json.dumps({
        'tip': 'kesin' if son else 'onizleme',
        'kare': len(dets), 'esik': esik,
        'adaylar': adaylar[:30], 'capaklar': kesin,
        'bolgeler_px': bpx,
        'bolgeler_m': [[round(a, 3), round(b, 3)] for a, b in bolgeler]})))


def cluster(dets):
    """Kare-tespitleri -> CAPAKLAR -> bolgeler.  (v2 — KUTU-BAZLI eslesme)
    1) Kimlik: kumeleme.kumele — yeni tespitin merkezi, kumenin %KUTU_BUYUTME
       buyutulmus MEDYAN kutusuna dusuyorsa ayni capak (sabit cm YOK).
    2) Kalicilik: karelerin KALICILIK_ORAN'inda (>= MIN_GOZLEM) gorulme sarti;
       kareler ardisik olmak zorunda degil. Harita disina dusenler cope.
    3) Bolge: capaklar arasi bosluk <= GAP_M (disk capi) ayni iniste taranir.
    Donus: (bolgeler[(y0,y1)], noktalar[(y, px)])"""
    kumeler = kumeleme.kumele(dets, KUTU_BUYUTME, VARSAYILAN_KUTU)
    esik = max(MIN_GOZLEM, int(round(len(dets) * KALICILIK_ORAN)))
    noktalar = []
    for i, k in enumerate(kumeler):
        y = _px_to_y(k['cx'])
        icerde = (Y_MIN - 0.02) <= y <= (Y_MAX + 0.02)
        gecti = k['n'] >= esik and icerde
        if i < 12:                                  # teshis dokumu (sinirli)
            durum = 'CAPAK ✓' if gecti else ('harita-disi' if not icerde else 'elendi')
            log_gui(f"  kume: x={k['cx']:.0f}px kutu={k['w']:.0f}x{k['h']:.0f} "
                    f"gozlem={k['n']}/{esik} -> {durum}")
        if gecti:
            noktalar.append((min(max(y, Y_MIN), Y_MAX), float(k['cx'])))
    if len(kumeler) > 12:
        log_gui(f'  (+{len(kumeler)-12} kume daha)')
    if not noktalar:
        return [], []
    noktalar.sort()
    bolgeler = _bolgeler_birlestir([y for y, _ in noktalar])
    return bolgeler, noktalar

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
    state['gorev'] = True        # gorev basladi: e-stop artik SEVIYE tetikli
    state['em_hazir'] = False    # ...ama once mini PC'nin 'false' demesini bekle
    state['lvl'] = 'HIGH'; state['y_son'] = None
    gercek_zimpara(False)        # onceki kosudan takili kalmis roleyi KAPAT
    log_gui('START alindi -> kol tarama pozuna gidiyor (esik +25cm)...')
    if not play_path(T['park_to_scan'], hiz=1.6):        # tek surekli hareket
        to_park(acele=True); continue
    bekle_varis(T['scan'], 0.06, 6.0, 'TARAMA')          # gercekten VARDIGINI dogrula
    log_gui('  tarama pozunda ✓')

    # KOL VARDI -> simdi GERCEK kamera kutusunu ac (tek beyin senkronu)
    log_gui('Kol tarama pozunda ✓ — kamera kutusu ACILIYOR...')
    kamera_kutusu(True)
    t0 = time.time()
    while time.time() - t0 < 6.0:      # kutunun fiziksel acilmasi
        if iptal(): break
        send(T['scan']); rclpy.spin_once(n, timeout_sec=0.05)

    log_gui(f'Kamera penceresi: {CAM_WINDOW_S:.0f} sn gercek tespit toplaniyor...')
    state['dets'] = []                  # pencere oncesi kareleri sayma
    t0 = time.time(); son_oniz = 0.0
    while time.time() - t0 < CAM_WINDOW_S:
        if iptal(): break
        send(T['scan'])
        if time.time() - son_oniz >= 0.5:   # CANLI maske: ara-kumeleme -> GUI
            _regions_yayinla(son=False)
            son_oniz = time.time()
        rclpy.spin_once(n, timeout_sec=0.05)

    log_gui('Kamera kutusu kapaniyor...')
    kamera_kutusu(False)
    if iptal():
        acil = state['emergency']
        state['emergency'] = False; state['stop'] = False; state['gorev'] = False
        if acil:
            log_gui('EMERGENCY — PARK\'a donuluyor'); to_park(acele=True)
        else:
            log_gui('STOP — kol oldugu yerde bekliyor')
        continue

    bolgeler, noktalar = cluster(state['dets'])
    _regions_yayinla(son=True)          # GUI maskesi: kesinlesmis hal
    log_gui(f'{len(state["dets"])} karede {sum(len(f) for f in state["dets"])} tespit -> '
            f'{len(noktalar)} capak noktasi -> {len(bolgeler)} bolge')
    for y, px in noktalar:
        log_gui(f'  capak: goruntu x={px:.0f}px (%{100*px/FRAME_W:.0f}) -> y={y:+.3f} m')
    for i, (a, b) in enumerate(bolgeler, 1):
        log_gui(f'  bolge {i}: [{a:+.3f} .. {b:+.3f}] m ({abs(b-a)*100:.0f} cm)')

    if not bolgeler:
        print('Tespit YOK -> geri cekiliyor, park.', flush=True)
        to_park(); continue

    state['dets'] = []               # pencere sonrasi kareler birikmesin
    ok = True
    cur_y = None
    state['lvl'] = 'HIGH'; state['y_son'] = SCAN_Y
    for bi, (ya, yb) in enumerate(bolgeler, 1):
        # Tek capakta bile disk capinin yarisi kadar pas at (5cm = 25 sn):
        # 1cm'lik pas gorsel olarak "hic zimparalamadi" gibi duruyordu.
        y_end = ya + max(yb - ya, MIN_PAS)
        etiket = f'BOLGE {bi}/{len(bolgeler)}'
        entry = table_at(HIGH, ya)
        state['y_son'] = ya
        if cur_y is None:
            ok = move_seg(T['scan'], entry, 2.0)
        else:
            ok = glide(HIGH, cur_y, ya, 0.10)
        if not ok: break
        # TASIMA SONU KAPISI: bolge girisine GERCEKTEN varmadan inise baslama
        if not bekle_varis(entry, 0.05, 6.0, etiket): ok = False; break
        log_gui(f'{etiket} [{ya:+.2f}..{y_end:+.2f}] -> ASAMALI INIS')
        son, a_t = asamali_inis(etiket, ya)
        if son == 'IPTAL':
            ok = False; break
        if son == 'BOS':
            log_gui(f'{etiket}: zimpara yapilmadan GERI CEKILIYOR (bosluk)')
            move_seg(karisim(ya, a_t), table_at(HIGH, ya), 1.8, zorla=True)
            state['lvl'] = 'HIGH'
            bekle_varis(table_at(HIGH, ya), 0.05, 6.0, etiket)
            cur_y = ya
            continue
        state['lvl'] = 'LOW'
        log_gui(f'{etiket} ZIMPARA: {abs(y_end-ya)*100:.0f} cm @ 1cm/5sn (gercek role ACIK)')
        gercek_zimpara(True)                                   # GERCEK role calisir!
        # TEMAS DERINLIGINDE surun (LOW'a zorlama yok -> ani dalis yok)
        ok = glide_blend(ya, y_end, a_t, CREEP_MPS, spin=SPIN, wall=True)
        gercek_zimpara(False)
        send(karisim(y_end, a_t), 0.0)
        if not ok: break
        log_gui(f'{etiket} tamam -> kalkis')
        ok = move_seg(karisim(y_end, a_t), table_at(HIGH, y_end), 1.5)  # kalk
        if ok: state['lvl'] = 'HIGH'
        if not ok: break
        # KALKIS KAPISI: HIGH'a cikmadan yatay tasima YOK (sasiye surunmenin koku)
        bekle_varis(table_at(HIGH, y_end), 0.05, 6.0, etiket)
        cur_y = y_end
    gercek_zimpara(False)   # her ihtimale karsi role kapali
    if state['emergency'] or state['stop']:
        # GUVENLI IPTAL: (1) role/kutu kapali, (2) kol LOW'daysa once DIKEY
        # kalkis (zorla=True: iptal bayragi kalkisi engelleyemez), (3) bayraklar
        # TEMIZLENIR ki park koridoru kesintisiz oynasin — yoksa kol koridoru
        # atlayip dogrudan park referansina sicrar (tehlikeli).
        acil = state['emergency']
        kamera_kutusu(False)
        ys = state['y_son'] if state['y_son'] is not None else SCAN_Y
        if state['lvl'] == 'LOW':                    # her iki durumda da once KALK
            move_seg(table_at(LOW, ys), table_at(HIGH, ys), 0.8, zorla=True)
            state['lvl'] = 'HIGH'
        state['emergency'] = False; state['stop'] = False; state['gorev'] = False
        if acil:
            log_gui('EMERGENCY! her sey durdu — PARK\'a donuluyor')
            to_park(from_y=ys, acele=True)
            log_gui('EMERGENCY tamam — kol parkta')
        else:
            log_gui('STOP! her sey durdu — kol OLDUGU YERDE bekliyor (park yok)')
    else:
        state['gorev'] = False
        log_gui('Tum bolgeler bitti -> PARK\'a donuluyor')
        to_park(from_y=cur_y)
        log_gui('GOREV TAMAM ✓ — tekrar START bekleniyor')
    state['gorev'] = False
    state['start'] = False       # gorev sirasinda birikmis basislar sayilmasin
