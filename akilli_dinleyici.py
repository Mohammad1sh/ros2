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
    """kamera kutusu — GERCEK servo + SIM kapagi (7. eksen) birlikte hareket eder"""
    s = 35 if acik else 160
    pub_servo.publish(String(data=json.dumps({'s1': s, 's2': s, 'sander': 222})))
    state['kutu_sim'] = 0.025 if acik else 0.0

def gercek_zimpara(acik):
    pub_sander.publish(String(data=json.dumps({'sander': 111 if acik else 222})))

FORCE_N      = 25.0          # zimpara baslama sarti (gercek load cell, Newton)
FORCE_WAIT_S = 120.0         # 25N bekleme ust siniri (sim-sn); dolarsa uyariyla devam
DISK_R       = 0.05          # disk yaricapi (kumeleme icin)

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
        if e and (state.get('gorev') or (onceki is not None and not onceki)):
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
    # 7. eksen = kamera kutusu KAPAGI (prismatik 0-0.025m): sim'de de acilip kapanir
    m = Float64MultiArray(); m.data = [float(v) for v in q] + [state.get('kutu_sim', 0.0)]
    pub_j.publish(m)
    z = Float64MultiArray(); z.data = [float(spin)]
    pub_z.publish(z)

def gecikme():
    """son komut ile GERCEK eklem arasindaki en buyuk fark (rad)"""
    q = state.get('son_komut')
    if not q or not all(k in jstate for k in JNAMES): return 0.0
    return max(abs(jstate[k] - q[i]) for i, k in enumerate(JNAMES))

def move_seg(q0, q1, dur, spin=0.0, zorla=False):
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
            log_gui(f'  yol basina hizalaniyor (fark {e:.2f} rad)')
            move_seg([jstate[k] for k in JNAMES], list(q0), max(1.2, e / 0.8), zorla=True)
            t0 = time.time()
            while time.time() - t0 < max(sure, e * 3):
                send(list(q0)); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
                if all(k in jstate for k in JNAMES):
                    h = max(abs(jstate[k] - q0[i]) for i, k in enumerate(JNAMES))
                    if h < 0.10: break
            log_gui(f'  hizalandi (hata {h:.3f} rad, {time.time()-t0:.1f} sn)')
    return True

def play_path(path, per_seg=None, spin=0.0, etiket='yol'):
    """Yolu oynat — ILERLEME LOGLU ve ZAMAN BUTCELI (asla takilip kalmaz)."""
    yol_basina_git(path[0])
    dur = per_seg or TRAVERSE_S
    butce = len(path) * dur * 3.0 + 20.0      # gecikme kapisi paylı ust sinir
    t0 = time.time(); son_log = 0.0
    for k in range(1, len(path)):
        if time.time() - t0 > butce:
            log_gui(f'  {etiket}: zaman butcesi doldu ({k}/{len(path)}) — dogrudan hedefe')
            move_seg([jstate[j] for j in JNAMES] if all(j in jstate for j in JNAMES) else path[k-1],
                     path[-1], 2.5, zorla=True)
            return True
        if not move_seg(path[k-1], path[k], dur, spin):
            return False
        if time.time() - son_log > 2.5:
            son_log = time.time()
            log_gui(f'  {etiket}: {k}/{len(path)} ({time.time()-t0:.0f} sn)')
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
n.create_subscription(JointState, '/gz/joint_states',
                      lambda m: jstate.update(zip(m.name, m.position)), 10)
JNAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

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
        _park_git(from_y, hiz, segs, segp)
    finally:
        state['park_modu'] = False
        state['emergency'] = False; state['stop'] = False   # yankilari yut

def _park_git(from_y, hiz, segs, segp):
    # 1) kol ASAGIDAYSA once dikey kalk (yatay hareket sasiyi sıyırmasin)
    if state.get('lvl') == 'LOW' and state.get('y_son') is not None:
        ys = state['y_son']
        log_gui('  once DIKEY kalkis')
        move_seg([jstate[k] for k in JNAMES] if all(k in jstate for k in JNAMES)
                 else karisim(ys, 1.0), table_at(HIGH, ys), 1.0, zorla=True)
        state['lvl'] = 'HIGH'
    # 2) koridora EN YAKIN adimdan geri oyna
    i, d = _en_yakin_koridor()
    if i is not None and d < 0.6:
        log_gui(f'  koridora giris: adim {i}/{len(T["park_to_scan"])} (fark {d:.2f} rad)')
        path = list(reversed(T['park_to_scan'][:i+1]))
    else:
        # 3) koridordan uzak: once HIGH hatti + tarama pozu
        if from_y is not None:
            glide(HIGH, from_y, SCAN_Y, hiz)
        move_seg([jstate[k] for k in JNAMES] if all(k in jstate for k in JNAMES)
                 else table_at(HIGH, SCAN_Y), T['scan'], segs, zorla=True)
        path = list(reversed(T['park_to_scan']))
    if not play_path(path, per_seg=segp):
        # koridor sirasinda YENI acil durum: oldugun yerde SABITLEN (yanki yok)
        if all(k in jstate for k in JNAMES):
            dur = [jstate[k] for k in JNAMES]
            t0 = time.time()
            while time.time() - t0 < 3.0:
                send(dur); rclpy.spin_once(n, timeout_sec=0.02); time.sleep(0.03)
        log_gui('Park donusu kesildi — kol guvenli sekilde sabitlendi')
        return
    # parki eklem geri beslemesiyle kilitle
    t0 = time.time(); son_log = 0.0
    while time.time() - t0 < 25:       # kisa kilit: dinleyici hemen bosalir,
        send(T['park'])                # ikinci START gecikmeden islenir
        rclpy.spin_once(n, timeout_sec=0.01); time.sleep(0.03)
        if all(k in jstate for k in JNAMES):
            err = max(abs(jstate[k] - T['park'][i]) for i, k in enumerate(JNAMES))
            if err < 0.06:
                log_gui(f'PARK ✓ ({time.time()-t0:.1f} sn, hata {err:.3f} rad)')
                return
            if time.time() - son_log > 4.0:
                son_log = time.time()
                log_gui(f'  parka yaklasiyor... hata {err:.2f} rad')
    log_gui('Park zaman asimi — kol yaklasik park pozunda')

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

SPAN = 0.155   # HIGH(z=0.300) ile LOW(z=0.145) arasi dusey mesafe (m)

def bekle_varis(q, tol=0.05, sure=6.0, etiket=''):
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
    Donus: 'TEMAS' | 'BOS' | 'IPTAL'"""
    a_ust = 1 - 0.05 / SPAN            # LOW'un 5cm ustu
    a_alt = 1 + 0.05 / SPAN            # LOW'un 5cm alti
    hedef = karisim(ya, a_ust)
    if not move_seg(table_at(HIGH, ya), hedef, 1.5):
        return ('IPTAL', a_ust)
    bekle_varis(hedef, 0.04, 4.0)          # 5cm ustunde GERCEKTEN dur
    orn = []; t0 = time.time()
    while time.time() - t0 < 0.8:      # dara: bosta okuma
        send(hedef); rclpy.spin_once(n, timeout_sec=0.05)
        orn.append(state['force'])
    orn.sort(); dara = orn[len(orn)//2] if orn else 0.0
    log_gui(f'{etiket}: 5cm ustte dara={dara:.1f} -> asamali inis (kuvvet izleniyor)')

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

def cluster(dets):
    """Kare-tespitleri -> CAPAK NOKTALARI -> bolgeler.
    1) Her karedeki kutular metreye eslenir; harita DISINA dusenler COPE atilir
       (eskiden uclara yapistiriliyordu -> bolgeler boydan boya sisiyordu).
    2) 3cm toleransla 1D gruplama + KALICILIK filtresi (yeterince karede
       gorulmeyen hayaletler elenir) -> her capagin MEDYAN konumu = 1 nokta.
    3) Noktalar arasi bosluk <= GAP_M (disk capi) ise ayni iniste taranir;
       buyukse KALK-GEC (in-zimpara-kalk). Disk MERKEZI ilk noktaya iner,
       son noktaya kadar gider.
    Donus: (bolgeler[(y0,y1)], noktalar[(y, px)])"""
    hepsi = []                               # (y, px)
    for frame in dets:
        for b in frame:
            dy = (b['x'] - FRAME_W / 2) * PX2M * AXIS_SIGN
            y = SCAN_Y + dy
            if Y_MIN - 0.02 <= y <= Y_MAX + 0.02:
                hepsi.append((min(max(y, Y_MIN), Y_MAX), float(b['x'])))
    if not hepsi:
        return [], []
    hepsi.sort()
    # -- 1D gruplama (3cm) --
    gruplar = [[hepsi[0]]]
    for y, px in hepsi[1:]:
        if y - gruplar[-1][-1][0] <= 0.03:
            gruplar[-1].append((y, px))
        else:
            gruplar.append([(y, px)])
    # -- kalicilik: en az 3 gozlem (veya kare sayisinin 1/5'i) --
    esik = max(3, len(dets) // 5)
    noktalar = []
    for g in gruplar:
        if len(g) >= esik:
            g_y  = sorted(v[0] for v in g)
            g_px = sorted(v[1] for v in g)
            noktalar.append((g_y[len(g_y)//2], g_px[len(g_px)//2]))
    if not noktalar:
        return [], []
    # -- noktalardan bolgeler: bosluk > GAP_M ise ayri inis --
    bolgeler = [[noktalar[0][0], noktalar[0][0]]]
    for y, _ in noktalar[1:]:
        if y - bolgeler[-1][1] <= GAP_M:
            bolgeler[-1][1] = y
        else:
            bolgeler.append([y, y])
    return [(a, b) for a, b in bolgeler], noktalar

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
    state['lvl'] = 'HIGH'; state['y_son'] = None
    gercek_zimpara(False)        # onceki kosudan takili kalmis roleyi KAPAT
    log_gui('START alindi -> kol tarama pozuna gidiyor (esik +25cm)...')
    if not play_path(T['park_to_scan'], per_seg=0.08):   # ~13 sn (olculen guvenli hiz)
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
    t0 = time.time()
    while time.time() - t0 < CAM_WINDOW_S:
        if iptal(): break
        send(T['scan'])
        rclpy.spin_once(n, timeout_sec=0.05)

    log_gui('Kamera kutusu kapaniyor...')
    kamera_kutusu(False)
    if iptal():
        state['emergency'] = False; state['stop'] = False; state['gorev'] = False
        to_park(acele=True); log_gui('Iptal edildi -> parkta.'); continue

    bolgeler, noktalar = cluster(state['dets'])
    log_gui(f'{sum(len(f) for f in state["dets"])} kare-tespit -> '
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
        y_end = max(yb, ya + 0.01)   # min 1cm ilerleme (tek capak = 5sn)
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
        sebep = 'EMERGENCY' if state['emergency'] else 'STOP'
        log_gui(f'{sebep}! role kapatildi — HIZLI kalkis + parka donus')
        kamera_kutusu(False)
        ys = state['y_son'] if state['y_son'] is not None else SCAN_Y
        if state['lvl'] == 'LOW':
            move_seg(table_at(LOW, ys), table_at(HIGH, ys), 0.8, zorla=True)
            state['lvl'] = 'HIGH'
        state['emergency'] = False; state['stop'] = False
        state['gorev'] = False   # park donusu sirasinda bayrak yankilanmasin
        to_park(from_y=ys, acele=True)
        log_gui('Iptal tamamlandi — kol parkta. Tekrar START bekleniyor')
    else:
        state['gorev'] = False
        log_gui('Tum bolgeler bitti -> PARK\'a donuluyor')
        to_park(from_y=cur_y)
        log_gui('GOREV TAMAM ✓ — tekrar START bekleniyor')
    state['gorev'] = False
    state['start'] = False       # gorev sirasinda birikmis basislar sayilmasin
