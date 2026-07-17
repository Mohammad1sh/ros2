#!/usr/bin/env python3
"""SIM PROVA — mini PC OLMADAN tam gorev provasi.
Sahte START + tarama penceresinde sahte capak tespitleri + dinleyicinin
'SIMDI BASTIR' mesajina REAKTIF sahte 25N basisi. Gazebo'da tum akisi izle:
park -> tarama -> inis -> temas -> 1cm/5sn zimpara -> kalkis -> 2. bolge -> park.
Kullanim: gazebo_baslat.sh calisirken calistir. Mini PC'ye hicbir etkisi yok."""
import rclpy, time, json, threading
from std_msgs.msg import Bool, String

rclpy.init()
n  = rclpy.create_node('sim_prova')
ps = n.create_publisher(Bool,   '/end_effector/mission_start',  10)
pd = n.create_publisher(String, '/end_effector/detections',     10)
pm = n.create_publisher(String, '/end_effector/mission_status', 10)

TABAN = 412.0            # dara alinmamis gercekci bos okuma
durum = {'bas_bitis': 0.0, 'bitti': False}

def on_log(m):
    s = m.data
    if 'SIMDI BASTIR' in s:
        durum['bas_bitis'] = time.time() + 9.0   # 1 sn sonra 8 sn bas
        print('>> prova: BASTIR istendi -> +32N uygulanacak', flush=True)
    if 'GOREV TAMAM' in s or 'Iptal tamamlandi' in s:
        durum['bitti'] = True
n.create_subscription(String, '/end_effector/log', on_log, 10)

def yayinci():
    t0 = time.time()
    while not durum['bitti'] and time.time() - t0 < 420:
        f = TABAN + (32.0 if time.time() < durum['bas_bitis'] else 0.0)
        pm.publish(String(data=json.dumps({'contact_force': f, 'emergency': False})))
        # tarama penceresine dusen araliklarda tespit yayinla (genis tut)
        el = time.time() - t0
        if 8.0 < el < 45.0 and int(el * 2) % 3 == 0:
            pd.publish(String(data=json.dumps({'burrs': [
                {'x': 500, 'y': 360, 'conf': 0.9, 'dist': 0},
                {'x': 560, 'y': 360, 'conf': 0.9, 'dist': 0},   # yakin -> ayni bolge
                {'x': 980, 'y': 360, 'conf': 0.9, 'dist': 0},   # uzak  -> ayri bolge
            ], 'count': 3})))
        rclpy.spin_once(n, timeout_sec=0.05)
        time.sleep(0.1)

th = threading.Thread(target=yayinci, daemon=True)
th.start()
time.sleep(1.0)
b = Bool(); b.data = True
ps.publish(b)
print('>> prova: START verildi — Gazebo penceresini izle', flush=True)
th.join()
print('>> prova BITTI' if durum['bitti'] else '>> prova zaman asimi', flush=True)
