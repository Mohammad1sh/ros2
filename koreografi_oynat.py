#!/usr/bin/env python3
"""koreografi.json'u Gazebo'da SIM-SAATINE senkron oynatir.
RTF dusuk olsa bile kol her noktayi isler. Kullanim:
  python3 koreografi_oynat.py [dosya]"""
import sys, json, time
import rclpy
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

path = sys.argv[1] if len(sys.argv) > 1 else '/home/sheik/ros2-end-effector/koreografi.json'
data = json.load(open(path))
rate, pts = data['rate'], data['points']
dt = 1.0 / rate

rclpy.init()
n = rclpy.create_node('koreografi_oynatici')
pub_j = n.create_publisher(Float64MultiArray, '/gz/dsr_position_controller/commands', 10)
pub_z = n.create_publisher(Float64MultiArray, '/gz/zimpara_velocity_controller/commands', 10)

sim = {'t': None}
def clk(m):
    sim['t'] = m.clock.sec + m.clock.nanosec * 1e-9
n.create_subscription(Clock, '/clock', clk, 10)

# clock bekle (2 sn) — yoksa duvar saatine dus
t0w = time.time()
while sim['t'] is None and time.time() - t0w < 3.0:
    rclpy.spin_once(n, timeout_sec=0.05)
use_sim = sim['t'] is not None
print(f'zaman kaynagi: {"SIM /clock" if use_sim else "duvar saati (clock yok!)"}', flush=True)

def now():
    if use_sim:
        rclpy.spin_once(n, timeout_sec=0.0)
        return sim['t']
    return time.time()

print(f'{len(pts)} nokta, {len(pts)*dt:.1f} sim-sn — basliyor...', flush=True)
t0 = now()
last_report = -1
for i, p in enumerate(pts):
    target_t = t0 + i * dt
    while now() < target_t:
        rclpy.spin_once(n, timeout_sec=0.01)
        time.sleep(0.002)
    m = Float64MultiArray(); m.data = [float(v) for v in p['j']] + [0.0]
    pub_j.publish(m)
    z = Float64MultiArray(); z.data = [float(p.get('spin', 0.0))]
    pub_z.publish(z)
    sec = int(i * dt)
    if sec != last_report:
        last_report = sec
        print(f'  sim {sec:3d}/{int(len(pts)*dt)} sn', flush=True)

# SON POZA GERI BESLEMELI KILIT: eklemler gercekten varana kadar yayinla
# (bu controller yalnizca mesaj geldikce hareket uygular — yayin kesilirse donar)
from sensor_msgs.msg import JointState
jstate = {}
n.create_subscription(JointState, '/gz/joint_states',
                      lambda m: jstate.update(zip(m.name, m.position)), 10)
last = Float64MultiArray(); last.data = [float(v) for v in pts[-1]['j']] + [0.0]
names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
t0w = time.time()
while time.time() - t0w < 90:
    pub_j.publish(last)
    z = Float64MultiArray(); z.data = [0.0]
    pub_z.publish(z)
    rclpy.spin_once(n, timeout_sec=0.01)
    time.sleep(0.04)
    if all(k in jstate for k in names):
        err = max(abs(jstate[k] - last.data[i]) for i, k in enumerate(names))
        if err < 0.02:
            print(f'park dogrulandi (maks hata {err:.3f} rad)', flush=True)
            break
print('BITTI — kol park pozunda.', flush=True)
n.destroy_node(); rclpy.shutdown()
