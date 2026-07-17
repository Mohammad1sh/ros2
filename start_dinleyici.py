#!/usr/bin/env python3
"""START dinleyici — mini PC arayuzundeki START AUTONOMOUS dugmesine
basilinca (mission_start, zenoh uzerinden gelir) koreografiyi oynatir.
EMERGENCY STOP basilirsa oynatmayi durdurur.
Kullanim: demo_baslat.sh icinden calisir."""
import subprocess, sys, time, os, signal
import rclpy
from std_msgs.msg import Bool

WS = os.path.expanduser('~/ros2-end-effector')
PLAYER = [sys.executable, '-u', os.path.join(WS, 'koreografi_oynat.py')]

rclpy.init()
n = rclpy.create_node('start_dinleyici')
proc = {'p': None}

def playing():
    return proc['p'] is not None and proc['p'].poll() is None

def on_start(m):
    if not m.data:
        return
    if playing():
        n.get_logger().info('Zaten oynuyor — yoksayildi.')
        return
    n.get_logger().info('START ALINDI -> koreografi basliyor!')
    proc['p'] = subprocess.Popen(PLAYER, preexec_fn=os.setsid)

def on_stop(m):
    if m.data and playing():
        n.get_logger().warn('EMERGENCY -> oynatma durduruldu!')
        try:
            os.killpg(os.getpgid(proc['p'].pid), signal.SIGTERM)
        except Exception:
            pass

n.create_subscription(Bool, '/end_effector/mission_start', on_start, 10)
n.create_subscription(Bool, '/end_effector/emergency_stop', on_stop, 10)

print('╔══════════════════════════════════════════════╗')
print('║  HAZIR — mini PC arayuzunde START\'a bas!      ║')
print('║  (veya bu terminalde Ctrl+C ile cik)          ║')
print('╚══════════════════════════════════════════════╝', flush=True)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    if playing():
        os.killpg(os.getpgid(proc['p'].pid), signal.SIGTERM)
    n.destroy_node()
