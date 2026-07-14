#!/usr/bin/env python3
"""
gazebo_bridge.py - Gazebo Köprüsü v3.1
========================================
Değişiklikler v3.1:
  - Zımpara velocity controller entegre edildi
  - Zımpara açıldığında disk otomatik döner
  - 7 joint gönderimi: joint_1-6 + servo_joint
  - IK entegre
"""

import rclpy
from rclpy.node import Node
import json
import math
import time
import random
import threading
import numpy as np

from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


# ── Sabitler ──────────────────────────────────────────────────────────────────
JOINT_LIMITS = [
    (-math.pi * 2, math.pi * 2),
    (-math.pi * 2, math.pi * 2),
    (-math.pi * 165/180, math.pi * 165/180),   # J3 ±165° (H2515 katalog)
    (-math.pi * 2, math.pi * 2),
    (-math.pi * 2, math.pi * 2),
    (-math.pi * 2, math.pi * 2),
]
# Dik takım kısıtıyla erişim zarfı (H2515, maks yarıçap 1700mm katalog):
REACH_MAX_M = 1.35   # yatay maksimum (dik takımla)
REACH_MIN_M = 0.45   # yatay minimum (tabana çok yakın = tekil bölge)
Z_MAX_M     = 1.15
Z_MIN_M     = 0.05   # eşik yüzeyi ~0.12m — alçak temaslara izin ver
HOME_JOINTS_RAD  = [0.0, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
# PARK pozisyonu: kol araçtan uzağa (-Y) dönük, kompakt katlanmış.
# Görev bitince / acil durumda / açılışta bu poza gidilir.
PARK_JOINTS_RAD  = [-math.pi/2, 0.0, math.pi/2, 0.0, math.pi/2, 0.0]
# Arabaya (ön kapı eşiği ~(-0.56, +0.22)) doğru IK başlangıç açıları
IK_INIT_RAD      = [2.76, 0.3, 1.2, 0.0, 0.8, 0.0]
SERVO_CENTER_DEG = 160
SANDER_ON        = 111
SANDER_OFF       = 222
ZIMPARA_CTRL     = 'zimpara_velocity_controller'
ZIMPARA_SPEED    = 50.0   # rad/s

# ── IK parametreleri ──────────────────────────────────────────────────────────
JOINTS_DATA = [
    ([0.0,    0.0,     0.3443], [0.0,        0.0,        0.0      ]),
    ([0.0,    0.0099,  0.0   ], [0.0,       -math.pi/2, -math.pi/2]),
    ([0.7595, 0.0,     0.0   ], [0.0,        0.0,        math.pi/2]),
    ([0.0,   -0.6195,  0.0   ], [math.pi/2,  0.0,        0.0      ]),
    ([0.0,    0.0,     0.0   ], [-math.pi/2, 0.0,        0.0      ]),
    ([0.0,   -0.121,   0.0   ], [math.pi/2,  0.0,        0.0      ]),
]
# Zımpara diski temas yüzü (link_6 çerçevesinde): montaj CAD'ine göre
# disk ekseni (0.003, 0.011), temas yüzü z≈0.225 (bkz. robot_with_sander.urdf.xacro)
# NOT: takla YOK — montaj link_6'nın +Z'sinde uzanır; flanş aşağı bakınca disk aşağı bakar.
TOOL_XYZ = [0.003, 0.011, 0.225]
TOOL_RPY = [0.0, 0.0, 0.0]

def _rot_x(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]],dtype=float)
def _rot_y(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]],dtype=float)
def _rot_z(a):
    c,s=math.cos(a),math.sin(a)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
def _trans(x,y,z):
    T=np.eye(4); T[0,3]=x; T[1,3]=y; T[2,3]=z; return T
def _jt(xyz,rpy,theta):
    # URDF rpy konvansiyonu: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)  (sabit eksen XYZ)
    # (Önceki Rx@Ry@Rz sırası YANLIŞTI — Gazebo'daki robotla 2.4m'ye varan sapma yaratıyordu)
    x,y,z=xyz; r,p,yw=rpy
    return _trans(x,y,z)@_rot_z(yw)@_rot_y(p)@_rot_x(r)@_rot_z(theta)

def _quat_to_rot(qx, qy, qz, qw):
    """Normalize quaternion → 3×3 rotation matrix."""
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-9: return np.eye(3)
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)  ],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)  ],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)],
    ])

def _rot_err_vec(R_cur, R_des):
    """SO(3) oryantasyon hatası — 3D eksen-açı vektörü olarak döndürür."""
    R_e = R_des @ R_cur.T
    return 0.5 * np.array([R_e[2,1]-R_e[1,2], R_e[0,2]-R_e[2,0], R_e[1,0]-R_e[0,1]])

_TOOL_T = _jt(TOOL_XYZ, TOOL_RPY, 0.0)  # sabit, bir kez hesapla

def forward_kinematics(joints):
    T=np.eye(4)
    for i,(xyz,rpy) in enumerate(JOINTS_DATA):
        T=T@_jt(xyz,rpy,joints[i])
    return T @ _TOOL_T  # zımpara ucu dahil

def inverse_kinematics(target, q_init=None, target_rot=None,
                       max_iter=2000, alpha=0.3, w_orient=0.3, lam=0.01):
    """
    3DOF pozisyon IK (target_rot=None) veya 6DOF IK (target_rot=3×3 ndarray).

    target:     [x, y, z] metre — tool tip hedefi
    target_rot: 3×3 ndarray — hedef oryantasyon (None → sadece pozisyon)
    w_orient:   oryantasyon hata ağırlığı (pozisyon=1.0 sabit, ori=w_orient)
    lam:        damped least squares sönümleme katsayısı (singülerlik güvencesi)
    Döndürür:   (joint_list_rad, pozisyon_hata_metre)
    """
    if q_init is None: q_init = HOME_JOINTS_RAD.copy()
    q = np.array(q_init, dtype=float)
    use_6dof = target_rot is not None
    n = 6 if use_6dof else 3
    lam2 = lam * lam

    for _ in range(max_iter):
        T = forward_kinematics(q)
        pos_cur = T[:3, 3]
        e_pos = np.array(target) - pos_cur

        if use_6dof:
            e_ori = _rot_err_vec(T[:3, :3], target_rot) * w_orient
            err = np.concatenate([e_pos, e_ori])
        else:
            err = e_pos

        # Erken çıkış: pozisyon + oryantasyon birlikte küçüldüğünde
        if np.linalg.norm(err) < 1e-4: break

        J = np.zeros((n, 6))
        delta = 1e-6
        for i in range(6):
            qd = q.copy(); qd[i] += delta
            T_d = forward_kinematics(qd)
            J[:3, i] = (T_d[:3, 3] - pos_cur) / delta
            if use_6dof:
                # NOT: e_ori "kalan dönüş"tür (hedef−mevcut gibi) → türevin işareti
                # pozisyon satırlarıyla tutarlı olması için NEGATİF alınmalı.
                J[3:, i] = -(_rot_err_vec(T_d[:3, :3], target_rot) - e_ori/w_orient) / delta * w_orient

        # Damped least squares: (JᵀJ + λ²I)⁻¹ Jᵀ — singülaritelerde kararlı
        JtJ = J.T @ J
        dq = np.linalg.solve(JtJ + lam2 * np.eye(6), J.T @ err)
        q += alpha * dq
        # Eklem açılarını [-π, π] aralığına normalize et — sarma hatasını önler
        q = np.array([(a + math.pi) % (2*math.pi) - math.pi for a in q])
        q = np.clip(q, [-l for _,l in JOINT_LIMITS], [l for _,l in JOINT_LIMITS])

    T = forward_kinematics(q)
    return q.tolist(), float(np.linalg.norm(np.array(target) - T[:3, 3]))

IK_DOWN_D = -math.pi      # 2-3-5 eksenleri paralel: q5 = D-(q2+q3) → FLANŞ (ve montaj) TAM aşağı
                          # (URDF-doğru zincirle kalibre edildi: sapma 0.1°)

def ik_down(target, q_init=None):
    """
    Takım DAİMA aşağı bakacak şekilde analitik-kısıtlı IK.
    q4=q6=0, q5=π/2-(q2+q3) bağı takımı dik tutar; pozisyon q1,q2,q3 ile çözülür.
    Doğrulama: 3 hedefte hata <0.1mm, takım z=(0,0,-1).
    """
    if q_init is None:
        q_init = IK_INIT_RAD
    q1, q2, q3 = q_init[0], q_init[1], q_init[2]
    for _ in range(2000):
        q = [q1, q2, q3, 0.0, IK_DOWN_D - (q2 + q3), 0.0]
        T = forward_kinematics(q)
        p = T[:3, 3]
        e = np.array(target) - p
        if np.linalg.norm(e) < 1e-4:
            break
        J = np.zeros((3, 3)); d = 1e-6
        for i in range(3):
            qq = [q1, q2, q3]; qq[i] += d
            q_ = [qq[0], qq[1], qq[2], 0.0, IK_DOWN_D - (qq[1] + qq[2]), 0.0]
            J[:, i] = (forward_kinematics(q_)[:3, 3] - p) / d
        dq = np.linalg.solve(J.T @ J + 1e-4 * np.eye(3), J.T @ e)
        q1 += 0.4 * dq[0]; q2 += 0.4 * dq[1]; q3 += 0.4 * dq[2]
    q = [q1, q2, q3, 0.0, IK_DOWN_D - (q2 + q3), 0.0]
    q = [(a + math.pi) % (2 * math.pi) - math.pi for a in q]
    T = forward_kinematics(q)
    return q, float(np.linalg.norm(np.array(target) - T[:3, 3]))

def _R_down_yaw(yaw_rad):
    """Takım aşağı bakar + kendi ekseni etrafında yaw kadar dönük yönelim matrisi."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])

def _rot_angle(R_cur, R_des):
    """İki yönelim arasındaki açı (rad)."""
    c = (np.trace(R_des @ R_cur.T) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))

def ik_solve(target, target_rot, elbow='auto', current=None):
    """
    Tam 6-eksen IK: çok tohumlu (dirsek-yukarı / dirsek-aşağı / mevcut poz)
    sayısal çözüm; en iyi (pozisyon+yönelim) skoru seçilir.
    elbow: 'up' | 'down' | 'auto'
    Döner: (joints, poz_hata_m, ori_hata_rad)
    """
    q1t = math.atan2(target[1], target[0])
    seeds = []
    if elbow in ('auto', 'up'):
        s, _ = ik_down(target, [q1t, 0.3, 1.2]);  seeds.append(s)
    if elbow in ('auto', 'down'):
        s, _ = ik_down(target, [q1t, 1.5, -1.0]); seeds.append(s)
    if current is not None and len(current) >= 6:
        seeds.append(list(current[:6]))
    best = None
    for s in seeds:
        q, e = inverse_kinematics(target, list(s), target_rot=target_rot,
                                  max_iter=1500, alpha=0.3, w_orient=0.5, lam=0.02)
        # Çözücü bazen takım ekseninde 180° ters yaw'a oturur —
        # J6'yı π çevirmek pozisyonu değiştirmeden yaw'ı düzeltir; iyisini al.
        for q_c in (q, q[:5] + [((q[5] + math.pi) + math.pi) % (2*math.pi) - math.pi]):
            oe = _rot_angle(forward_kinematics(q_c)[:3, :3], target_rot)
            score = e + 0.1 * oe
            if best is None or score < best[0]:
                best = (score, q_c, e, oe)
    return best[1], best[2], best[3]

def deg_to_rad(d): return math.radians(d)
def rad_to_deg(r): return math.degrees(r)
def clamp(v,lo,hi): return max(lo,min(hi,v))
def servo_deg_to_rad(d): return math.radians(d-SERVO_CENTER_DEG)


class GazeboBridge(Node):

    def __init__(self):
        super().__init__('gazebo_bridge')
        self.declare_parameter('simulation', True)
        self.declare_parameter('publish_rate', 10.0)
        # Canlı ayarlanabilir: ros2 param set /gazebo_bridge tool_yaw_deg 90
        self.declare_parameter('tool_yaw_deg', 0.0)     # takımın kendi ekseni yönü
        self.declare_parameter('elbow_mode', 'auto')    # 'up' | 'down' | 'auto'
        self.simulation   = self.get_parameter('simulation').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self._current_joints_rad = list(HOME_JOINTS_RAD)
        self._servo_pos     = 0.0
        self._sander_active = False
        self._zimpara_ctrl_ready = False
        self._running       = True
        self._home_sent     = False
        self._lock          = threading.Lock()
        self._sim_contact_z = None   # son IK hedef Z — simüle temas için
        # simulation:=false → GERÇEK donanım modunda başla: sahte load cell /
        # can_status yayınlama (mini PC'deki gerçek can_node ile çakışmasın)
        self._mode          = 'simulation' if self.simulation else 'hardware'

        # Publisher'lar
        self.pub_gz_joints   = self.create_publisher(Float64MultiArray, '/gz/dsr_position_controller/commands', 10)
        self.pub_zimpara     = self.create_publisher(Float64MultiArray, f'/gz/{ZIMPARA_CTRL}/commands', 10)
        self.pub_robot_state = self.create_publisher(String, '/end_effector/robot_state',  10)
        self.pub_can_status  = self.create_publisher(Bool,   '/end_effector/can_status',   10)
        self.pub_drfl_status = self.create_publisher(String, '/end_effector/dsr2_status',  10)
        self.pub_load_cells  = self.create_publisher(String, '/end_effector/load_cells',   10)
        self.pub_sim_info    = self.create_publisher(String, '/end_effector/sim_info',     10)
        self.pub_log         = self.create_publisher(String, '/end_effector/log',          10)

        # Subscriber'lar
        self.create_subscription(String,           '/end_effector/servo_command',          self._cb_servo_cmd,    10)
        self.create_subscription(Float64MultiArray,'/end_effector/joint_command',          self._cb_joint_cmd,    10)
        self.create_subscription(String,           '/end_effector/cartesian_command',      self._cb_cartesian_cmd,10)
        self.create_subscription(Bool,             '/end_effector/emergency_stop',         self._cb_emergency,    10)
        self.create_subscription(String,           '/end_effector/sander_only',            self._cb_sander_only,  10)
        self.create_subscription(JointState,       '/gz/joint_states',                     self._cb_joint_states, 10)
        self.create_subscription(PoseStamped,      '/cartesian_interface/arm/reference',   self._cb_ik_pose,      10)
        self.create_subscription(String,           '/end_effector/set_mode',               self._cb_set_mode,     10)
        self.create_subscription(Bool,             '/end_effector/shutdown',               self._cb_shutdown,     10)
        self.create_subscription(Bool,             '/end_effector/go_home',                self._cb_go_home,      10)

        # Timer'lar
        self.create_timer(1.0/self.publish_rate, self._publish_state)
        self.create_timer(0.1, self._publish_sim_load_cells)
        self.create_timer(2.0, self._publish_connection_status)
        self.create_timer(6.0, self._go_home_once)   # 6s: controller hazır olsun

        # zimpara_velocity_controller gazebo.launch.py'deki zimpara_spawner tarafından yükleniyor
        self._zimpara_ctrl_ready = True
        self.get_logger().info('Zımpara controller gazebo.launch.py tarafından yönetiliyor')

        T = forward_kinematics(HOME_JOINTS_RAD)
        self.get_logger().info(
            f'GazeboBridge v3.2 başlatıldı (cartesian_to_joint birleştirildi)\n'
            f'  7 joint: joint_1-6 + servo_joint\n'
            f'  IK: tool tip offset dahil ({TOOL_XYZ})\n'
            f'  Zımpara controller: {ZIMPARA_CTRL}\n'
            f'  Home tool-tip FK: x={T[0,3]:.3f} y={T[1,3]:.3f} z={T[2,3]:.3f}'
        )

    # ── Zımpara Disk Kontrolü ─────────────────────────────────────────────────
    def _set_zimpara_speed(self, speed: float):
        """Zımpara disk hızını ayarla (rad/s)."""
        msg = Float64MultiArray()
        msg.data = [speed]
        self.pub_zimpara.publish(msg)

    # ── Home ──────────────────────────────────────────────────────────────────
    def _go_home_once(self):
        if self._home_sent: return
        self._home_sent = True
        self._send_to_gazebo(PARK_JOINTS_RAD)
        self.get_logger().info('PARK pozisyonu gönderildi (açılış)')

    # ── Gazebo'ya Gönder — 7 joint ────────────────────────────────────────────
    def _send_to_gazebo(self, joints_rad: list):
        clamped = []
        for i,(val,(lo,hi)) in enumerate(zip(joints_rad, JOINT_LIMITS)):
            c = clamp(val,lo,hi)
            if abs(c-val) > 0.01:
                self.get_logger().warn(f'J{i+1} limit: {rad_to_deg(val):.1f}° → {rad_to_deg(c):.1f}°')
            clamped.append(c)
        with self._lock:
            self._current_joints_rad = list(clamped)
        msg = Float64MultiArray()
        msg.data = clamped + [self._servo_pos]
        self.pub_gz_joints.publish(msg)

    # ── IK Callback ───────────────────────────────────────────────────────────
    def _cb_ik_pose(self, msg: PoseStamped):
        target = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

        # ── Erişim zarfı kırpması: bölge dışı hedef en yakın erişilebilir noktaya ──
        r = math.hypot(target[0], target[1])
        if r > 1e-6 and (r > REACH_MAX_M or r < REACH_MIN_M):
            r_c = clamp(r, REACH_MIN_M, REACH_MAX_M)
            s = r_c / r
            self.get_logger().warn(
                f'Hedef erişim dışı (yatay {r:.2f}m) → {r_c:.2f}m sınırına kırpıldı')
            self.pub_log.publish(String(
                data=f'[WARN] Hedef erişim dışı ({r:.2f}m) → {r_c:.2f}m'))
            target[0] *= s
            target[1] *= s
        if not (Z_MIN_M <= target[2] <= Z_MAX_M):
            z_c = clamp(target[2], Z_MIN_M, Z_MAX_M)
            self.get_logger().warn(f'Z erişim dışı ({target[2]:.2f}m) → {z_c:.2f}m')
            target[2] = z_c

        self._sim_contact_z = target[2]
        self.get_logger().info(f'IK hedef: x={target[0]:.3f} y={target[1]:.3f} z={target[2]:.3f}')

        # Yönelim: quaternion verilmişse aynen; verilmemişse "aşağı + tool_yaw_deg"
        qx = msg.pose.orientation.x; qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z; qw = msg.pose.orientation.w
        norm_sq = qx*qx + qy*qy + qz*qz + qw*qw
        try:
            yaw_deg = float(self.get_parameter('tool_yaw_deg').value)
            elbow   = str(self.get_parameter('elbow_mode').value)
        except Exception:
            yaw_deg, elbow = 0.0, 'auto'
        if norm_sq > 0.1:
            target_rot = _quat_to_rot(qx, qy, qz, qw)   # tam 6-eksen serbest yönelim
        else:
            target_rot = _R_down_yaw(math.radians(yaw_deg))

        with self._lock:
            cur = list(self._current_joints_rad)
        joints, err, ori_err = ik_solve(target, target_rot, elbow=elbow, current=cur)
        self.get_logger().info(
            f'IK: poz={err*1000:.2f}mm ori={math.degrees(ori_err):.1f}° '
            f'(elbow={elbow}, yaw={yaw_deg:.0f}°)')
        if err > 0.002 or ori_err > 0.09:
            self.get_logger().warn(
                f'6DOF tam oturmadı — dik kısıtlı çözüme dönülüyor')
            joints, err = ik_down(target, [math.atan2(target[1], target[0]), 0.3, 1.2])

        if err > 0.001:
            # İPTAL ETME — en yakın çözümü uygula (kol en azından doğru bölgeye gitsin)
            self.get_logger().warn(
                f'IK tam yakınsamadı ({err*1000:.1f}mm) — en yakın çözüm uygulanıyor')
            self.pub_log.publish(String(data=f'[WARN] IK hata={err*1000:.1f}mm (uygulandı)'))
        elif err > 0.0005:
            self.get_logger().warn(f'IK hata={err*1000:.2f}mm')
        else:
            self.get_logger().info(
                f'IK OK hata={err*1000:.3f}mm → '
                f'[{", ".join(f"{math.degrees(j):.1f}°" for j in joints)}]'
            )
        self._send_to_gazebo(joints)

    # ── Diğer Callback'ler ────────────────────────────────────────────────────
    def _cb_servo_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)

            # Sander alanı varsa güncelle — yoksa mevcut durumu koru
            if 'sander' in data:
                self._sander_active = (int(data['sander']) == SANDER_ON)
                self._set_zimpara_speed(ZIMPARA_SPEED if self._sander_active else 0.0)

            # 'camera' alanı: doğrudan metre cinsinden (0=kapalı, 0.025=açık)
            if 'camera' in data:
                self._servo_pos = float(clamp(data['camera'], 0.0, 0.025))
                state = 'AÇIK' if self._servo_pos > 0.01 else 'KAPALI'
                self.get_logger().info(f'Kamera kutusu: {state} ({self._servo_pos:.3f}m)')
            elif 's1' in data:
                # 0-180° tam aralık → 0-0.025m
                self._servo_pos = clamp(int(data['s1']) / 180.0, 0.0, 1.0) * 0.025
                self.get_logger().info(f'Kamera kutusu s1: {self._servo_pos:.3f}m')

            with self._lock:
                joints = list(self._current_joints_rad)
            self._send_to_gazebo(joints)
        except Exception as e:
            self.get_logger().error(f'servo_cmd: {e}')

    def _cb_joint_cmd(self, msg: Float64MultiArray):
        if len(msg.data) < 6: return
        self._send_to_gazebo([deg_to_rad(d) for d in msg.data[:6]])

    def _cb_cartesian_cmd(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('cmd') != 'move_cartesian': return
            scale = math.pi / 1500.0
            with self._lock:
                joints = list(self._current_joints_rad)
            joints[0] = clamp(float(data.get('x',0))*scale, *JOINT_LIMITS[0])
            joints[1] = clamp(float(data.get('y',0))*scale, *JOINT_LIMITS[1])
            joints[2] = clamp(float(data.get('z',0))*scale, *JOINT_LIMITS[2])
            self._send_to_gazebo(joints)
        except Exception as e:
            self.get_logger().error(f'cartesian_cmd: {e}')

    def _cb_sander_only(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._sander_active = (int(data.get('sander', SANDER_OFF)) == SANDER_ON)
            # Zımpara disk hızı
            speed = ZIMPARA_SPEED if self._sander_active else 0.0
            self._set_zimpara_speed(speed)
            state = 'AÇIK' if self._sander_active else 'KAPALI'
            self.get_logger().info(f'Zımpara: {state} ({speed} rad/s)')
        except Exception as e:
            self.get_logger().error(f'sander_only: {e}')

    def _cb_set_mode(self, msg: String):
        self._mode = msg.data  # 'simulation' veya 'hardware'
        if self._mode == 'hardware':
            self.get_logger().info('[MOD] Gerçek Donanım — simüle yayınlar durduruldu')
        else:
            self.get_logger().info('[MOD] Simülasyon — sahte sensör aktif')

    def _cb_emergency(self, msg: Bool):
        if msg.data:
            self.get_logger().error('!!! ACİL DURDURMA — HOME pozisyonuna dönülüyor !!!')
            self._sander_active = False
            self._set_zimpara_speed(0.0)
            self._servo_pos = 0.0          # kamera kapat
            self._sim_contact_z = None     # simüle temas sıfırla
            # Mevcut pozisyonda dondurmak yerine PARK'a git
            self._send_to_gazebo(list(PARK_JOINTS_RAD))
            self.pub_log.publish(String(data='[EMERGENCY] Sander off, robot PARK pozisyonuna döndü'))

    def _cb_joint_states(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))
        self._joints = [
            joint_map.get('joint_1', 0.0),
            joint_map.get('joint_2', 0.0),
            joint_map.get('joint_3', 0.0),
            joint_map.get('joint_4', 0.0),
            joint_map.get('joint_5', 0.0),
            joint_map.get('joint_6', 0.0),
        ]
        with self._lock:
            self._current_joints_rad = list(self._joints)

    # ── Durum Yayınları ───────────────────────────────────────────────────────
    def _publish_state(self):
        with self._lock:
            joints_rad = list(self._current_joints_rad)
        self.pub_robot_state.publish(String(data=json.dumps({
            'joints':     [round(rad_to_deg(j),2) for j in joints_rad],
            'joints_rad': [round(j,4) for j in joints_rad],
            'sander_active': self._sander_active,
            'mode': 'gazebo_simulation',
        })))

    def _publish_sim_load_cells(self):
        if self._mode == 'hardware':
            return  # Gerçek donanım modunda sahte load cell yayınlama
        t = time.time()
        # Simüle temas: Z hedefi eşik yüzeyine (~0.12m + pay) inince kuvvet artar
        sim_contact = (self._sim_contact_z is not None
                       and self._sim_contact_z <= 0.15)
        if self._sander_active or sim_contact:
            base, noise = 8.0 + 4.0*math.sin(t*170.0), 2.0  # ~8-12N/kanal → toplam ~32-48N
        else:
            base, noise = 2.0 + 0.5*math.sin(t*0.3), 0.2    # ~2N/kanal → toplam ~8N
        values = [round(base+random.gauss(0,noise),2) for _ in range(4)]
        self.pub_load_cells.publish(String(data=json.dumps({'values': values})))

    def _publish_connection_status(self):
        if self._mode == 'hardware':
            return  # Gerçek donanım modunda sahte bağlantı durumu yayınlama
        self.pub_can_status.publish(Bool(data=True))
        self.pub_drfl_status.publish(String(data=json.dumps({
            'connected': True, 'mode': 'virtual (Gazebo)', 'model': 'h2515',
        })))
        with self._lock:
            joints_rad = list(self._current_joints_rad)
        self.pub_sim_info.publish(String(data=json.dumps({
            'topic': '/gz/dsr_position_controller/commands',
            'joints_deg': [round(rad_to_deg(j),1) for j in joints_rad],
            'sander': 'ON' if self._sander_active else 'OFF',
            'zimpara_ctrl_ready': self._zimpara_ctrl_ready,
        })))

    def _cb_go_home(self, msg: Bool):
        if msg.data:
            self.get_logger().info('PARK pozisyonuna dönülüyor')
            self._sander_active = False
            self._set_zimpara_speed(0.0)
            self._servo_pos = 0.0
            self._sim_contact_z = None
            self._send_to_gazebo(list(PARK_JOINTS_RAD))
            self.pub_log.publish(String(data='[INFO] Robot PARK pozisyonuna döndü'))

    def _cb_shutdown(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown sinyali alındı — kapatılıyor')
            import os, signal
            os.kill(os.getpid(), signal.SIGINT)

    def destroy_node(self):
        self._running = False
        self._set_zimpara_speed(0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()