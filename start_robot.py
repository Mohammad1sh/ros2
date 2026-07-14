#!/usr/bin/env python3
"""
start_robot.py — Doosan H2515 End Effector Başlatıcı
3 mod:
  Simülasyon   — dsr_bringup2 (virtual) + simulation:=true
  CAN Donanım  — sadece CAN kart + kamera, DSR robot yok
  Gerçek Robot — dsr_bringup2 (real) + CAN kart

Tüm end-effector node'ları aynı Python prosesinde çalışır
(DDS inter-process iletişimine gerek kalmaz).
"""

import sys
import os
import subprocess
import signal
import time
import threading

os.environ.setdefault('DISPLAY', ':0')

from PyQt6.QtWidgets import (
    QApplication, QDialog, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QProgressDialog,
    QMessageBox, QFrame,
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

WS    = os.path.expanduser('~/ros2-end-effector')
BASH  = '/bin/bash'
SETUP = f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'

STYLE = """
QDialog  { background: #1e1e2e; }
QLabel   { color: #cdd6f4; font-size: 13px; }
QLabel#title { color: #89b4fa; font-size: 15px; font-weight: bold; }
QLabel#sub   { color: #6c7086; font-size: 11px; }
QLineEdit {
    background: #313244; color: #cdd6f4;
    border: 1px solid #585b70; border-radius: 6px;
    padding: 8px; font-size: 13px;
}
QPushButton {
    border-radius: 6px; padding: 10px 0;
    font-size: 13px; font-weight: bold;
}
QPushButton#can  { background: #a6e3a1; color: #1e1e2e; }
QPushButton#can:hover  { background: #94e2d5; }
QPushButton#real { background: #f38ba8; color: #1e1e2e; }
QPushButton#real:hover { background: #eba0ac; }
QPushButton#sim  { background: #45475a; color: #cdd6f4; }
QPushButton#sim:hover  { background: #585b70; }
"""

MODE_SIM  = 'sim'
MODE_CAN  = 'can'
MODE_REAL = 'real'


class StartupDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.mode      = MODE_CAN
        self.robot_ip  = '192.168.137.100'
        self._build()

    def _build(self):
        self.setWindowTitle('Doosan H2515 — End Effector')
        self.setFixedSize(460, 260)
        self.setStyleSheet(STYLE)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(28, 24, 28, 24)
        lay.setSpacing(10)

        title = QLabel('🤖  Doosan H2515 — B-Pillar Zımparalama')
        title.setObjectName('title')
        lay.addWidget(title)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet('color: #45475a; margin: 2px 0;')
        lay.addWidget(sep)

        lay.addWidget(QLabel('Robot IP Adresi (sadece Gerçek Robot için):'))
        self.ip_edit = QLineEdit('192.168.137.100')
        self.ip_edit.setPlaceholderText('ör. 192.168.137.100')
        lay.addWidget(self.ip_edit)

        hint = QLabel('CAN Donanım: USB seri kart + kamera — DSR robot gerekmez')
        hint.setObjectName('sub')
        lay.addWidget(hint)

        lay.addSpacing(6)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        b_sim = QPushButton('Simülasyon')
        b_sim.setObjectName('sim')
        b_sim.clicked.connect(self._launch_sim)
        btn_row.addWidget(b_sim)

        b_can = QPushButton('CAN Donanım  ▶')
        b_can.setObjectName('can')
        b_can.setDefault(True)
        b_can.clicked.connect(self._launch_can)
        btn_row.addWidget(b_can)

        b_real = QPushButton('Gerçek Robot')
        b_real.setObjectName('real')
        b_real.clicked.connect(self._launch_real)
        btn_row.addWidget(b_real)

        lay.addLayout(btn_row)

    def _launch_sim(self):
        self.mode = MODE_SIM
        self.accept()

    def _launch_can(self):
        self.mode = MODE_CAN
        self.accept()

    def _launch_real(self):
        ip = self.ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(self, 'Hata', 'IP adresi boş olamaz.')
            return
        self.robot_ip = ip
        self.mode = MODE_REAL
        self.accept()


def shell(cmd):
    return subprocess.Popen(
        f"bash -c '{SETUP} && {cmd}'",
        shell=True, executable=BASH,
        preexec_fn=os.setsid,
    )


def kill_proc(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass


def wait_for_service(app, service='/dsr01/motion/move_line', timeout=35):
    check = f"bash -c '{SETUP} && ros2 service list 2>/dev/null | grep -q \"{service}\"'"
    for _ in range(timeout):
        app.processEvents()
        r = subprocess.run(check, shell=True, executable=BASH)
        if r.returncode == 0:
            return True
        time.sleep(1)
    return False


def main():
    app = QApplication(sys.argv)

    # DAĞITIK MİNİ PC: mod seçim penceresini atla, doğrudan CAN Donanım
    # modunda aç (gui+logic+can+vision TEK proseste → DDS inter-process
    # keşfi gerekmez, load cell/kamera abonelikleri sorunsuz çalışır).
    auto = os.environ.get('MINIPC_AUTO_MODE', '').strip().lower()
    if auto == 'can':
        mode = MODE_CAN
        ip   = '192.168.137.100'
    else:
        dlg = StartupDialog()
        if dlg.exec() != QDialog.DialogCode.Accepted:
            sys.exit(0)
        mode = dlg.mode
        ip   = dlg.robot_ip

    procs = []

    def cleanup(*_):
        for p in procs:
            kill_proc(p)
        sys.exit(0)

    signal.signal(signal.SIGINT,  cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    # ── Gerçek Robot modunda DSR bringup (ayrı proses, ROS2 launch gerekli) ──
    if mode == MODE_SIM:
        dsr_cmd = (f'ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py '
                   f'model:=h2515 mode:=virtual host:={ip}')
        procs.append(shell(dsr_cmd))

        prog = QProgressDialog('DSR emülatörü başlatılıyor…', None, 0, 0)
        prog.setWindowTitle('Başlatılıyor')
        prog.setWindowModality(Qt.WindowModality.ApplicationModal)
        prog.setMinimumDuration(0); prog.setValue(0); prog.show()
        app.processEvents()
        wait_for_service(app)
        prog.close()

    elif mode == MODE_REAL:
        dsr_cmd = (f'ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py '
                   f'model:=h2515 mode:=real host:={ip}')
        procs.append(shell(dsr_cmd))

        prog = QProgressDialog(f'Robot ({ip}) bağlanıyor…', None, 0, 0)
        prog.setWindowTitle('Başlatılıyor')
        prog.setWindowModality(Qt.WindowModality.ApplicationModal)
        prog.setMinimumDuration(0); prog.setValue(0); prog.show()
        app.processEvents()

        ready = wait_for_service(app)
        prog.close()

        if not ready:
            QMessageBox.critical(None, 'Bağlantı Hatası',
                f'Robot servisleri başlatılamadı!\n\n'
                f'• Ethernet kablosunun bağlı olduğunu kontrol edin\n'
                f'• Robot IP: {ip}\n'
                f'• Robotun açık ve hazır olduğundan emin olun')
            cleanup()

    # ── Tüm end-effector node'larını AYNI proseste başlat (DDS bypass) ───────
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    is_sim     = (mode == MODE_SIM)
    use_real   = (mode == MODE_REAL)

    # Node parametrelerini rclpy.init() aracılığıyla geç
    sim_str  = 'true' if is_sim   else 'false'
    real_str = 'true' if use_real else 'false'
    rclpy.init(args=['--ros-args',
                     '-p', f'simulation:={sim_str}',
                     '-p', f'use_real_robot:={real_str}',
                     # DSR2 katmanı SADECE gerçek robot modunda — robot yokken
                     # DSR_ROBOT2 servis çağrıları sonsuza dek bloklar ve
                     # can_node'un tüm callback'lerini kilitler!
                     '-p', f'use_dsr2:={real_str}',
                     '-p', 'model_name:=latest.pt',
                     '-p', 'camera_index:=0'])

    from end_effector_ros2.logic_node   import LogicNode
    from end_effector_ros2.can_node     import CANNode
    from end_effector_ros2.vision_node  import VisionNode
    from end_effector_ros2.gui_node     import (GUINode, ROSBridge,
                                                MainWindow, Dsr2Layer)

    logic_node  = LogicNode()
    can_node_   = CANNode()
    vision_node_= VisionNode()
    gui_ros     = GUINode()

    executor = MultiThreadedExecutor()
    executor.add_node(logic_node)
    executor.add_node(can_node_)
    executor.add_node(vision_node_)
    executor.add_node(gui_ros)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ── Qt ana penceresi (aynı QApplication üzerinde) ────────────────────────
    bridge = ROSBridge(gui_ros)
    dsr2   = Dsr2Layer(node=gui_ros, sim=not use_real,
                       logger=gui_ros.get_logger())
    win = MainWindow(gui_ros, bridge, dsr2, is_sim_can=is_sim)
    win.showMaximized()

    from PyQt6.QtCore import QTimer
    # spin_once'a GEREK YOK — executor arka planda hallediyor
    # Sadece Qt event loop'u çalıştır
    try:
        ret = app.exec()
    finally:
        executor.shutdown()
        rclpy.shutdown()
        for p in procs:
            kill_proc(p)
    sys.exit(ret)


if __name__ == '__main__':
    main()
