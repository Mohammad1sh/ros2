#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# MİNİ PC = ARAYÜZ + GERÇEK DONANIM
#   • gui_node    (kontrol arayüzü — kullanıcı burada oturur)
#   • logic_node  (görev mantığı: tara → çapak → zımparala)
#   • can_node    (GERÇEK CAN kartı: load cell + sander röle)
#   • vision_node (GERÇEK kamera + YOLO)
#   • zenoh köprü (laptop buraya bağlanır)
# Robot KOL simülasyonu LAPTOP'ta (Gazebo). START'a mini PC'de basılır;
# kol hareketi laptop Gazebo'sunda görünür (zenoh üzerinden).
#
# Kullanım (mini PC'de, monitör bağlı):  bash ~/ros2-end-effector/minipc_baslat.sh
# ─────────────────────────────────────────────────────────────────
export ROS_DOMAIN_ID=42
export DISPLAY="${DISPLAY:-:0}"      # GUI için (mini PC monitörü)
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI                  # mini PC native → VARSAYILAN DDS

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Eski süreçleri temizle (port 7447 / düğüm çakışması olmasın)
echo "[TEMİZLİK] Eski süreçler kapatılıyor..."
pkill -f zenoh-bridge-ros2dds 2>/dev/null
pkill -f 'end_effector_ros2' 2>/dev/null
sleep 2

# USB izinleri (udev kuralları kurulumda eklendi; bu sadece yedek)
sudo -n chmod 666 /dev/ttyUSB* 2>/dev/null || true
sudo -n chmod 777 /dev/video0 /dev/video1 2>/dev/null || true

# ── Zenoh köprüsü — laptop (kol sim) buraya bağlanır ──
echo "[ZENOH] Köprü dinlemede: tcp/0.0.0.0:7447"
RUST_LOG=info "$WS/zenoh-bridge-ros2dds" -d 42 -l tcp/0.0.0.0:7447 \
    > /tmp/zenoh_bridge.log 2>&1 &
ZENOH_PID=$!
sleep 2
kill -0 $ZENOH_PID 2>/dev/null || { echo "[ZENOH] BAŞLAMADI! /tmp/zenoh_bridge.log"; exit 1; }

# ── ARAYÜZ + GERÇEK DONANIM (gazebo_bridge YOK — o laptopta) ──
echo "[MİNİ PC] Arayüz + gerçek donanım başlatılıyor (gui+logic+can+vision)..."
ros2 launch end_effector_ros2 end_effector.launch.py \
    simulation:=false use_gazebo:=false use_real_robot:=false \
    start_gui:=true start_logic:=true start_can:=true start_vision:=true \
    can_port:=/dev/ttyUSB0 model_name:=latest.pt &
APP_PID=$!

trap "kill $APP_PID $ZENOH_PID 2>/dev/null" INT TERM EXIT
echo "[MİNİ PC] Çalışıyor. Laptop'ta (KOL SİMÜLASYONU):"
echo "   Terminal 1: bash ~/ros2-end-effector/gazebo_baslat.sh"
echo "   Terminal 2: MINI_PC_IP=$(hostname -I | awk '{print $1}') bash ~/ros2-end-effector/arayuz_baslat.sh"
wait
