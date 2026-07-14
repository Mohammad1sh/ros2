#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# MİNİ PC tarafı: GERÇEK donanım düğümleri (kamera + CAN kartı)
# Kullanım (mini PC'de, kurulum sonrası):
#   bash ~/ros2-end-effector/minipc_baslat.sh
# Laptop bağlantısı zenoh üzerinden GELİR (laptop bu makineye bağlanır)
# — mini PC sadece 7447 portunu dinler, IP bilmesine gerek yok.
# ─────────────────────────────────────────────────────────────────
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# ÖNCEKİ oturumdan kalan süreçleri temizle — yoksa eski zenoh 7447 portunu
# tutar, yeni köprü "address in use" ile ölür ve script exit 1 ile durur
# (can_node/vision_node hiç başlamaz). Bu, veri akmamasının en sık sebebi.
echo "[TEMİZLİK] Eski süreçler kapatılıyor..."
pkill -f zenoh-bridge-ros2dds 2>/dev/null
pkill -f 'end_effector_ros2 can_node' 2>/dev/null
pkill -f 'end_effector_ros2 vision_node' 2>/dev/null
sleep 2

# Mini PC native Ubuntu — VARSAYILAN DDS keşfi yeterli (loopback XML
# sadece laptop'taki WSL için gerekliydi). Böylece can_node, vision_node
# ve zenoh köprüsü birbirini garantili bulur.
unset CYCLONEDDS_URI

# USB izinleri (udev kuralları kurulumda eklendi; bu sadece yedek)
sudo -n chmod 666 /dev/ttyUSB* 2>/dev/null || true
sudo -n chmod 777 /dev/video0 /dev/video1 2>/dev/null || true

echo "[ZENOH] Köprü dinlemede: tcp/0.0.0.0:7447"
RUST_LOG=info "$WS/zenoh-bridge-ros2dds" -d 42 -l tcp/0.0.0.0:7447 \
    > /tmp/zenoh_bridge.log 2>&1 &
ZENOH_PID=$!
sleep 2
kill -0 $ZENOH_PID 2>/dev/null || { echo "[ZENOH] BAŞLAMADI! /tmp/zenoh_bridge.log"; exit 1; }

echo "[MİNİ PC] can_node + vision_node başlatılıyor (GERÇEK donanım)..."
ros2 run end_effector_ros2 can_node --ros-args \
    -p port:=/dev/ttyUSB0 -p baudrate:=2000000 \
    -p simulation:=false -p use_dsr2:=false -p use_soem:=false &
CAN_PID=$!

ros2 run end_effector_ros2 vision_node --ros-args \
    -p camera_index:=0 -p model_name:=latest.pt -p stream_fps:=20 &
VIS_PID=$!

trap "kill $CAN_PID $VIS_PID $ZENOH_PID 2>/dev/null" INT TERM EXIT
echo "[MİNİ PC] Çalışıyor. Laptop'ta: MINI_PC_IP=$(hostname -I | awk '{print $1}') bash arayuz_baslat.sh"
wait
