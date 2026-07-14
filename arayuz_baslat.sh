#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# LAPTOP = ROBOT KOL SİMÜLASYONU (Gazebo köprüsü)
#   • gazebo_bridge  (mini PC logic'inden gelen kartezyen hedefi IK ile
#                     Gazebo eklem komutuna çevirir → kol Gazebo'da hareket)
#   • zenoh köprü    (mini PC'ye bağlanır — arayüz + gerçek donanım orada)
#
# Arayüz (gui) ve gerçek donanım (kamera/CAN/load cell) MİNİ PC'de çalışır.
# Bu makine yalnızca robot kolun yerini tutar (Gazebo simülasyonu).
#
# Sıra:
#   Terminal 1: bash ~/ros2-end-effector/gazebo_baslat.sh          (Gazebo penceresi)
#   Terminal 2: MINI_PC_IP=192.168.1.112 bash ~/ros2-end-effector/arayuz_baslat.sh
# ─────────────────────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# DDS: VARSAYILAN CycloneDDS (eski loopback XML WSL'de düğümleri asıyordu).
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
rm -f "$HOME/cyclonedds.xml" 2>/dev/null
echo "[DDS] Domain ID: $ROS_DOMAIN_ID | CycloneDDS (varsayılan)"

# Kalan zenoh köprüsünü temizle (port 7447 çakışması olmasın)
pkill -9 -f zenoh-bridge-ros2dds 2>/dev/null && sleep 1

if [ -z "$MINI_PC_IP" ]; then
    echo "HATA: MINI_PC_IP verilmedi."
    echo "  Kullanım: MINI_PC_IP=192.168.1.112 bash arayuz_baslat.sh"
    exit 1
fi

echo "[MOD] LAPTOP = ROBOT KOL SİMÜLASYONU — mini PC (arayüz+donanım): $MINI_PC_IP"

# WSL bazen (virtioproxy / Wi-Fi client-isolation) LAN'daki mini PC'ye
# DOĞRUDAN ulaşamaz — ama Windows ulaşır. Windows portproxy (127.0.0.1:7447)
# üzerinden bağlan.
TARGET_IP="$MINI_PC_IP"
if ! timeout 3 bash -c "echo > /dev/tcp/$MINI_PC_IP/7447" 2>/dev/null; then
    if timeout 3 bash -c "echo > /dev/tcp/127.0.0.1/7447" 2>/dev/null; then
        echo "[AĞ] WSL doğrudan $MINI_PC_IP'ye ulaşamıyor → Windows portproxy (127.0.0.1)"
        TARGET_IP="127.0.0.1"
    else
        echo "[AĞ] UYARI: Ne $MINI_PC_IP ne portproxy (127.0.0.1:7447) erişilebilir!"
        echo "       Mini PC açık ve minipc_baslat.sh çalışıyor mu?"
    fi
fi

RUST_LOG=info "$WS/zenoh-bridge-ros2dds" -d 42 -e "tcp/$TARGET_IP:7447" \
    > /tmp/zenoh_bridge.log 2>&1 &
ZENOH_PID=$!
sleep 2
if kill -0 $ZENOH_PID 2>/dev/null; then
    echo "[ZENOH] Köprü çalışıyor (log: /tmp/zenoh_bridge.log)"
else
    echo "[ZENOH] KÖPRÜ BAŞLAMADI! /tmp/zenoh_bridge.log kontrol et"; exit 1
fi

cleanup() { kill $ZENOH_PID 2>/dev/null; }
trap cleanup EXIT INT TERM

# SADECE gazebo_bridge (kol köprüsü). gui/logic/can/vision MİNİ PC'de.
ros2 launch end_effector_ros2 end_effector.launch.py \
    use_gazebo:=true simulation:=false use_real_robot:=false \
    start_gui:=false start_logic:=false start_can:=false start_vision:=false "$@"
