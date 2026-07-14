#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# End-effector arayüzü + Gazebo köprüsü (Terminal 2)
# Önce Terminal 1'de: bash ~/ros2-end-effector/gazebo_baslat.sh
#
# YEREL sim (varsayılan):    bash arayuz_baslat.sh
# UZAK GERÇEK DONANIM modu:  MINI_PC_IP=192.168.1.50 bash arayuz_baslat.sh
#   → kamera + CAN (load cell, röle) mini PC'de; kol Gazebo'da.
#   Bağlantı zenoh köprüsüyle (tek giden TCP) — WSL ağı için ideal.
# ─────────────────────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# DDS: VARSAYILAN CycloneDDS. Eski loopback XML (unicast peer / multicast
# kapalı) WSL'de node'ları asıyordu — ros2 launch'ın ayrı process'leri
# birbirini bulamıyordu. Varsayılan CycloneDDS ile inter-process keşif
# sorunsuz çalışıyor (test edildi). Makineler arası taşımayı zenoh yapar;
# WSL zaten LAN'a L2 erişemediği için mini PC ile doğrudan DDS keşfi olmaz.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
rm -f "$HOME/cyclonedds.xml" 2>/dev/null
echo "[DDS] Domain ID: $ROS_DOMAIN_ID | CycloneDDS (varsayılan)"

ZENOH_PID=""
# Önceki oturumdan kalan zenoh köprüsü 7447'yi tutarsa yeni köprü
# "address in use" ile ölür → bu temizlik onu engeller.
pkill -9 -f zenoh-bridge-ros2dds 2>/dev/null && sleep 1

if [ -n "$MINI_PC_IP" ]; then
    echo "[MOD] UZAK GERÇEK DONANIM — mini PC: $MINI_PC_IP (kamera+CAN orada)"

    # WSL bazen (NAT/virtioproxy modunda, Wi-Fi client-isolation) yerel LAN'daki
    # mini PC'ye DOĞRUDAN ulaşamaz — ama Windows ulaşır. Bu durumda Windows'ta
    # kurulu portproxy (0.0.0.0:7447 → mini PC:7447) üzerinden 127.0.0.1 kullan.
    TARGET_IP="$MINI_PC_IP"
    if ! timeout 3 bash -c "echo > /dev/tcp/$MINI_PC_IP/7447" 2>/dev/null; then
        if timeout 3 bash -c "echo > /dev/tcp/127.0.0.1/7447" 2>/dev/null; then
            echo "[AĞ] WSL doğrudan $MINI_PC_IP'ye ulaşamıyor → Windows portproxy (127.0.0.1) kullanılıyor"
            TARGET_IP="127.0.0.1"
        else
            echo "[AĞ] UYARI: Ne $MINI_PC_IP'ye ne de portproxy'ye (127.0.0.1:7447) ulaşılamıyor!"
            echo "       Mini PC açık ve minipc_baslat.sh çalışıyor mu? Portproxy kurulu mu?"
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
    EXTRA_ARGS="simulation:=false start_can:=false start_vision:=false"
else
    echo "[MOD] Yerel simülasyon"
    EXTRA_ARGS="simulation:=true"
fi

cleanup() { [ -n "$ZENOH_PID" ] && kill $ZENOH_PID 2>/dev/null; }
trap cleanup EXIT INT TERM

ros2 launch end_effector_ros2 end_effector.launch.py \
    use_gazebo:=true $EXTRA_ARGS "$@"
