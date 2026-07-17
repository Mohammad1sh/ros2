#!/bin/bash
# ─────────────────────────────────────────────────────────────
# DEMO BAŞLATICI — TEK KOMUT:
#   bash ~/ros2-end-effector/demo_baslat.sh          → her şeyi kur + START bekle
#   bash ~/ros2-end-effector/demo_baslat.sh oynat    → START beklemeden hemen oynat
#
# Yaptıkları:
#   1) Gazebo kapalıysa açar, controller'ları bekler
#   2) Zenoh köprüsünü açar (mini PC bağlantısı — START sinyali için)
#   3) IK köprüsünü (gazebo_bridge) KAPALI tutar (koreografiyle çakışmasın)
#   4) Mini PC'de START'a basılınca koreografiyi oynatır
# ─────────────────────────────────────────────────────────────
WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1) Gazebo
if ! pgrep -f 'ign gazebo server' > /dev/null; then
    echo "[1/3] Gazebo başlatılıyor (pencere ~30 sn içinde gelir)..."
    setsid nohup bash "$WS/gazebo_baslat.sh" > /tmp/gz_demo.log 2>&1 < /dev/null &
    for i in $(seq 1 90); do
        N=$(grep -c 'activate successful' /tmp/gz_demo.log 2>/dev/null)
        [ "${N:-0}" -ge 3 ] && { echo "      Hazır (${i} sn)."; break; }
        sleep 1
    done
else
    echo "[1/3] Gazebo zaten açık ✓"
fi

# hemen oynat modu
if [ "$1" = "oynat" ]; then
    echo "[OYNAT] Koreografi başlıyor — Gazebo'yu izle!"
    exec python3 -u "$WS/koreografi_oynat.py"
fi

# 2) IK koprusunu kapat (koreografiyle yarismasin) + zenoh'u tazele
pkill -9 -f 'gazebo_bridge --ros-args' 2>/dev/null
pkill -9 -f 'end_effector.launch.py' 2>/dev/null
pkill -9 -f zenoh-bridge-ros2dds 2>/dev/null
sleep 1
RUST_LOG=warn setsid nohup "$WS/zenoh-bridge-ros2dds" -d 42 -e tcp/127.0.0.1:7447 \
    > /tmp/zenoh_demo.log 2>&1 < /dev/null &
sleep 3
if pgrep -f zenoh-bridge-ros2dds > /dev/null; then
    echo "[2/3] Zenoh köprüsü açık (mini PC bağlantısı) ✓"
else
    echo "[2/3] UYARI: zenoh başlamadı — mini PC START'ı ulaşamaz."
    echo "      (Sorun değil: 'bash demo_baslat.sh oynat' ile elle oynatabilirsin)"
fi

# 3) START dinleyici (on planda kalir)
echo "[3/3] START dinleyici başlıyor..."
exec python3 -u "$WS/start_dinleyici.py"
