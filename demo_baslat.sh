#!/bin/bash
# ─────────────────────────────────────────────────────────────
# DEMO BAŞLATICI — TEK KOMUT:
#   bash ~/ros2-end-effector/demo_baslat.sh
#
# Yaptıkları:
#   1) Gazebo kapalıysa açar, controller'ları bekler
#   2) Zımpara koreografisini oynatır (park → tarama → eşik
#      süpürme → park). Bittiğinde tekrar çalıştırılabilir.
# ─────────────────────────────────────────────────────────────
WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1) Gazebo acik mi?
if ! pgrep -f 'ign gazebo server' > /dev/null; then
    echo "[1/2] Gazebo başlatılıyor (pencere ~30 sn içinde gelir)..."
    setsid nohup bash "$WS/gazebo_baslat.sh" > /tmp/gz_demo.log 2>&1 < /dev/null &
    for i in $(seq 1 90); do
        N=$(grep -c 'activate successful' /tmp/gz_demo.log 2>/dev/null)
        [ "${N:-0}" -ge 3 ] && { echo "      Hazır (${i} sn)."; break; }
        sleep 1
    done
else
    echo "[1/2] Gazebo zaten açık ✓"
fi

# 2) Koreografiyi oynat
echo "[2/2] Koreografi başlıyor — Gazebo penceresini izle!"
python3 -u "$WS/koreografi_oynat.py"
echo ""
echo "Tekrar oynatmak için aynı komutu çalıştır:"
echo "  bash ~/ros2-end-effector/demo_baslat.sh"
