#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# DOOSAN H2515 EMÜLATÖRÜ (DRCF, Docker) + ROS2 bringup
# Gerçek robot yerine yazılım emülatörü — movel/movej test edilebilir.
#
# Kullanım:  bash ~/ros2-end-effector/emulator_baslat.sh
# Docker Desktop açık + WSL entegrasyonu etkin olmalı.
# ─────────────────────────────────────────────────────────────────
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export DISPLAY="${DISPLAY:-:0}"
export QT_QPA_PLATFORM=xcb
export MODEL=h2515

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

pkill -9 -f dsr_bringup2 2>/dev/null; pkill -9 -f rviz2 2>/dev/null; sleep 1

# ── Emülatör container: çalışmıyorsa başlat ──
if docker ps --format '{{.Names}}' | grep -qx emulator; then
    echo "[EMÜLATÖR] Zaten çalışıyor."
else
    docker rm -f emulator 2>/dev/null
    echo "[EMÜLATÖR] Başlatılıyor (H2515)..."
    docker run -dit --privileged --name emulator \
        --env ROBOT_MODEL=H2515 -p 12345:12345 \
        doosanrobot/dsr_emulator:3.0.1 >/dev/null
fi

# ── DRCF boot'u bekle (12345 açılana kadar, en fazla 60sn) ──
echo "[EMÜLATÖR] DRCF hazırlanıyor (port 12345 bekleniyor)..."
for i in $(seq 1 60); do
    if timeout 2 bash -c 'echo > /dev/tcp/127.0.0.1/12345' 2>/dev/null; then
        echo "[EMÜLATÖR] Hazır ✓ (${i}sn)"
        break
    fi
    sleep 1
done

# ── ROS2 bringup — emülatöre mode:=real ile bağlan (yeniden başlatmaz) ──
echo "[BRINGUP] H2515 emülatörüne bağlanılıyor..."
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real host:=127.0.0.1 port:=12345 model:=h2515
