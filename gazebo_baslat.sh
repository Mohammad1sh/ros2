#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# GAZEBO + DEMO BAŞLATICI (TEK KOMUT)
#   bash ~/ros2-end-effector/gazebo_baslat.sh
#
# Yaptıkları:
#   1) Gazebo simülasyonunu başlatır (H2515 + zımpara + araba)
#   2) Zenoh köprüsünü açar (mini PC bağlantısı)
#   3) AKILLI DİNLEYİCİ: mini PC'de START'a basılınca kol tarama
#      pozuna gider, GERÇEK kamera tespitlerine göre çapak
#      bölgelerini 1cm/5sn kuralıyla zımparalar, parka döner.
#      Tespit yoksa sadece geri çekilir.
#
# NOT: Eski "sadece Gazebo" davranışı: bash gazebo_baslat.sh sade
# ─────────────────────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISTRO=humble
export QT_QPA_PLATFORM=xcb

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Eski süreçleri temizle
pkill -f "ros2 launch end_effector_ros2" 2>/dev/null
pkill -f "ign gazebo" 2>/dev/null
pkill -f akilli_dinleyici 2>/dev/null
pkill -9 -f gui_guard 2>/dev/null
pkill -f koreografi_oynat 2>/dev/null
sleep 1
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "ruby /usr/bin/ign" 2>/dev/null
sleep 1

# ── 1) Gazebo (arka planda) ──
echo "[1/3] Gazebo başlatılıyor (pencere ~30 sn)..."
setsid nohup ros2 launch end_effector_ros2 gazebo.launch.py > /tmp/gz_launch.log 2>&1 < /dev/null &
for i in $(seq 1 90); do
    N=$(grep -c 'activate successful' /tmp/gz_launch.log 2>/dev/null)
    [ "${N:-0}" -ge 2 ] && { echo "      Controller'lar hazır (${i} sn)."; break; }
    sleep 1
done

# sade mod: sadece Gazebo istenirse burada dur

# ── [1.5] GUI BEKCISI: GUI cokse/kapansa da sunucu YASAR; 3 sn'de yeni pencere ──
pkill -9 -f 'gui_guard' 2>/dev/null
pkill -9 -f 'ign gazebo -g' 2>/dev/null
cat > /tmp/gui_guard.sh << 'GGEOF'
#!/bin/bash
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export QT_QPA_PLATFORM=xcb
while true; do
  ign gazebo -g --render-engine-gui ogre >> /tmp/gui.log 2>&1
  sleep 3
done
GGEOF
chmod +x /tmp/gui_guard.sh
: # birlesik modda GUI bekcisi kapali (GUI launch icinde)
echo "[1.5/3] Birlesik mod: GUI launch icinde acilacak"

if [ "$1" = "sade" ]; then
    echo "[SADE] Sadece Gazebo çalışıyor. Çıkmak için Ctrl+C."
    wait; exit 0
fi

# ── 2) Zenoh (mini PC bağlantısı) — BEKÇİLİ: düşerse 3sn'de yeniden bağlanır ──
pkill -9 -f zenoh-bridge-ros2dds 2>/dev/null
pkill -9 -f zenoh_guard 2>/dev/null
sleep 1
cat > /tmp/zenoh_guard.sh << 'GEOF'
#!/bin/bash
cd "$HOME/ros2-end-effector"
while true; do
  RUST_LOG=warn ./zenoh-bridge-ros2dds -d 42 -e tcp/127.0.0.1:7447 >> /tmp/zenoh_demo.log 2>&1
  echo "[GUARD] zenoh dustu, yeniden..." >> /tmp/zenoh_demo.log
  sleep 3
done
GEOF
chmod +x /tmp/zenoh_guard.sh
setsid nohup bash /tmp/zenoh_guard.sh > /dev/null 2>&1 < /dev/null &
sleep 3
if pgrep -f zenoh-bridge-ros2dds > /dev/null; then
    echo "[2/3] Zenoh köprüsü açık (mini PC bağlantısı) ✓"
else
    echo "[2/3] UYARI: zenoh başlamadı — mini PC'ye ulaşılamayabilir."
fi

# ── 3) Akıllı dinleyici (ön planda, BEKÇİLİ: düşerse yeniden başlar) ──
echo "[3/3] Akıllı dinleyici başlıyor..."
while true; do
    python3 -u "$WS/akilli_dinleyici.py"
    echo "[GUARD] dinleyici kapandı — 3 sn sonra yeniden başlıyor (çıkmak: Ctrl+C)"
    sleep 3
done
