#!/bin/bash
# ─────────────────────────────────────────────────────────────────
# Gazebo + Doosan H2515 + Zımpara başlatıcı (Terminal 1)
# Kullanım:  bash ~/ros2-end-effector/gazebo_baslat.sh
# Sonra Terminal 2'de:  bash ~/ros2-end-effector/arayuz_baslat.sh
# ─────────────────────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export ROS_DOMAIN_ID=42
unset ROS_LOCALHOST_ONLY

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# DDS: launch.sh ile aynı mantık (WSL'de varsayılan keşif kırık)
cat > /tmp/fastdds_localhost.xml << 'XMLEOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>loopback</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="DefaultProfile" is_default_profile="true">
        <rtps>
            <userTransports><transport_id>loopback</transport_id></userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
XMLEOF
if ros2 pkg list 2>/dev/null | grep -q "rmw_cyclonedds"; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "[DDS] CycloneDDS kullanılıyor"
else
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_localhost.xml
    echo "[DDS] Fast-DDS loopback kullanılıyor"
fi
echo "[DDS] Domain ID: $ROS_DOMAIN_ID | RMW: $RMW_IMPLEMENTATION"

# Eski/takılı Gazebo-ROS süreçlerini temizle (çakışan sunucu → beyaz ekran yapar)
pkill -f "ros2 launch end_effector_ros2" 2>/dev/null
pkill -f "ign gazebo" 2>/dev/null
sleep 1
pkill -9 -f "ign gazebo" 2>/dev/null
pkill -9 -f "ruby /usr/bin/ign" 2>/dev/null
sleep 1

# Gazebo GUI GLX istediği için X11 (xcb) şart — Wayland ÇALIŞMAZ
export QT_QPA_PLATFORM=xcb

# GUI, launch içinde ogre1 motoruyla açılır (--render-engine-gui ogre);
# WSLg'de ogre2 çöküyordu. GPU sorun çıkarırsa yazılımsal render'a düş:
#   LIBGL_ALWAYS_SOFTWARE=1 bash gazebo_baslat.sh

ros2 launch end_effector_ros2 gazebo.launch.py "$@"
