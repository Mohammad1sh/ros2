#!/bin/bash
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="/run/user/$(id -u)"
export ROS_DOMAIN_ID=42

WS="$HOME/ros2-end-effector"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# Fast-DDS loopback XML'i her seferinde /tmp'ye yaz (dosya yok hatası olmasın)
cat > /tmp/fastdds_localhost.xml << 'XMLEOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>loopback</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>127.0.0.1</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="DefaultProfile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>loopback</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
XMLEOF

# CycloneDDS varsa kullan, yoksa Fast-DDS loopback XML ile çalış
if ros2 pkg list 2>/dev/null | grep -q "rmw_cyclonedds"; then
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    echo "[DDS] CycloneDDS kullanılıyor"
else
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_localhost.xml
    echo "[DDS] Fast-DDS loopback kullanılıyor"
fi

unset ROS_LOCALHOST_ONLY

echo "[DDS] Domain ID: $ROS_DOMAIN_ID | RMW: $RMW_IMPLEMENTATION"
python3 "$WS/start_robot.py" 2>&1 | tee /tmp/end_effector_launch.log
