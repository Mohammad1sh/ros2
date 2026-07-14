#!/bin/bash
# ═════════════════════════════════════════════════════════════════
# MİNİ PC SIFIRDAN KURULUM — Ubuntu 22.04 üzerinde tek seferlik
# Kullanım:
#   1) Mini PC'ye Ubuntu 22.04 kur (farklı sürümse bana söyle)
#   2) Bu repo klasörünü USB ile ~/ros2-end-effector olarak kopyala
#   3) bash ~/ros2-end-effector/minipc_kurulum.sh
# ═════════════════════════════════════════════════════════════════
set -e

echo "═══ 1/5 ROS2 Humble deposu ═══"
sudo apt update && sudo apt install -y curl gnupg lsb-release software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

echo "═══ 2/5 ROS2 + araçlar + GUI (arayüz mini PC'de açılır) ═══"
sudo apt install -y ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge python3-colcon-common-extensions python3-pip \
    python3-serial v4l-utils unzip \
    python3-pyqt6 libxcb-cursor0        # GUI (gui_node PyQt6 arayüzü)

echo "═══ 3/5 Python bağımlılıkları (YOLO + numpy<2 uyumluluk) ═══"
pip3 install --user ultralytics opencv-python-headless pyserial "numpy<2"

echo "═══ 4/5 USB izin kuralları (kalıcı) ═══"
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-usb-serial.rules
echo 'KERNEL=="video*",  MODE="0666"' | sudo tee /etc/udev/rules.d/99-camera.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "═══ 5/5 Proje derleme ═══"
cd "$HOME/ros2-end-effector"
chmod +x zenoh-bridge-ros2dds minipc_baslat.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select end_effector_ros2

echo ""
echo "══════════════════════════════════════════════"
echo " KURULUM TAMAM ✓"
echo " Çalıştır:  bash ~/ros2-end-effector/minipc_baslat.sh"
echo " (CAN kartı ve kamerayı USB'ye tak, aynı ağa bağlan)"
echo "══════════════════════════════════════════════"
