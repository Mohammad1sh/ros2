#!/bin/bash
# CHECKPOINT'E TAM DONUS — "eski haline don" dendiginde calistirilir.
# 1) kodu checkpoint-demo etiketine dondurur
# 2) install kopyalarini (git disinda kalan URDF/launch) src'den tazeler
# 3) tum yigini temiz baslatir
set -e
R=/home/sheik/ros2-end-effector
cd $R
git reset --hard checkpoint-demo
git clean -fd --exclude=zenoh-bridge-ros2dds --exclude='*.pt' src/ 2>/dev/null || true
# install tazele (gitignore disinda kaldigi icin elle):
cp $R/src/end_effector_ros2/urdf/robot_with_sander.urdf.xacro \
   $R/install/end_effector_ros2/share/end_effector_ros2/urdf/
cp $R/src/end_effector_ros2/launch/gazebo.launch.py \
   $R/install/end_effector_ros2/share/end_effector_ros2/launch/
cp $R/src/end_effector_ros2/models/arac_sase/model.sdf \
   $R/install/end_effector_ros2/share/end_effector_ros2/models/arac_sase/ 2>/dev/null || true
echo "KOD checkpoint-demo'ya dondu."
# temiz baslat
pkill -9 -f 'akilli_dinleyici' 2>/dev/null || true
pkill -9 -f 'gazebo_baslat' 2>/dev/null || true
pkill -9 -f 'gui_guard' 2>/dev/null || true
pkill -9 -f 'zenoh-bridge' 2>/dev/null || true
pkill -9 -f 'ign gazebo' 2>/dev/null || true
sleep 3
rm -f /home/sheik/demo.log
setsid nohup bash $R/gazebo_baslat.sh > /home/sheik/demo.log 2>&1 < /dev/null &
echo "Yigin yeniden baslatildi — ~60 sn icinde HAZIR."
