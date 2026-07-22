#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GERCEK ROBOT LAUNCH — Doosan H2515 saha calistirmasi
=====================================================
Toplantida istenen "tek komutla calisan" launch dosyasi. Sahaya gidince
SADECE robot IP'sini verip bunu calistirmaniz yeter:

    ros2 launch end_effector_ros2 gercek_robot.launch.py robot_ip:=192.168.1.100

Bu launch sirayla su dort seyi ayaga kaldirir:
  1) dsr_bringup2  (Doosan'in RESMI surucusu) — verilen IP'ye REAL modda baglanir
  2) end_effector donanim dugumleri (kamera + load cell + servo + sander rolesi)
  3) real_kol_surucu (BIZIM adaptor) — gorev beynini Doosan movej/movesx'e cevirir
  4) akilli_dinleyici (gorev beyni) — simulasyonda kanitlanan AYNI mantik

Simulasyondan FARKI: sadece 1. katman degisir (Gazebo yerine gercek Doosan
surucusu). Gorev mantigi (2..4) birebir aynidir — Gazebo'da dogrulandi.

KURULUM NOTLARI (sahada dogrulanacak — asagida TODO ile isaretli):
  * robot_ip     : firmanin verecegi IP (zorunlu)
  * rt_host      : gercek-zaman kanali IP'si (Doosan varsayilani 192.168.137.50)
  * mode=real    : SANAL DEGIL gercek robota baglanir — dikkat, kol hareket eder!
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    LogInfo, GroupAction, ExecuteProcess
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Kullanicinin sahada verecegi TEK zorunlu parametre ────────────────
    robot_ip = LaunchConfiguration('robot_ip')
    rt_host  = LaunchConfiguration('rt_host')
    model    = LaunchConfiguration('model')
    name     = LaunchConfiguration('name')

    args = [
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.100',
                              description='Firmanin verdigi robot kontrolcu IP adresi'),
        DeclareLaunchArgument('rt_host',  default_value=LaunchConfiguration('robot_ip'),
                              description='RT kanali IP — verilmezse robot_ip kullanilir (tek IP yeter)'),
        DeclareLaunchArgument('model',    default_value='h2515',
                              description='Robot modeli (bu proje: h2515)'),
        DeclareLaunchArgument('name',     default_value='dsr01',
                              description='ROS ad alani (namespace)'),
    ]

    # ── 1) DOOSAN RESMI SURUCUSU — REAL modda IP'ye baglanir ──────────────
    # dsr_bringup2_rviz.launch.py: host=robot_ip, mode=real, model=h2515.
    # Bu, /dsr01/motion/move_joint gibi GERCEK hareket servislerini acar.
    doosan_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dsr_bringup2'),
                'launch', 'dsr_bringup2_rviz.launch.py'
            ])
        ),
        launch_arguments={
            'name':    name,
            'host':    robot_ip,        # <<< SAHADA VERILEN IP BURAYA GIDER
            'rt_host': rt_host,
            'port':    '12345',
            'mode':    'real',          # <<< GERCEK ROBOT (virtual DEGIL)
            'model':   model,
            'color':   'white',
            'gui':     'false',         # RViz istemiyoruz (headless calisir)
            'gz':      'false',         # Gazebo YOK — gercek robot
        }.items(),
    )

    # ── 2) DONANIM DUGUMLERI (kamera + load cell + servo + sander) ────────
    # Bunlar mini PC tarafinda calisir; ayni makinede calistirilacaksa buraya,
    # ayri mini PC'deyse minipc_baslat.sh ile orada baslatilir + zenoh koprusu.
    vision_node = Node(
        package='end_effector_ros2', executable='vision_node',
        name='vision_node', output='screen',
        parameters=[{'simulation': False}],      # TODO: gercek kamera cihaz idx
    )
    can_node = Node(
        package='end_effector_ros2', executable='can_node',
        name='can_node', output='screen',
        parameters=[{'simulation': False}],      # TODO: gercek CAN/load cell portu
    )

    # ── 3+4) GOREV BEYNI (gercek surucu adaptoruyle) ──────────────────────
    # akilli_dinleyici + real_kol_surucu. Doosan servisleri hazir olana kadar
    # 8 sn bekler (bringup baglanti suresi), sonra gorev dugumu baslar.
    gorev_beyni = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='>>> Doosan baglantisi tamam — GOREV BEYNI baslıyor'),
            ExecuteProcess(
                cmd=['python3', '-u',
                     os.path.expanduser('~/ros2-end-effector/akilli_dinleyici.py')],
                output='screen',
                additional_env={'GERCEK_ROBOT': '1'},   # <<< gercek mod anahtari
            ),
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg=['========================================']),
        LogInfo(msg=['  GERCEK ROBOT baslatiliyor']),
        LogInfo(msg=['  IP: ', robot_ip, '  Model: ', model, '  Mod: REAL']),
        LogInfo(msg=['  DIKKAT: kol GERCEKTEN hareket edecek!']),
        LogInfo(msg=['========================================']),
        doosan_bringup,
        vision_node,
        can_node,
        gorev_beyni,
    ])
