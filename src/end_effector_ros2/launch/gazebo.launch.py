#!/usr/bin/env python3
"""
gazebo.launch.py
================
Gazebo (Ignition Fortress) simülasyonunu başlatır:
Doosan H2515 + zımpara end-effector + (opsiyonel) araç şasesi.

Kullanım:
  ros2 launch end_effector_ros2 gazebo.launch.py
  ros2 launch end_effector_ros2 gazebo.launch.py spawn_car:=false

Sıra:
  Terminal 1: ros2 launch end_effector_ros2 gazebo.launch.py
  Terminal 2: ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_gazebo:=true

Mimari:
  - Robot URDF'i urdf/robot_with_sander.urdf.xacro'dan üretilir (zımpara dahil).
  - ros2_control plugin'i /gz namespace'inde çalışır → controller manager:
    /gz/controller_manager. gazebo_bridge de /gz/... topic'lerini kullanır.
  - Controller tanımları: config/sander_controllers.yaml
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    AppendEnvironmentVariable, TimerAction, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('end_effector_ros2')
    dsr_share = get_package_share_directory('dsr_description2')

    # ── Gazebo kaynak yolları ─────────────────────────────────────────────
    # package://end_effector_ros2/... ve package://dsr_description2/...
    # URI'larının çözülebilmesi için her iki paketin share üst dizini,
    # model://arac_sase için de models dizini eklenir.
    resource_paths = os.pathsep.join([
        os.path.abspath(os.path.join(pkg_share, '..')),
        os.path.abspath(os.path.join(dsr_share, '..')),
        os.path.join(pkg_share, 'models'),
    ])
    env_actions = [
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_paths),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_paths),
    ]

    # ── Argümanlar ────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('spawn_car', default_value='true',
            description='Araç şasesini (arac_sase) dünyaya ekle'),
    ]

    # ── URDF üretimi (xacro) ─────────────────────────────────────────────
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_with_sander.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    urdf_path = '/tmp/doosan_sander.urdf'
    with open(urdf_path, 'w') as f:
        f.write(robot_desc)

    # ── robot_state_publisher ────────────────────────────────────────────
    # gz namespace'inde: ign_ros2_control plugin'i robot_description'ı
    # /gz/robot_state_publisher parametre servisinden okur.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='gz',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # ── Gazebo (Ignition) ────────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        # --render-engine-gui ogre: WSLg'de ogre2(ogre-next) GUI çöküyor
        # (GL3PlusTextureGpu::copyTo / GLX sorunları) → GUI ogre1 kullanır
        # zimpara_dunyasi.sdf: gölgesiz + 2ms fizik (performans)
        launch_arguments={'gz_args':
            f"-s -r -v 3 {os.path.join(pkg_share, 'worlds', 'zimpara_dunyasi.sdf')}"
        }.items(),
    )

    # ── Robotu spawn et ──────────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'doosan_with_sander', '-file', urdf_path],
    )

    # ── Araç şasesi (opsiyonel) ──────────────────────────────────────────
    spawn_car = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'arac_sase',
            '-file', os.path.join(pkg_share, 'models', 'arac_sase', 'model.sdf'),
            '-x', '-0.52', '-y', '0.78', '-z', '0.27',
        ],
        condition=IfCondition(LaunchConfiguration('spawn_car')),
    )

    # ── /clock köprüsü ───────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # ── Controller spawner'ları (/gz/controller_manager) ────────────────
    def spawner(controller, delay):
        return TimerAction(
            period=delay,
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller,
                    '--controller-manager', '/gz/controller_manager',
                    '--controller-manager-timeout', '120',
                ],
            )],
        )

    jsb_spawner     = spawner('joint_state_broadcaster',      6.0)
    dsr_spawner     = spawner('dsr_position_controller',      8.0)
    zimpara_spawner = spawner('zimpara_velocity_controller',  8.0)

    log = LogInfo(msg=[
        '\n',
        '╔══════════════════════════════════════════╗\n',
        '║   Gazebo: H2515 + Zımpara Başlatıldı     ║\n',
        '╚══════════════════════════════════════════╝\n',
        '  Controller manager : /gz/controller_manager\n',
        '  Sonraki adım:\n',
        '  ros2 launch end_effector_ros2 end_effector.launch.py \\\n',
        '    simulation:=true use_gazebo:=true\n',
    ])

    return LaunchDescription([
        *env_actions,
        *args,
        log,
        rsp_node,
        gazebo_launch,
        clock_bridge,
        spawn_robot,
        spawn_car,
        jsb_spawner,
        dsr_spawner,
        zimpara_spawner,
    ])
