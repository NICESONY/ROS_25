#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
최종 구조
bringup_all_two.launch.py

Gazebo world + spawn

Static TFs (map→odom, base_link→base_scan)

초기 포즈 퍼블리시

Nav2 (navigation2.launch.py)

ros2_control + spawners

patrol_mission.launch.py

PatrolManager 노드만 실행

이렇게 분리해 두면, 터미널 A에서 Bringup, 터미널 B에서 PatrolMission을 띄워도 점유나 충돌 없이 깔끔하게 동작합니다.

"""

def generate_launch_description():
    # ───────── Launch arguments ─────────
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ───────── 패키지 경로 ─────────
    pkg_self = get_package_share_directory('turtlebot3_controller')

    # ───────── PatrolManager 노드 ─────────
    patrol_manager = Node(
        package='turtlebot3_controller',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ───────── LaunchDescription 조립 ─────────
    ld = LaunchDescription([
        declare_sim_time,
        patrol_manager,
    ])

    return ld
