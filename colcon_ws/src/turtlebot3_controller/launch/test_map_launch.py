#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_controller')
    world_file = os.path.join(pkg_share, 'worlds', 'my_test_map.world')

    # 1) Gazebo 서버: my_test_map.world 로드
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2) Gazebo 클라이언트 (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            )
        )
    )

    # 3) 실제 TurtleBot3 스폰: spawn_turtlebot3.launch.py 호출
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_turtlebot3.launch.py')
        )
    )

    return LaunchDescription([gzserver, gzclient, spawn])
