#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1) Gazebo world + 로봇 스폰 포함된 turtlebot3_home2.launch.py 사용
    pkg_gz = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'turtlebot3_home2.launch.py')
        )
    )

    # 2) Nav2 파라미터 준비 (저장해둔 SLAM 맵 사용)
    pkg_self = get_package_share_directory('turtlebot3_controller')
    map_yaml_path = os.path.join(pkg_self, 'maps', 'home2_map.yaml')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_self, 'config', 'nav2.yaml'),
        root_key='',
        param_rewrites={'yaml_filename': map_yaml_path},
        convert_types=True,
    )

    # 3) Nav2, ros2_control, RViz2, MissionManager 노드들
    nodes = [
        # map_server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params],
        ),
        # AMCL localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
        ),
        # Lifecycle manager for map_server + amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
        ),
        # ros2_control main node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[os.path.join(pkg_self, 'config', 'diff_controllers.yaml')],
        ),
        # diff drive controller spawner
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_controller'],
            output='screen'
        ),
        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_self, 'config', 'nav2_default_view.rviz')],
        ),
        # Mission manager control node
        Node(
            package='turtlebot3_controller',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    return LaunchDescription([
        gazebo_launch,
        *nodes
    ])
