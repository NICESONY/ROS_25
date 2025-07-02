#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

"""
Launch file for TurtleBot3 patrol mission using ROS 2 Navigation Stack.

# 터미널 A: bringup_all_two.launch.py (Gazebo→Nav2→RViz 등)
ros2 launch turtlebot3_controller bringup_all_two.launch.py

# 터미널 B: patrol_mission.launch.py (patrol_manager만)
ros2 launch turtlebot3_controller patrol_mission.launch.py

"""

def generate_launch_description():
    pkg = get_package_share_directory('turtlebot3_controller')

    return LaunchDescription([
        # 1) map_server + amcl + lifecycle (이미 bringup에서 띄웠다면 생략 가능)
        Node(package='nav2_map_server', executable='map_server',
             parameters=[os.path.join(pkg, 'config', 'nav2.yaml')],
             output='screen'),
        Node(package='nav2_amcl', executable='amcl',
             parameters=[os.path.join(pkg, 'config', 'nav2.yaml')],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             parameters=[{'use_sim_time': True,
                          'autostart': True,
                          'node_names': ['map_server','amcl']}]),

        # 2) controller_manager + diff_controller
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[os.path.join(pkg, 'config', 'diff_controllers.yaml')],
             output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['diff_controller'], output='screen'),

        # 3) patrol_manager
        Node(package='turtlebot3_controller', executable='patrol_manager',
             name='patrol_manager', output='screen',
             parameters=[{'use_sim_time': True}]),
    ])
