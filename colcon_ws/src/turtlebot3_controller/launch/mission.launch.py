# ~/.../launch/mission.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('turtlebot3_controller')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'bringup_core.launch.py'])
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'map_yaml': PathJoinSubstitution([pkg, 'maps', 'carto_map.yaml'])
            }.items(),
        ),
        Node(
            package='turtlebot3_controller',
            executable='mission_manager',
            name='mission_manager',
            output='screen'
        ),
    ])
