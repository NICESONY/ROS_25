# ~/ROS/ROS_25/colcon_ws/src/turtlebot3_controller/launch/bringup_core.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share  = get_package_share_directory('turtlebot3_controller')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml  = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        # 0. 인자 선언
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([pkg_share, 'maps', 'carto_map.yaml']),
            description='Path to map yaml file'
        ),

        # 1. Gazebo 시뮬레이션
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 2. Cartographer SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 3. Nav2 (map_server + AMCL + planner)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('turtlebot3_navigation2'),
                    'launch',
                    'navigation2.launch.py'
                ])
            ),
            launch_arguments={
                'map':           map_yaml,
                'use_sim_time':  use_sim_time
            }.items(),
        ),

        # (옵션) 여기에 센서 트리거 등 Core 서비스만 추가할 수 있습니다.
    ])
