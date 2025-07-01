# ~/ROS/ROS_25/colcon_ws/src/turtlebot3_controller/launch/bringup_core.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share    = get_package_share_directory('turtlebot3_controller')

    # 0. DeclareLaunchArgument 먼저: 기본값 지정
    decl_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    decl_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value=PathJoinSubstitution([pkg_share, 'maps', 'carto_map.yaml']),
        description='Full path to map yaml file'
    )
    decl_params   = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2.yaml']),
        description='Full path to Nav2 parameter file'
    )
    decl_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the Nav2 lifecycle'
    )

    # 1. LaunchConfiguration 생성 (항상 DeclareLaunchArgument 뒤에)
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map_yaml')
    params_file  = LaunchConfiguration('params_file')
    autostart    = LaunchConfiguration('autostart')

    return LaunchDescription([
        # 인자 선언
        decl_sim_time,
        decl_map_yaml,
        decl_params,
        decl_autostart,

        # 디버그: 인자 값 확인
        LogInfo(msg=['[DEBUG] map_yaml    = ', map_yaml]),
        LogInfo(msg=['[DEBUG] params_file = ', params_file]),
        LogInfo(msg=['[DEBUG] autostart   = ', autostart]),

        # 2. Gazebo 시뮬레이션
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

        # 3. Cartographer SLAM
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

        # 4. Nav2 (map_server + AMCL + planner)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('turtlebot3_navigation2'),
                    'launch',
                    'navigation2.launch.py'
                ])
            ),
            launch_arguments={
                'map':          map_yaml,
                'use_sim_time': use_sim_time,
                'params_file':  params_file,
                'autostart':    autostart
            }.items(),
        ),
    ])
