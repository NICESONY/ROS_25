from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration('map_yaml', default=PathJoinSubstitution([
        get_package_share_directory('turtlebot3_controller'),
        'maps', 'carto_map.yaml'
    ]))

    return LaunchDescription([
        
        # 0. 시뮬레이션 시간 사용 여부 설정
        # 이 설정은 Cartographer와 Nav2에서 시뮬레이션 시간을 사용하도록 합니다.
        # 실제 로봇에서는 false로 설정해야 합니다.
        # 시뮬레이션에서는 true로 설정합니다.

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # 0. Map YAML 파일 경로 설정
        # 이 경로는 Cartographer가 생성한 YAML 파일이어야 합니다.
        # 예시: turtlebot3_controller/maps/carto_map.yaml

        DeclareLaunchArgument(
            'map_yaml',
            default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot3_controller'),
            'maps',
            'carto_map.yaml'
        ]),
        description='Full path to map yaml file'
        ),
        # 1. Gazebo 시뮬레이션
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_gazebo'),
                '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # 2. SLAM (Cartographer)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_cartographer'),
                '/launch/cartographer.launch.py'
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        # 3. Map Saver (optional, 자동 저장)
        # 4. Map Server + AMCL + Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_navigation2'),
                '/launch/navigation2.launch.py'
            ]),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time
            }.items(),
        ),
        # 5. Teleop
        Node(package='turtlebot3_teleop', executable='teleop_keyboard', name='teleop_keyboard'),
        # 6. Sensor Listener
        Node(package='turtlebot3_controller', executable='sensor_listener', name='sensor_listener'),
        # 7. Obstacle Avoider
        Node(
            package='turtlebot3_controller',
            executable='avoid',
            name='obstacle_avoider',
            parameters=[{
                'safety_distance': 0.5,   # [m]
                'forward_speed':   0.05,  # [m/s]
                'turn_speed':      0.05,
                'use_sim_time':    use_sim_time
            }]
        ),
        # 8. Navigator
        Node(package='turtlebot3_controller', executable='navigate', name='navigator'),
    ])
