#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ───────── Launch arguments ─────────
    declare_sim_time   = DeclareLaunchArgument('use_sim_time', default_value='true',
                                               description='Use simulation clock')
    # ▶ Gazebo 스폰 전용 인자
    declare_spawn_x    = DeclareLaunchArgument('spawn_x', default_value='2.1',
                                               description='Gazebo spawn X')
    declare_spawn_y    = DeclareLaunchArgument('spawn_y', default_value='0.25',
                                               description='Gazebo spawn Y')
    declare_spawn_yaw  = DeclareLaunchArgument('spawn_yaw', default_value='0.0',
                                               description='Gazebo spawn yaw (rad)')
    # ▶ AMCL 초기 포즈 전용 인자
    declare_init_x     = DeclareLaunchArgument('init_x', default_value='0.0',
                                               description='AMCL initial X')
    declare_init_y     = DeclareLaunchArgument('init_y', default_value='0.0',
                                               description='AMCL initial Y')
    declare_init_yaw   = DeclareLaunchArgument('init_yaw', default_value='0.0',
                                               description='AMCL initial yaw (rad)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x      = LaunchConfiguration('spawn_x')
    spawn_y      = LaunchConfiguration('spawn_y')
    spawn_yaw    = LaunchConfiguration('spawn_yaw')
    init_x       = LaunchConfiguration('init_x')
    init_y       = LaunchConfiguration('init_y')
    init_yaw     = LaunchConfiguration('init_yaw')

    # ───────── 패키지 경로 ─────────
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('turtlebot3_navigation2')
    pkg_self   = get_package_share_directory('turtlebot3_controller')

    # ───────── 1) Gazebo + 스폰 ─────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot3_home2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose':       spawn_x,
            'y_pose':       spawn_y,
            'yaw':          spawn_yaw,
        }.items()
    )

    # ───────── 2) base_link → base_scan (레이저 TF) ─────────
    static_base_to_scan = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_scan', output='screen',
        arguments=[
            '0.0','0.0','0.1',
            '0.0','0.0','0.0',
            'base_link','base_scan'
        ]
    )

    # ───────── 3) 초기 포즈 퍼블리시 ─────────
    initial_pose_publisher = Node(
        package='turtlebot3_controller',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'x': init_x},
            {'y': init_y},
            {'a': init_yaw},
        ],
    )

    # ───────── 4) Nav2 (공식 런치 포함) ─────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map':          os.path.join(pkg_self, 'maps', 'home2_map.yaml'),
            'params_file':  os.path.join(pkg_self, 'config', 'nav2.yaml'),
            'rviz':         'true',
        }.items()
    )

    # ───────── 5) ros2_control & 스포너 ─────────
    ctrl_node = Node(
        package='controller_manager', executable='ros2_control_node',
        name='controller_manager', output='screen',
        parameters=[os.path.join(pkg_self,'config','diff_controllers.yaml'),
                    {'use_sim_time': use_sim_time}],
    )
    spawner_js = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager','/controller_manager'],
        output='screen'
    )
    spawner_dd = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_controller',
                   '--controller-manager','/controller_manager'],
        output='screen'
    )

    # ───────── 6) PatrolManager (미션 매니저) ─────────
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
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,
        declare_init_x,
        declare_init_y,
        declare_init_yaw,
        gazebo_launch,
        static_base_to_scan,
        initial_pose_publisher,
        nav2_launch,
        ctrl_node,
        spawner_js,
        spawner_dd,
        patrol_manager,
    ])

    return ld
