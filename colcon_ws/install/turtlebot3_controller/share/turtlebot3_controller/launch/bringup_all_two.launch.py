#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 0) Launch arguments
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    declare_x = DeclareLaunchArgument(
        'x_pose', default_value='2.1',
        description='Robot spawn X'
    )
    declare_y = DeclareLaunchArgument(
        'y_pose', default_value='0.25',
        description='Robot spawn Y'
    )
    declare_yaw = DeclareLaunchArgument(
        'yaw_pose', default_value='0.0',
        description='Robot spawn yaw (rad)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')
    yaw_pose     = LaunchConfiguration('yaw_pose')

    # 패키지 경로
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2   = get_package_share_directory('turtlebot3_navigation2')
    pkg_self   = get_package_share_directory('turtlebot3_controller')

    # 1) Gazebo + spawn (x, y, yaw 모두 인자로)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'turtlebot3_home2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose':       x_pose,
            'y_pose':       y_pose,
            'yaw':          yaw_pose,
        }.items()
    )

    # 1.1) map → odom (Static TF)
    static_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom', output='screen',
        arguments=[
            x_pose, y_pose, '0.0',      # x, y, z
            '0.0', '0.0', yaw_pose,     # roll, pitch, yaw
            'map', 'odom'
        ]
    )

    # 1.2) base_link → base_scan (LaserScan frame)
    static_base_to_scan = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_scan', output='screen',
        arguments=[
            '0.0','0.0','0.1',           # sensor offset z
            '0.0','0.0','0.0',           # roll, pitch, yaw
            'base_link','base_scan'
        ]
    )

    # 1.3) 초기 포즈 한 번 퍼블리시하는 노드
    initial_pose_publisher = Node(
        package='turtlebot3_controller',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'x': x_pose},
            {'y': y_pose},
            {'a': yaw_pose},
        ],
    )

    # 2) Official Nav2 stack include
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(pkg_self, 'maps', 'home2_map.yaml'),
            'params_file': os.path.join(pkg_self, 'config', 'nav2.yaml'),
        }.items()
    )

    # 3) ros2_control (controller_manager + broadcasters + diff_controller)
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        name='controller_manager', output='screen',
        parameters=[os.path.join(pkg_self,'config','diff_controllers.yaml')],
    )
    js_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster','--controller-manager','/controller_manager'],
        output='screen',
    )
    diff_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_controller','--controller-manager','/controller_manager'],
        output='screen',
    )

    # 4) PatrolManager (미션 제어 노드)
    patrol_manager = Node(
        package='turtlebot3_controller',
        executable='patrol_manager',
        name='patrol_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Assemble LaunchDescription
    ld = LaunchDescription()

    # launch arguments
    ld.add_action(declare_sim_time)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_yaw)

    # bringup: Gazebo + TF + 초기 pose
    ld.add_action(gazebo_launch)
    ld.add_action(static_map_to_odom)
    ld.add_action(static_base_to_scan)
    ld.add_action(initial_pose_publisher)

    # official Nav2 stack
    ld.add_action(nav2_launch)

    # ros2_control
    ld.add_action(controller_manager)
    ld.add_action(js_broadcaster)
    ld.add_action(diff_controller)

    # 미션 매니저
    ld.add_action(patrol_manager)

    return ld
