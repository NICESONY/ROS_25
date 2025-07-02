#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 0) Launch arguments
    declare_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true',
                                            description='Use simulation clock')
    declare_x = DeclareLaunchArgument('x_pose', default_value='2.1',
                                      description='Robot spawn X')
    declare_y = DeclareLaunchArgument('y_pose', default_value='0.25',
                                      description='Robot spawn Y')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')

    pkg_gz  = get_package_share_directory('turtlebot3_gazebo')
    pkg_self= get_package_share_directory('turtlebot3_controller')

    # 1) Gazebo + spawn
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'turtlebot3_home2.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # 1.1) map→odom
    static_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_map_to_odom', output='screen',
        arguments=[ x_pose, y_pose, '0.0',
                    '0.0', '0.0', '0.0',
                    'map', 'odom' ]
    )

    # 1.2) base_link→base_scan
    static_base_to_scan = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='static_base_to_scan', output='screen',
        arguments=[ '0.0','0.0','0.1',
                    '0.0','0.0','0.0',
                    'base_link','base_scan' ]
    )

    # 1.3) 초기 포즈 퍼블리시
    initial_pose_publisher = Node(
        package='turtlebot3_controller', executable='initial_pose_publisher',
        name='initial_pose_publisher', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 2) Nav2 params with your map
    map_yaml = os.path.join(pkg_self, 'maps', 'home2_map.yaml')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_self, 'config', 'nav2.yaml'),
        root_key='',
        param_rewrites={'yaml_filename': map_yaml},
        convert_types=True,
    )

    # 3) Nav2 localization
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[nav2_params],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen', parameters=[nav2_params],
    )
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server','amcl']
        }],
    )

    # 4) Nav2 planning & control
    planner_server = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[nav2_params],
    )
    controller_server = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', parameters=[nav2_params],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=[nav2_params],
    )
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server','amcl',
                'planner_server','controller_server','bt_navigator'
            ]
        }],
    )

    # 5) ros2_control
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

    # 6) RViz2
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_self,'config','nav2_default_view.rviz')],
    )

    # 7) PatrolManager
    patrol_manager = Node(
        package='turtlebot3_controller', executable='patrol_manager',
        name='patrol_manager', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Assemble
    ld = LaunchDescription()

    # args
    ld.add_action(declare_sim_time)
    ld.add_action(declare_x)
    ld.add_action(declare_y)

    # bringup
    ld.add_action(gazebo_launch)
    ld.add_action(static_map_to_odom)
    ld.add_action(static_base_to_scan)
    ld.add_action(initial_pose_publisher)

    # localization
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_localization)

    # navigation
    ld.add_action(planner_server)
    ld.add_action(controller_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_navigation)

    # control + rviz + mission
    ld.add_action(controller_manager)
    ld.add_action(js_broadcaster)
    ld.add_action(diff_controller)
    ld.add_action(rviz)
    ld.add_action(patrol_manager)

    return ld
