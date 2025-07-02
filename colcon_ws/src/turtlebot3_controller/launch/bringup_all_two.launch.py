#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # --------------------------------------------------------------------
    # 0) Declare launch arguments for gazebo spawn (x, y) and sim time
    # --------------------------------------------------------------------
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_x = DeclareLaunchArgument(
        'x_pose', default_value='2.1',
        description='Initial X position for robot spawn in Gazebo')
    declare_y = DeclareLaunchArgument(
        'y_pose', default_value='0.25',
        description='Initial Y position for robot spawn in Gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # --------------------------------------------------------------------
    # 1) Gazebo world + robot spawn (turtlebot3_home2.launch.py)
    # --------------------------------------------------------------------
    pkg_gz = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'turtlebot3_home2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # --------------------------------------------------------------------
    # 1.1) Static transform: map -> odom (must match spawn x_pose, y_pose)
    # --------------------------------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=[
            # x, y, z offset (match x_pose, y_pose)
            x_pose, y_pose, '0.0',
            # roll, pitch, yaw
            '0.0', '0.0', '0.0',
            'map', 'odom'
        ]
    )

    # --------------------------------------------------------------------
    # 2) Nav2 parameters: load saved SLAM map
    # --------------------------------------------------------------------
    pkg_self = get_package_share_directory('turtlebot3_controller')
    map_yaml = os.path.join(pkg_self, 'maps', 'home2_map.yaml')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg_self, 'config', 'nav2.yaml'),
        root_key='',
        param_rewrites={'yaml_filename': map_yaml},
        convert_types=True,
    )

    # --------------------------------------------------------------------
    # 3) Nodes: map_server, amcl, lifecycle_manager
    # --------------------------------------------------------------------
    nodes = [
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
        ),
        # ----------------------------------------------------------------
        # 4) ros2_control: controller_manager + joint_state_broadcaster +
        #    diff_controller
        # ----------------------------------------------------------------
        Node(
            package='controller_manager', executable='ros2_control_node',
            name='controller_manager', output='screen',
            parameters=[os.path.join(pkg_self, 'config', 'diff_controllers.yaml')],
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['diff_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
        # ----------------------------------------------------------------
        # 5) RViz2 for visualization
        # ----------------------------------------------------------------
        Node(
            package='rviz2', executable='rviz2',
            name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_self, 'config', 'nav2_default_view.rviz')],
        ),
        # ----------------------------------------------------------------
        # 6) Mission or Patrol Manager (your control node)
        # ----------------------------------------------------------------
        Node(
            package='turtlebot3_controller', executable='patrol_manager',
            name='patrol_manager', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    # --------------------------------------------------------------------
    # 7) LaunchDescription: assemble everything
    # --------------------------------------------------------------------
    ld = LaunchDescription()

    # 0) launch args
    ld.add_action(declare_sim_time)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    # 1) Gazebo + spawn
    ld.add_action(gazebo_launch)
    # 1.1) Static TF
    ld.add_action(static_tf)
    # 2–6) other nodes
    for n in nodes:
        ld.add_action(n)

    return ld
