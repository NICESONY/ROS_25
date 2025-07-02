# turtlebot3_controller/launch/bringup.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg = get_package_share_directory('turtlebot3_controller')
    map_yaml = os.path.join(pkg, 'maps', 'home2_map.yaml')

    # nav2.yaml에서 map 경로만 교체
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2.yaml'),
        root_key='',
        param_rewrites={'yaml_filename': map_yaml},
        convert_types=True
    )

    return LaunchDescription([
        # 1) Map Server
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[nav2_params],
        ),
        # 2) AMCL
        Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', output='screen',
            parameters=[nav2_params],
        ),
        # 3) Lifecycle Manager (map_server, amcl)
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
        ),
        # 4) ros2_control + DiffDrive 스포너
        Node(
            package='controller_manager', executable='ros2_control_node',
            name='controller_manager', output='screen',
            parameters=[os.path.join(pkg, 'config', 'diff_controllers.yaml')],
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['diff_controller'], output='screen'
        ),
        # 5) Mission Manager
        Node(
            package='turtlebot3_controller', executable='mission_manager',
            name='mission_manager', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
