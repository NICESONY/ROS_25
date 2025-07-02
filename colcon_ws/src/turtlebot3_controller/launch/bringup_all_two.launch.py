import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 1) Gazebo world launch
    pkg_gz = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'turtlebot3_home2.launch.py')
        )
    )

    # 2) Spawn robot in Gazebo at fixed pose (x, y, yaw)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '2.0',
            '-z', '0.0',
            '-Y', '1.57'
        ],
    )

    # 3) Optional: static transform map -> odom
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=[
            '1.0', '2.0', '0.0',  # x, y, z
            '0.0', '0.0', '1.57', # roll, pitch, yaw
            'map', 'odom'
        ]
    )

    # 4) Nav2 parameters rewrite for map file
    pkg = get_package_share_directory('turtlebot3_controller')
    map_yaml_path = os.path.join(pkg, 'maps', 'home2_map.yaml')
    nav2_params = RewrittenYaml(
        source_file=os.path.join(pkg, 'config', 'nav2.yaml'),
        root_key='',
        param_rewrites={'yaml_filename': map_yaml_path},
        convert_types=True,
    )

    # 5) Navigation2, control, RViz, and mission manager nodes
    nodes = [
        # Map server
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[nav2_params],
        ),
        # AMCL localization
        Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', output='screen',
            parameters=[nav2_params],
        ),
        # Lifecycle manager for localization
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
        ),
        # ros2_control main node
        Node(
            package='controller_manager', executable='ros2_control_node',
            name='controller_manager', output='screen',
            parameters=[os.path.join(pkg, 'config', 'diff_controllers.yaml')],
        ),
        # diff drive controller spawner
        Node(
            package='controller_manager', executable='spawner',
            arguments=['diff_controller'],
            output='screen'
        ),
        # RViz2 for visualization
        Node(
            package='rviz2', executable='rviz2',
            name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg, 'config', 'nav2_default_view.rviz')],
        ),
        # Mission manager control node
        Node(
            package='turtlebot3_controller', executable='mission_manager',
            name='mission_manager', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    return LaunchDescription([
        gazebo_launch,
        spawn_entity,
        #static_tf,  # Uncomment if static transform is desired
        *nodes
    ])
