from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_cfg = LaunchConfiguration('map')
    return LaunchDescription([
        DeclareLaunchArgument('map',
            default_value='src/turtlebot3_controller/maps/home2_map.yaml'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_cfg}],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp_rviz',
            parameters=[{
                'robot_description': open(
                    'install/share/turtlebot3_controller/urdf/turtlebot3_burger.urdf'
                ).read()
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'install/share/turtlebot3_controller/rviz/robot.rviz']
        ),
    ])
