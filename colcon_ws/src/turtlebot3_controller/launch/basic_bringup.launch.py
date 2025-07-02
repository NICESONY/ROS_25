from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # robot_state_publisher (URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='rsp_basic',
            output='screen',
            parameters=[{
                'robot_description': open(
                    'install/share/turtlebot3_controller/urdf/turtlebot3_burger.urdf'
                ).read()
            }]
        ),

        # joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner.py',
            name='jsb_spawner',
            arguments=['joint_state_broadcaster']
        ),

        # diff_drive_controller 스폰
        Node(
            package='controller_manager',
            executable='spawner.py',
            name='dd_spawner',
            arguments=['basic_diff_drive']
        ),

        # 컨트롤러 매개변수 로드
        Node(
            package='controller_manager',
            executable='controller_manager',
            name='cm_basic',
            parameters=['config/basic_diff_controllers.yaml'],
            output='screen'
        ),
    ])
