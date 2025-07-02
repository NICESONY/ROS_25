from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        # 1) 전체 네비게이션 + 컨트롤러
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/full_navigation.launch.py']
            )
        ),

        # 2) mission_manager 노드
        Node(
            package='turtlebot3_controller',
            executable='mission_manager.py',
            name='mission_manager',
            output='screen',
            parameters=[{'start_pose': [0.0, 0.0, 0.0]}]
        ),
    ])
