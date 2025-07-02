from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

def generate_launch_description():
    map_path = LaunchConfiguration('map_file',
                  default=str(ThisLaunchFileDir()) + '/../maps/home2_map.yaml')
    use_sim = LaunchConfiguration('use_sim_time', default='True')

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=map_path),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim),

        # 1) map_server + odom→map static transform
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/map_server_launch.py']
            ),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim
            }.items()
        ),

        # 2) Nav2 bringup (AMCL, planner 등 설정: config/nav2_params.yaml)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/nav2_bringup_launch.py']
            ),
            launch_arguments={'use_sim_time': use_sim}.items()
        ),

        # 3) basic bringup (URDF + 컨트롤러)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/basic_bringup.launch.py']
            )
        ),
    ])
