# launch/test_map_launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('turtlebot3_controller')
    world_file = os.path.join(pkg_share, 'worlds', 'my_test_map.world')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    return LaunchDescription([gazebo_launch])
