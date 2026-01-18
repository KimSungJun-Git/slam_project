import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    pkg_path = get_package_share_directory('capstone_robot')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map' : os.path.join(pkg_path, 'maps', 'capstone_map.yaml'),
                'params_file' : os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
                'use_sim_time' : 'False'
            }.items()
        ),
    ])