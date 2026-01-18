import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'custom_slam'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. URDF 파일 로드
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # 1. robot_state_publisher (이름 충돌 방지를 위해 name을 명시적으로 지정하거나 하나만 남겨야 함)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher', # 이름을 확실히 지정
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True # 시뮬레이션(Gazebo) 사용 시 필수!
            }]
        ),

        # 2. Cartographer 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': True}], # 시간 동기화 필수
            arguments=[
                '-configuration_directory', os.path.join(pkg_share, 'config'),
                '-configuration_basename', 'turtlebot3_lds_2d.lua'
            ],
            remappings=[('scan', '/scan')]
        ),

        # 3. 점유 격자 지도 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05']
        ),
    ])
