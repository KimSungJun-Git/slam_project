import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('capstone_robot')
    
    return LaunchDescription([
        # 1. Cartographer 메인 노드 (QoS 설정 포함)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                # Stella 공식 파일의 핵심: 통신 안정성 확보
                'qos_overrides./scan.reliability': 'best_effort',
                'qos_overrides./scan.history': 'keep_last',
                'qos_overrides./scan.depth': 10
            }],
            arguments=[
                '-configuration_directory', os.path.join(pkg_path, 'config'),
                '-configuration_basename', 'cartographer.lua'
            ]
        ),

        # 2. 지도 변환 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])
    