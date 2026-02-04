import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 현재 폴더 경로 기준 (파일들이 실행 위치에 있다고 가정)
    map_yaml_file = '~/maps5/my_map.yaml'
    params_file = '../config/nav2_params.yaml'
    
    # 1. 맵 서버 (지도 제공)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}, 
                    {'use_sim_time': False}]
    )

    # 2. 플래너 서버 (길 찾기)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # 3. 라이프사이클 매니저 (노드들 활성화)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_path',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'planner_server']}]
    )

    # 4. 경로 생성 파이썬 스크립트 실행 (서버 켜지고 3초 뒤 실행)
    path_generator_script = ExecuteProcess(
        cmd=['python3', 'generate_nav2_json.py'],
        output='screen'
    )
    
    # 5. 실행 순서: 서버들 켜고 -> 라이프사이클 켜고 -> 5초 뒤에 파이썬 코드 실행
    return LaunchDescription([
        map_server_node,
        planner_server_node,
        lifecycle_manager_node,
        TimerAction(period=5.0, actions=[path_generator_script])
    ])