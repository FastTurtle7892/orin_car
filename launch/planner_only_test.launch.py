import os  # <--- 필수! 경로 계산을 위해 필요합니다.
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 파일 경로 설정
    map_file = '/home/ubuntu/maps5/my_map.yaml'
    params_file = '/home/ubuntu/Workspace/ros_ws/src/orin_car/config/nav2_params.yaml'
    
    # RViz 설정 파일 경로 정의 (~/.rviz2/robot.rviz)
    # os.path.expanduser('~')는 '/home/ubuntu'로 변환해줍니다.
    rviz_config_file = os.path.join(os.path.expanduser('~'), '.rviz2', 'robot.rviz')

    lifecycle_nodes = ['map_server', 'planner_server']

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # 가짜 TF (map -> base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}]
        ),

        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        ),
        
        # [수정됨] RViz2 (저장된 설정 파일 불러오기)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # <--- 이 부분이 핵심입니다!
        )
    ])