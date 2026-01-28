import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'orin_car_mqtt_test' # 패키지 이름이 맞는지 확인하세요
    nav2_pkg = 'nav2_bringup'

    # 설정 파일 경로
    nav2_params_file = os.path.join(get_package_share_directory(pkg_name), 'config', 'nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory(pkg_name), 'maps', 'my_map.yaml') # 맵 파일 경로 확인

    return LaunchDescription([
        # 1. 아커만 드라이버 (하드웨어 제어)
        Node(
            package=pkg_name,
            executable='ackermann_driver.py',
            name='ackermann_driver',
            output='screen'
        ),

        # 2. [NEW] MQTT 경로 추종 노드 (기존 nav_bridge 대체)
        Node(
            package=pkg_name,
            executable='mqtt_path_follower.py',
            name='mqtt_path_follower',
            output='screen'
        ),

        # 3. Nav2 Bringup (Controller Server 실행용)
        # Planner가 켜지긴 하지만, 우리가 FollowPath를 직접 호출하므로 Planner는 무시됨
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(nav2_pkg), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params_file,
                'use_sim_time': 'False'
            }.items()
        )
    ])
