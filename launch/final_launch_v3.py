import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 기본 주행 시스템 (LiDAR, Nav2 등)
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'trailer_nav_launch.py')
        )
    )

    # 2. 마스터 컨트롤러 (동적 카메라 제어 기능 탑재)
    master_controller = Node(
        package=pkg_name,
        executable='master_controller_dynamic.py', 
        name='master_controller',
        output='screen'
    )

    # 3. 기능 컨트롤러들
    docking_controller = Node(
        package=pkg_name,
        executable='docking_controller_v2.py',
        name='docking_controller',
        output='screen'
    )

    marshaller_controller = Node(
        package=pkg_name,
        executable='marshaller_controller_v2.py',
        name='marshaller_controller',
        output='screen',
        parameters=[{'speed_fast': 0.25, 'speed_slow': 0.12, 'turn_angle': 0.5}]
    )

    # 4. 웹 서버 (선택)
    web_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    # 컨트롤러들은 시스템 안정화 후 5초 뒤 실행
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[master_controller, docking_controller, marshaller_controller]
    )

    return LaunchDescription([
        base_system,
        web_server,
        delayed_controllers
    ])
