import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)

    # =========================================================
    # 1. 기본 주행 시스템 (Lidar, Nav2, Driver, TF 등)
    # =========================================================
    # [핵심] 기존에 잘 되던 설정을 그대로 가져옵니다. 
    # 라이다 오류는 이걸로 해결됩니다.
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'trailer_nav_launch.py')
        )
    )

    # =========================================================
    # 2. 카메라 시스템 (Front: video0 / Rear: video2)
    # =========================================================
    front_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        parameters=[{
            'video_device': '/dev/video0',   # 전면
            'framerate': 30.0,
            'image_width': 320,  # 성능 최적화 (320x240)
            'image_height': 240,
            'pixel_format': 'mjpeg'
        }],
        remappings=[('/image_raw', '/front_camera/image_raw')]
    )

    rear_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2',   # 후면
            'framerate': 30.0,
            'image_width': 320,
            'image_height': 240,
            'pixel_format': 'mjpeg'
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    web_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    # =========================================================
    # 3. 추가된 미션 & 제어 노드
    # =========================================================
    
    # (1) 경로 추종 (MQTT Path Follower) - [새로 추가된 기능]
    path_follower = Node(
        package='orin_car',
        executable='mqtt_path_follower.py',
        name='mqtt_path_follower',
        output='screen'
    )

    # (2) 미션 마스터 (도킹/마샬링 총괄)
    # 시스템이 안정화된 후(5초 뒤) 실행
    mission_master = TimerAction(
        period=5.0, 
        actions=[
            Node(
                package='orin_car',
                executable='mission_master.py',
                name='mission_master',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        base_system,      # 기존 주행 시스템 (라이다/오도메트리 포함)
        front_cam,        # 전면 카메라
        rear_cam,         # 후면 카메라
        web_server,       # 웹 비디오 서버
        path_follower,    # [NEW] 경로 추종
        mission_master    # [NEW] 미션 마스터
    ])
