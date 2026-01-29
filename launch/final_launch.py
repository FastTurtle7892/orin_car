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
    # 1. 기본 주행 시스템 (제일 먼저 켜짐)
    # =========================================================
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'trailer_nav_launch.py')
        )
    )

    # =========================================================
    # 2. 카메라 시스템 (설정)
    # =========================================================
    # [최적화] 30fps -> 15fps (두 개 동시에 안정적으로 켜기 위함)
    
    front_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 15.0,        # 15로 낮춤 (안전빵)
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/front_camera/image_raw')]
    )

    rear_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2',
            'framerate': 15.0,        # 15로 낮춤
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    # =========================================================
    # [핵심 해결책] 시간차 실행 (TimerAction)
    # =========================================================
    
    # 1. 전면 카메라는 시작 후 2초 뒤에 켭니다.
    delayed_front_cam = TimerAction(
        period=2.0,
        actions=[front_cam_node]
    )

    # 2. 후면 카메라는 시작 후 4초 뒤에 켭니다. (전면 카메라 켜지고 2초 뒤)
    delayed_rear_cam = TimerAction(
        period=4.0,
        actions=[rear_cam_node]
    )

    web_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    # =========================================================
    # 3. 미션 & 제어 노드
    # =========================================================
    path_follower = Node(
        package='orin_car',
        executable='mqtt_path_follower.py',
        name='mqtt_path_follower',
        output='screen'
    )

    # 미션 마스터는 모든 센서가 안정화된 8초 뒤에 실행
    mission_master = TimerAction(
        period=8.0, 
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
        base_system,        # 0초: 라이다/Nav2 시작
        web_server,         # 0초: 웹 서버 시작
        path_follower,      # 0초: 경로 추종 시작
        delayed_front_cam,  # 2초: 전면 카메라 시작
        delayed_rear_cam,   # 4초: 후면 카메라 시작
        mission_master      # 8초: 미션 마스터 시작
    ])
