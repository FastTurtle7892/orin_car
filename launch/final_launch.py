import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 기본 주행 시스템 (Lidar, Nav2, Driver)
    # 기존 trailer_nav_launch.py 실행
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'trailer_nav_launch.py')
        )
    )

    # 2. 전방 카메라 (마샬러 & 기장 UI 송출용)
    # MJPEG 포맷, 320x240 해상도 (성능 최적화)
    front_cam = Node(
        package='usb_cam', 
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        parameters=[{
            'video_device': '/dev/video0', # 포트 확인 필요 (ls /dev/video*)
            'image_width': 320,
            'image_height': 240,
            'pixel_format': 'mjpeg',
            'framerate': 30.0
        }]
    )

    # 3. 후방 카메라 (도킹용)
    rear_cam = Node(
        package='usb_cam', 
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2', # 포트 확인 필요
            'image_width': 320,
            'image_height': 240,
            'pixel_format': 'mjpeg',
            'framerate': 30.0
        }]
    )

    # 4. 웹 비디오 서버 (기장 UI 영상 송출용)
    # 접속 주소: http://<로봇IP>:8080/stream?topic=/front_camera/image_raw&type=mjpeg
    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
        parameters=[{'port': 8080}]
    )

    # 5. 미션 마스터 (통합 제어)
    # 다른 노드들이 켜질 시간을 주기 위해 5초 뒤 실행
    mission_master = TimerAction(
        period=5.0,
        actions=[
            Node(
                package=pkg_name,
                executable='mission_master.py',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        base_system,
        front_cam,
        rear_cam,
        web_video_server,
        mission_master
    ])
