import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. 기본 주행 시스템 (LiDAR, Nav2, Ackermann Driver)
    base_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'trailer_nav_launch.py')
        )
    )

    # 2. 카메라 (충돌 방지를 위한 시간차 실행)
    # [전면] Marshalling (video0)
    front_cam_node = Node(
        package='usb_cam', executable='usb_cam_node_exe',
        name='front_camera', namespace='front_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 15.0,
            'image_width': 640, 'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/front_camera/image_raw')]
    )
    
    # [후면] Docking (video2)
    rear_cam_node = Node(
        package='usb_cam', executable='usb_cam_node_exe',
        name='rear_camera', namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2',
            'framerate': 15.0,
            'image_width': 640, 'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    # 전면 3초 후, 후면 6초 후 실행
    delayed_front_cam = TimerAction(period=3.0, actions=[front_cam_node])
    delayed_rear_cam = TimerAction(period=6.0, actions=[rear_cam_node])

    # 3. 통합 컨트롤러들 실행 (10초 후 안정화되면 실행)
    master_controller = Node(
        package=pkg_name, executable='master_controller_ssafy.py', 
        name='master_controller', output='screen'
    )

    docking_controller = Node(
        package=pkg_name, executable='docking_controller_v2.py',
        name='docking_controller', output='screen'
    )

    marshaller_controller = Node(
        package=pkg_name, executable='marshaller_controller_v2.py',
        name='marshaller_controller', output='screen',
        parameters=[{'speed_fast': 0.25, 'speed_slow': 0.12, 'turn_angle': 0.5}]
    )

    delayed_controllers = TimerAction(
        period=10.0,
        actions=[master_controller, docking_controller, marshaller_controller]
    )

    # 웹 서버 (선택)
    web_server = Node(
        package='web_video_server', executable='web_video_server',
        name='web_video_server', output='screen',
        parameters=[{'port': 8080}]
    )

    return LaunchDescription([
        base_system,
        web_server,
        delayed_front_cam,
        delayed_rear_cam,
        delayed_controllers
    ])
