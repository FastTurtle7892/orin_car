import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    
    # 1. 하드웨어 드라이버 (모터 및 서보 제어)
    ackermann_driver = Node(
        package=pkg_name,
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )

    # 2. 전면 카메라 실행 (USB 캠)
    front_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        parameters=[{
            'video_device': '/dev/video0', # 카메라 번호 확인 필수!
            'framerate': 15.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/front_camera/image_raw')]
    )

    # 3. 마샬러(제스처) 컨트롤러 실행
    # 카메라가 켜지고 안정화될 때까지 2초 기다렸다가 실행
    marshaller_node = Node(
        package=pkg_name,
        executable='marshaller_controller.py',
        name='marshaller_controller',
        output='screen',
        parameters=[{
            'speed_fast': 0.25,
            'speed_slow': 0.12,
            'turn_angle': 0.5
        }]
    )

    return LaunchDescription([
        ackermann_driver,
        front_cam_node,
        TimerAction(period=2.0, actions=[marshaller_node])
    ])
