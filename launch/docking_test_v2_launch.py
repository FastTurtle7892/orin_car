import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 하드웨어 드라이버 (Ackermann Driver)
    ackermann_node = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )

    # 2. 뒷면 카메라 (Rear Camera)
    rear_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2', # 포트 번호 확인!
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    # 3. [NEW] 도킹 컨트롤러 V2 (Yaw 보정 기능 추가됨)
    docking_controller_node = Node(
        package='orin_car',
        executable='docking_controller_v2.py', # 파일명 주의
        name='docking_controller_v2',
        output='screen'
    )

    return LaunchDescription([
        ackermann_node,
        rear_cam_node,
        docking_controller_node
    ])
