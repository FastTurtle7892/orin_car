import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 하드웨어 드라이버 (모터 & 서보 제어)
    ackermann_node = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )

    # 2. 뒷면 카메라 (Rear Camera)
    # 실제 연결된 포트 번호(/dev/videoX) 확인 필수!
    rear_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2', # 보통 0:앞, 2:뒤 (확인 필요)
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    # 3. 도킹 컨트롤러 (방금 만든 것)
    docking_controller_node = Node(
        package='orin_car',
        executable='docking_controller.py',
        name='docking_controller',
        output='screen'
    )

    return LaunchDescription([
        ackermann_node,
        rear_cam_node,
        docking_controller_node
    ])
