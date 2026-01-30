import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='rear_camera',
            namespace='rear_camera',
            parameters=[{
                'video_device': '/dev/video2',
                'framerate': 30.0,              # 15.0 -> 30.0 (부드럽게)
                'image_width': 320,             # 320 -> 640 (AI 설정과 일치시킴)
                'image_height': 230,            # 240 -> 480
                'pixel_format': 'mjpeg2rgb',    # yuyv -> mjpeg2rgb (색감/호환성 개선)
                'brightness': 50,
            }],
            # 토픽 이름이 안 맞을 수 있으니 리매핑 추가
            remappings=[('/image_raw', '/rear_camera/image_raw')]
        )
    ])
