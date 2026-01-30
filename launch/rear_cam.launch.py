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
                'video_device': '/dev/video2',  # 장치 번호 (/dev/video*) 확인 필수
                'framerate': 15.0,              # [수정] 30 -> 15 (데이터량 절반 감소)
                'image_width': 320,             # [수정] 640 -> 320 (데이터량 1/4 감소)
                'image_height': 240,            # [수정] 480 -> 240 (데이터량 1/4 감소)
                'pixel_format': 'yuyv',
                'brightness': 50,
            }],
        )
    ])
