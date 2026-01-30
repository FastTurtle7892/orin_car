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
                'video_device': '/dev/video2', # 장치 번호 확인!
                'framerate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg2rgb',
                'brightness': 50,
            }],
        )
    ])
