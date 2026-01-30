import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='front_camera',
            namespace='front_camera',
            parameters=[{
                'video_device': '/dev/video0', # 장치 번호 확인!
                'framerate': 15.0,
                'image_width': 320,
                'image_height': 240,
                'pixel_format': 'mjpeg2rgb',
                'brightness': 50,
            }],
            remappings=[('/image_raw', '/front_camera/image_raw')]
        )
    ])
