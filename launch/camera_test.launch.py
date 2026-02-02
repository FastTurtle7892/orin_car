import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # [전방 카메라]
    front_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',   # 마샬러용
            'framerate': 15.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/front_camera/image_raw')]
    )

    # [후방 카메라]
    rear_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        output='screen',
        parameters=[{
            'video_device': '/dev/video2',   # 도킹용 (번호 확인 필요!)
            'framerate': 15.0,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    return LaunchDescription([
        #front_cam,
        rear_cam
    ])
