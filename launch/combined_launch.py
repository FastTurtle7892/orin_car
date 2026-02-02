import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 실제 파일 경로에 맞춰 수정하세요
    combined_node = Node(
        package='orin_car', # 본인의 패키지명
        executable='vision_mqtt_combined.py',
        name='vision_mqtt_combined',
        output='screen'
    )

    return LaunchDescription([
        combined_node
    ])
