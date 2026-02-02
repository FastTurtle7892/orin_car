import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. [도킹 컨트롤러] (카메라 열고, 판단하고, 명령 내림)
    docking_node = Node(
        package='orin_car',
        executable='docking_controller.py', 
        name='docking_controller',
        output='screen'
    )

    # 2. [모터 드라이버] (명령 받아서 바퀴 굴림)
    motor_driver = Node(
        package='orin_car',
        executable='ackermann_driver.py', 
        name='ackermann_driver',
        output='screen'
    )

    return LaunchDescription([
        docking_node,
        motor_driver
    ])