import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    pkg_name = 'orin_car'

    motor_driver = Node(
        package=pkg_name,
        executable='ackermann_driver_test.py', # [변경됨] 테스트용 파일
        name='ackermann_driver',
        output='screen'
    )

    node_mqtt = Node(
        package=pkg_name, executable='mqtt_total_control_test.py', # [변경됨] 테스트용 파일
        name='mqtt_final',
        output='screen'
    )

    docking_ctrl = Node(package=pkg_name, executable='docking_controller_test_v2.py', output='screen')         # 회전하는거 테스트중

    return LaunchDescription([
        
        motor_driver,
        node_mqtt,
        docking_ctrl,

    ])