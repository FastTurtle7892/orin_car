import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 패키지 경로 설정
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)

    # 2. 파일 경로 설정
    # (1) URDF 모델
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    # (2) 라이다 설정 (X4-Pro)
    lidar_config_file = os.path.join(pkg_share, 'configs', 'X4-Pro.yaml')
    # (3) SLAM 설정 (slam_params.yaml) - 추가됨!
    slam_config_file = os.path.join(pkg_share, 'configs', 'slam_params.yaml')

    # 3. URDF 파싱
    doc = xacro.process_file(xacro_file)
    robot_description_config = {'robot_description': doc.toxml()}

    # 4. 노드 정의

    # [Node A] Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config]
    )

    # [Node B] Static TF (가짜 오도메트리)
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # [Node C] YDLidar Driver
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    # [Node D] SLAM Toolbox (설정 파일 적용)
    # slam_toolbox 패키지의 런치 파일을 가져오되, 우리의 slam_params.yaml을 전달합니다.
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_config_file  # 핵심: 우리의 설정 파일을 여기에 넣어줍니다.
        }.items()
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_static_tf,
        node_ydlidar,
        slam_toolbox_launch
    ])
