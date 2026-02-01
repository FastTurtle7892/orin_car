import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)
    nav2_pkg = 'nav2_bringup'

    # ================= [1. 파일 경로 설정] =================
    lidar_config = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    # 지도 파일 경로는 실제 환경에 맞춰 수정 필요
    map_file = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml') 
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')

    # ================= [2. 로봇 모델(URDF) 처리] =================
    doc = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': doc.toxml()}

    # ================= [3. 공통 하드웨어 & 센서] =================
    
    # 3-1. 로봇 상태 퍼블리셔 (TF)
    node_robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_desc, {'use_sim_time': False}]
    )
    
    # 3-2. 관절 상태 퍼블리셔
    node_joint_state = Node(
        package='joint_state_publisher', executable='joint_state_publisher',
        output='screen', parameters=[{'use_gui': False}]
    )

    # 3-3. 라이다 센서
    node_ydlidar = Node(
        package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
        output='screen', emulate_tty=True,
        parameters=[lidar_config], namespace='/'
    )

    # 3-4. 오도메트리 (위치 추정)
    node_rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan', 'odom_topic': '/odom',
            'publish_tf': True, 'base_frame_id': 'base_link', 'odom_frame_id': 'odom', 'freq': 10.0
        }]
    )

    # 3-5. 모터 드라이버 (Ackermann)
    node_driver = Node(
        package=pkg_name, executable='ackermann_driver.py',
        output='screen'
    )

    # ================= [4. 자율주행 & 제어 시스템] =================

    # 4-1. Nav2 스택 (지도, 경로계획)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_pkg), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # 4-2. 비전 매니저 (카메라 스위칭 담당)
    node_vision = Node(
        package=pkg_name, executable='vision_final.py',
        output='screen'
    )

    # 4-3. MQTT 브릿지 (명령 수신)
    node_bridge = Node(
        package=pkg_name, executable='mqtt_final.py',
        output='screen'
    )

    # ================= [5. 기능별 컨트롤러] =================
    
    # 5-1. 주행 (Path Follower)
    node_driving = Node(
        package=pkg_name, executable='driving_controller_final.py',
        output='screen'
    )

    # 5-2. 도킹 (Docking)
    node_docking = Node(
        package=pkg_name, executable='docking_controller_final.py',
        output='screen'
    )

    # 5-3. 마샬러 (Marshaller)
    node_marshaller = Node(
        package=pkg_name, executable='marshaller_controller_final.py',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state,
        node_joint_state,
        node_ydlidar,
        node_rf2o,
        node_driver,
        nav2_launch,
        node_vision,
        node_bridge,
        node_driving,
        node_docking,
        node_marshaller
    ])