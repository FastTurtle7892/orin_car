import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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
    map_file = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml') 
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')

    # ================= [2. 로봇 모델(URDF) 처리] =================
    try:
        doc = xacro.process_file(xacro_file)
        robot_desc = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"❌ Xacro 처리 실패: {e}")
        return LaunchDescription([])

    sim_time_config = {'use_sim_time': False}

    # ================= [3. 공통 하드웨어 & 센서 (즉시 실행)] =================
    
    # 3-1. 로봇 상태 퍼블리셔 (TF)
    node_robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', 
        parameters=[robot_desc, sim_time_config]
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
    # [수정] trailer_nav_launch.py 처럼 name 파라미터 명시 및 즉시 실행으로 변경
    node_rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry', # [중요] 이름 명시
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan', 
            'odom_topic': '/odom',
            'publish_tf': True, 
            'base_frame_id': 'base_link', 
            'odom_frame_id': 'odom', 
            'init_pose_from_topic': '', 
            'freq': 10.0
        }]
    )

    # 3-5. 모터 드라이버 (Ackermann)
    node_driver = Node(
        package=pkg_name, executable='ackermann_driver.py',
        name='ackermann_driver', # [중요] 이름 명시
        output='screen'
    )
    # [추가] 3-6. MQTT 통신 노드 (가볍기 때문에 즉시 실행)
    node_mqtt = Node(
        package=pkg_name, executable='mqtt_final.py',
        name='mqtt_final',
        output='screen'
    )

    # ================= [4. 자율주행 스택 (즉시 실행)] =================

    # 4-1. Nav2 스택 (지도, 경로계획)
    # [수정] trailer_nav_launch.py와 동일하게 즉시 실행 리스트에 포함
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

    # ================= [5. 응용 프로그램 (지연 실행)] =================
    # 비전 및 컨트롤러는 시스템 부하 분산을 위해 천천히 켭니다.

    # 5-1. 비전 매니저
    node_vision = Node(
        package=pkg_name, executable='vision_final.py',
        output='screen'
    )

    # 5-2. 기능별 컨트롤러들
    node_driving = Node(
        package=pkg_name, executable='driving_controller_final.py',
        output='screen'
    )
    node_docking = Node(
        package=pkg_name, executable='docking_controller_final.py',
        output='screen'
    )
    node_marshaller = Node(
        package=pkg_name, executable='marshaller_controller_final.py',
        output='screen'
    )

    return LaunchDescription([
        # 1. [Base Platform] 즉시 실행 그룹
        node_robot_state,
        node_joint_state,
        node_driver,
        node_ydlidar,
        node_rf2o,
        nav2_launch,
        node_mqtt,      # <--- [중요] 여기에 추가 (TimerAction 아님)
        
        # 2. [Applications] 순차 실행 그룹
        # T+15s: 비전 시스템
        TimerAction(period=25.0, actions=[node_vision]),
        
        # T+20s: 제어 로직
        TimerAction(period=30.0, actions=[
            node_driving, 
            node_docking, 
            node_marshaller
        ])
    ])
