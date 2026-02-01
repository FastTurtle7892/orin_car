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
    # [수정] trailer_nav_launch.py와 동일하게 경로 설정
    lidar_config = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_file = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml') 
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')

    # ================= [2. 로봇 모델(URDF) 처리] =================
    # [수정] trailer_nav_launch.py 처럼 예외 처리 추가하여 안전성 확보
    try:
        doc = xacro.process_file(xacro_file)
        robot_desc = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"❌ Xacro 처리 실패: {e}")
        return LaunchDescription([])

    # [수정] trailer_nav_launch.py와 동일하게 변수로 분리 (실수 방지)
    sim_time_config = {'use_sim_time': False}

    # ================= [3. 공통 하드웨어 & 센서] =================
    
    # 3-1. 로봇 상태 퍼블리셔 (TF)
    node_robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', 
        parameters=[robot_desc, sim_time_config] # [수정] 변수 적용
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
    # [수정] trailer_nav_launch.py의 파라미터 그대로 적용 (init_pose_from_topic 추가됨!)
    node_rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan', 
            'odom_topic': '/odom',
            'publish_tf': True, 
            'base_frame_id': 'base_link', 
            'odom_frame_id': 'odom', 
            'init_pose_from_topic': '', # [중요] 기존 코드에 빠져있던 부분 추가
            'freq': 10.0
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
            'use_sim_time': 'false', # [확인] 소문자 문자열 'false' 유지
            'autostart': 'true'
        }.items()
    )

    # 4-2. 비전 매니저 (카메라 스위칭 담당)
    node_vision = Node(
        package=pkg_name, executable='vision_final.py',
        output='screen'
    )

    # 4-3. MQTT 브릿지 (명령 수신)
    # [주석] 사용자 요청대로 여기서 실행 안 하고 따로 터미널에서 실행
    # node_bridge = Node(
    #     package=pkg_name, executable='mqtt_final.py',
    #     output='screen'
    # )

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
        # 1. 즉시 실행: 로봇 상태, 드라이버, 라이다 (기초 하드웨어)
        node_robot_state,
        node_joint_state,
        node_driver,
        node_ydlidar,
        
        # 2. 5초 후: 오도메트리 & 웹서버 (라이다 안정화 대기)
        # (기존 3초 -> 5초로 변경)
        TimerAction(period=1.0, actions=[node_rf2o]),

        # 3. 15초 후: Nav2 (가장 무거운 프로세스)
        # (기존 5초 -> 15초로 대폭 변경! 앞에서 켠 노드들이 안정될 시간을 줌)
        TimerAction(period=3.0, actions=[nav2_launch]),

        # 4. 25초 후: 카메라 & 비전
        # (Nav2가 켜지고 CPU가 진정될 때까지 10초 더 기다림)
        TimerAction(period=5.0, actions=[node_vision]),

        # 5. 30초 후: 컨트롤러들
        TimerAction(period=7.0, actions=[
            # node_bridge,
            node_driving, 
            node_docking, 
            node_marshaller
        ])
    ])

