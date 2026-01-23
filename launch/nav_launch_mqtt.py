import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)
    nav2_share = get_package_share_directory('nav2_bringup')

    # 1. 맵 및 파라미터 파일 경로 설정
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # 2. Launch Arguments (실행 시 변경 가능하도록 설정)
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Full path to param file to load'
    )

    # 3. 로봇 하드웨어 설정 (URDF, Lidar, Odom, Motor)
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    lidar_config = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    
    doc = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': doc.toxml()}

    # (A) Robot State Publisher (로봇 모델 발행)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc]
    )
    
    # (B) Joint State Publisher (바퀴 회전 등 TF 발행)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    # (C) RF2O (라이다 오도메트리 - 위치 추정)
    node_rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # (D) YDLidar 드라이버
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config],
        namespace='/'
    )

    # (E) 모터 드라이버 (차량 제어)
    node_ackermann = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        output='screen'
    )

    # 4. Nav2 Bringup (네비게이션 핵심 기능 실행)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # ================= [새로 추가된 부분] =================
    # CMakeLists.txt에 등록된 파이썬 스크립트들을 실행합니다.
    
    # (1) MQTT Bridge: 로봇 위치(x,y)를 서버로 전송
    node_mqtt_bridge = Node(
        package='orin_car',
        executable='mqtt_bridge.py',
        output='screen'
    )

    # (2) State Manager: 로봇 상태(IDLE, MOVING) 관리
    node_state_manager = Node(
        package='orin_car',
        executable='state_manager.py',
        output='screen'
    )

    # (3) Mission Client: 서버 명령(JSON) 받아서 Nav2 이동 명령 내림
    node_mission_client = Node(
        package='orin_car',
        executable='mission_client.py',
        output='screen'
    )
    # ====================================================

    return LaunchDescription([
        map_arg,
        params_arg,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rf2o,
        node_ydlidar,
        node_ackermann,
        nav2_launch,
        # 추가된 노드들도 여기서 실행 리스트에 포함
        node_mqtt_bridge,
        node_state_manager,
        node_mission_client
    ])
