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
    # [설정] final_launch.py와 동일하게 base_link 기준
    node_rf2o = Node(
        package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        arguments=['--ros-args', '--log-level', 'ERROR'],
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
    
    # 3-5. 아커만 모터 드라이버 (Test 버전)
    motor_driver = Node(
        package=pkg_name,
        executable='ackermann_driver_test.py', # [변경됨] 테스트용 파일
        name='ackermann_driver',
        output='screen'
    )

    # 3-6. MQTT 통신 노드 (Test 버전)
    node_mqtt = Node(
        package=pkg_name, executable='mqtt_total_control_test.py', # [변경됨] 테스트용 파일
        name='mqtt_final',
        output='screen'
    )

    # ================= [4. 자율주행 스택 (Nav2)] =================
    # [중요] final_launch.py처럼 즉시 실행 설정
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

    # ================= [5. 통합 테스트 컨트롤러 (지연 실행)] =================
    # 이것들은 Nav2가 켜진 뒤에 켜져야 하므로 10초 뒤 실행
    docking_ctrl = Node(package=pkg_name, executable='docking_controller_test.py', output='screen')
    marshaller_ctrl = Node(package=pkg_name, executable='marshaller_controller_test.py', output='screen')
    driving_ctrl = Node(package=pkg_name, executable='driving_controller_test.py', output='screen')

    return LaunchDescription([
        # [그룹 1: 즉시 실행] - final_launch.py와 똑같은 순서
        
        node_robot_state,
        node_joint_state,
        motor_driver,
        node_ydlidar,
        node_rf2o,
        nav2_launch, 
        node_mqtt,      # MQTT도 통신 대기를 위해 바로 실행
        
        # [그룹 2: 지연 실행] - 컨트롤러들
        TimerAction(period=30.0, actions=[
            
            docking_ctrl,
            #marshaller_ctrl,
            #driving_ctrl
        ]),
    ])