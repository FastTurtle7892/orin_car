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
    
    # 3-5. 아커만 모터 드라이버
    motor_driver = Node(
        package=pkg_name,
        executable='ackermann_driver_test.py',
        name='ackermann_driver',
        output='screen'
    )
    # [추가] 3-6. MQTT 통신 노드 (가볍기 때문에 즉시 실행)
    node_mqtt = Node(
        package=pkg_name, executable='mqtt_total_control_test.py',
        name='mqtt_final',
        output='screen'
    )

    # [4. 자율주행 스택 (Nav2)]
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params,
            'use_sim_time': 'false',
            'autostart': 'true' # 자동으로 노드 활성화
        }.items()
    )

    # [5. 통합 테스트 컨트롤러 (test_scripts)]
    docking_ctrl = Node(package=pkg_name, executable='docking_controller_test.py', output='screen')
    marshaller_ctrl = Node(package=pkg_name, executable='marshaller_controller_test.py', output='screen')
    driving_ctrl = Node(package=pkg_name, executable='driving_controller_test.py', output='screen')
    mqtt_ctrl = Node(package=pkg_name, executable='mqtt_total_control_test.py', output='screen')

    return LaunchDescription([
        # 즉시 실행: 로봇 모델 및 하드웨어 베이스
        node_robot_state,
        node_joint_state,
        motor_driver,
        node_ydlidar,
        node_rf2o,
        node_mqtt,
        
        # 순차 실행: 하드웨어 안정을 위해 Nav2와 컨트롤러는 지연 실행
        # Nav2 실행 (라이다 안정화 대기)
        TimerAction(period=5.0, actions=[nav2_launch]),
        
        # 컨트롤러 및 MQTT 실행 (Nav2 기동 대기)
        TimerAction(period=10.0, actions=[
            docking_ctrl,
            marshaller_ctrl,
            driving_ctrl
        ]),
    ])