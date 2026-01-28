import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'orin_car'
    nav2_pkg = 'nav2_bringup'
    
    # 패키지 공유 디렉토리 (install 폴더 기준)
    pkg_share = get_package_share_directory(pkg_name)

    # ==========================================
    # 1. 경로 설정 (Path Fix)
    # ==========================================
    
    # [지도 경로] 홈 디렉토리 기준 절대 경로
    map_file = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    
    # [설정 파일 경로]
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    
    # [URDF 경로]
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')

    # ==========================================
    # 2. 로봇 모델 처리 (Xacro -> URDF)
    # ==========================================
    try:
        doc = xacro.process_file(xacro_file)
        robot_description_config = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"❌ Xacro 처리 실패: {e}")
        return LaunchDescription([])

    # ==========================================
    # 3. 노드 정의
    # ==========================================

    # [수정] use_sim_time을 소문자 'false'로 명확하게 지정
    # 대문자 'False'는 문자열로 인식되어 True로 오해받을 수 있음 -> 로봇 안 보이는 원인 1순위
    sim_time_config = {'use_sim_time': False} 

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config, sim_time_config]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    node_rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    node_ackermann = Node(
        package=pkg_name,
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )

    node_mqtt_follower = Node(
        package=pkg_name,
        executable='mqtt_path_follower.py',
        name='mqtt_path_follower',
        output='screen'
    )

    # [Nav2 실행]
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_pkg), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false', # [중요] 소문자 문자열 'false' 사용
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        # 디버깅용 로그 출력
        LogInfo(msg=f"✅ Map File Path: {map_file}"),
        LogInfo(msg=f"✅ Nav2 Params Path: {nav2_params_file}"),
        LogInfo(msg=f"✅ URDF File Path: {xacro_file}"),
        
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_ydlidar,
        node_rf2o,
        node_ackermann,
        node_mqtt_follower,
        nav2_launch
    ])
