import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'orin_car'
    nav2_pkg = 'nav2_bringup'
    pkg_share = get_package_share_directory(pkg_name)

    # ==========================================
    # 1. 경로 및 설정 파일 지정
    # ==========================================
    # [지도 경로] 홈 디렉토리 기준 절대 경로 (사용자 설정 유지)
    map_file = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    
    # [Config 파일]
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    
    # [URDF 경로]
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')

    # ==========================================
    # 2. 로봇 모델 처리 (Xacro -> URDF)
    # ==========================================
    try:
        doc = xacro.process_file(xacro_file)
        robot_description = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"❌ Xacro 처리 실패: {e}")
        return LaunchDescription([])

    # ==========================================
    # 3. 하드웨어 및 기본 노드 (Always On)
    # ==========================================
    
    # [모터 드라이버]
    ackermann_node = Node(
        package=pkg_name,
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )

    # [Robot State Publisher]
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # [LiDAR 드라이버]
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    # [Odometry (rf2o)]
    rf2o_node = Node(
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
            'freq': 10.0
        }],
    )

    # [전방 카메라 - 마샬러용] (/dev/video0)
    front_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='front_camera',
        namespace='front_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 15.0,        # 부하를 줄이기 위해 15fps 설정
            'image_width': 320,       # 해상도 축소 (AI 처리 속도 향상)
            'image_height': 240,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/front_camera/image_raw')]
    )

    # [후방 카메라 - 도킹용] (/dev/video2)
    rear_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='rear_camera',
        namespace='rear_camera',
        parameters=[{
            'video_device': '/dev/video2',
            'framerate': 15.0,
            'image_width': 320,
            'image_height': 240,
            'pixel_format': 'mjpeg2rgb',
            'brightness': 50,
        }],
        remappings=[('/image_raw', '/rear_camera/image_raw')]
    )

    # [Nav2 (Navigation Stack)]
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_pkg), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    # ==========================================
    # 4. 기능별 컨트롤러 노드 (Mode 대기 상태)
    # ==========================================
    
    # [1번 기능: 도킹]
    docking_node = Node(
        package=pkg_name,
        executable='docking_controller.py',
        name='docking_controller',
        output='screen'
    )

    # [2번 기능: 주행 (MQTT 경로 추종)]
    # 로컬 테스트라면 mqtt_path_follower.py 내부 로직 확인 필요
    path_follower_node = Node(
        package=pkg_name,
        executable='mqtt_path_follower.py',
        name='path_follower',
        output='screen'
    )

    # [3번 기능: 마샬러]
    marshaller_node = Node(
        package=pkg_name,
        executable='marshaller_controller.py',
        name='marshaller_controller',
        output='screen',
        parameters=[{
            'speed_fast': 0.25,
            'speed_slow': 0.12,
            'turn_angle': 0.5
        }]
    )

    # ==========================================
    # 5. 실행 목록 반환 (순차 실행)
    # ==========================================
    return LaunchDescription([
        ackermann_node,      # 1. 모터
        ydlidar_node,        # 2. 라이다
        rf2o_node,           # 3. 오도메트리
        rsp_node,            # 4. TF
        front_cam_node,      # 5. 전방 캠
        rear_cam_node,       # 6. 후방 캠
        nav2_launch,         # 7. 내비게이션
        
        # 컨트롤러들은 시스템이 안정화된 후 5초 뒤에 실행
        TimerAction(
            period=5.0, 
            actions=[docking_node, path_follower_node, marshaller_node]
        )
    ])