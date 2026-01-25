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

    # 파일 경로 설정
    default_map_path = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')

    # Launch Arguments
    map_arg = DeclareLaunchArgument('map', default_value=default_map_path)
    params_arg = DeclareLaunchArgument('params_file', default_value=nav2_params_path)

    # 1. Robot State Publisher
    doc = xacro.process_file(xacro_file)
    robot_description_config = {'robot_description': doc.toxml()}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config, {
            'use_sim_time': False,
            'publish_frequency': 15.0
        }]
    )

    # 2. [추가] Scan Locker Node (정지 시 스캔 고정 -> 오도메트리 떨림 방지)
    # 반드시 scripts/scan_locker.py 가 존재하고 실행 권한(chmod +x)이 있어야 합니다.
    node_scan_locker = Node(
        package='orin_car',
        executable='scan_locker.py',
        output='screen'
    )

    # 3. [수정] RF2O Laser Odometry (입력 토픽 변경: /scan -> /scan_for_odom)
    node_rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan_for_odom', # scan_locker가 주는 안정된 데이터 사용
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 4. YDLidar Driver
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    # 5. Ackermann Driver (Hardware Control)
    node_ackermann_driver = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        output='screen'
    )

    # 6. [추가] MQTT Nav Bridge (통신 및 미션 수행)
#    node_mqtt_bridge = Node(
#       package='orin_car',
#       executable='mqtt_nav_bridge.py',
#       output='screen'
#    )

    # 7. Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        node_robot_state_publisher,
        node_scan_locker,      # 추가됨
        node_rf2o,
        node_ydlidar,
        node_ackermann_driver,
#       node_mqtt_bridge,      # 추가됨
        nav2_launch
    ])
