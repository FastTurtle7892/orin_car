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

    # 1. 경로 설정
    # (주의) map_file_path는 젯슨 나노의 홈 디렉토리(~/maps2)를 바라보게 설정했습니다.
    default_map_path = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')

    # 2. Launch Arguments (맵 파일과 파라미터 파일을 실행 시 변경 가능하도록 설정)
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_path,
        description='Full path to param file to load'
    )

    # 3. 하드웨어 및 상태 퍼블리셔 (slam_launch.py와 동일하게 유지)
    
    # [Node 1] Robot State Publisher (URDF 파싱)
    doc = xacro.process_file(xacro_file)
    robot_description_config = {'robot_description': doc.toxml()}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config]
    )

    # [Node 2] RF2O Laser Odometry
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

    # [Node 3] YDLidar Driver
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    # [Node 4] Ackermann Driver
    node_ackermann_driver = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        output='screen'
    )

    # 4. Nav2 Bringup (SLAM 대신 실행되는 부분)
    # 맵 서버, AMCL(위치추정), 내비게이션 스택을 모두 실행합니다.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false', # 실제 로봇이므로 false
            'autostart': 'true'      # 실행 즉시 활성화
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        node_robot_state_publisher,
        node_rf2o,
        node_ydlidar,
        node_ackermann_driver,
        nav2_launch
    ])
