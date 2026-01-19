import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)
    nav2_share = get_package_share_directory('nav2_bringup')

    # 1. 경로 설정
    # (주의) map_name은 저장한 지도 이름에 맞게 수정해서 쓰세요!
    map_file_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    params_file_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # 2. Launch Argument 설정
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

    # 3. 하드웨어 드라이버 실행 (센서 + 모터)
    # SLAM 할 때 썼던 그 런치 파일을 재사용합니다. (단, SLAM 노드는 빼고 싶지만 구조상 같이 켜질 수 있음)
    # 정석: navigation_launch 에서는 slam_toolbox를 끄고 map_server를 켜야 합니다.
    # 따라서 여기서는 하드웨어 노드만 따로 실행하거나, 
    # 기존 slam_launch.py에서 slam_toolbox 부분만 주석 처리한 'robot_launch.py'를 만드는 게 좋습니다.
    # 일단은 편의를 위해 여기서 하드웨어 노드를 직접 정의합니다.

    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    lidar_config = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    
    # URDF
    import xacro
    doc = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc]
    )

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

    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config],
        namespace='/'
    )

    node_ackermann = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        output='screen'
    )

    # 4. Nav2 Bringup (여기가 핵심!)
    # nav2_bringup 패키지의 기본 런치를 실행하되, 우리 설정파일을 먹여줍니다.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true'  # 자동으로 시작
        }.items()
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        node_robot_state_publisher,
        node_rf2o,
        node_ydlidar,
        node_ackermann,
        nav2_launch
    ])
