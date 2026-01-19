import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # [수정] 패키지 이름을 'orin_car'로 변경
    pkg_share = get_package_share_directory('orin_car')
    
    # 1. YDLidar 실행
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ydlidar_ros2_driver'), '/launch/ydlidar_launch.py'
        ])
    )

    # 2. RF2O Laser Odometry
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 3. SLAM (설정 파일 경로 주의)
    slam_config_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('slam_toolbox'), '/launch/online_async_launch.py'
        ]),
        # 만약 파라미터 파일이 없다면 아래 줄 주석 처리하거나 빈 파일이라도 만들어야 함
        # launch_arguments={'slam_params_file': slam_config_path}.items()
    )

    # 4. 로봇 모델 (URDF)
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'picar.urdf')
    
    robot_desc = ""
    if os.path.exists(urdf_file_path):
        with open(urdf_file_path, 'r') as infp:
            robot_desc = infp.read()
    else:
        print(f"Warning: URDF not found at {urdf_file_path}")

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4-1. Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        ydlidar_launch,
        rf2o_node,
        slam_launch,
        robot_state_publisher,
        joint_state_publisher,
    ])
