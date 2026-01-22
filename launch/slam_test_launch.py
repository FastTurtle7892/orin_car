import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)

    # 경로 설정
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    slam_config_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # URDF 파싱
    doc = xacro.process_file(xacro_file)
    robot_description_config = {'robot_description': doc.toxml()}

    # [Node 1] Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config]
    )

    # [Node 1.5] Joint State Publisher (필수 추가!)
    # 이것이 없으면 바퀴 TF가 생성되지 않아 RobotModel 에러가 발생합니다.
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    # [Node 2] RF2O Laser Odometry
    # [수정] QoS 설정을 맞추기 위해 파라미터를 점검합니다. 
    # 만약 계속 안 되면 rf2o 소스 코드를 수정해야 할 수도 있지만, 
    # 우선 실행 순서와 TF가 확실하면 해결될 수 있습니다.
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
    # [중요] QoS 호환성을 위해 parameter를 강제로 Reliable로 맞출 수 있는지 시도합니다.
    # 하지만 드라이버 내부 설정이 우선일 수 있으므로, 실행 후 확인이 필요합니다.
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    # [Node 4] SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_config_file
        }.items()
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher, # 추가됨
        node_rf2o,
        node_ydlidar,
        slam_toolbox_launch
    ])
