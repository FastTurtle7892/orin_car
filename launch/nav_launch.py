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

    default_map_path = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')

    map_arg = DeclareLaunchArgument('map', default_value=default_map_path)
    params_arg = DeclareLaunchArgument('params_file', default_value=nav2_params_path)

    # 1. Robot State Publisher (설정 강화)
    doc = xacro.process_file(xacro_file)
    robot_description_config = {'robot_description': doc.toxml()}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # [핵심] use_sim_time을 False로 명시하고, TF 발행 빈도를 높임
        parameters=[robot_description_config, {
            'use_sim_time': False,
            'publish_frequency': 50.0 
        }]
    )

    # 2. RF2O Laser Odometry
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

    # 3. YDLidar Driver
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )

    # 4. Ackermann Driver
    node_ackermann_driver = Node(
        package='orin_car',
        executable='ackermann_driver.py',
        output='screen'
    )

    # 5. Nav2 Bringup
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
        node_rf2o,
        node_ydlidar,
        node_ackermann_driver,
        nav2_launch
    ])
