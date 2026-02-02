import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)
    
    # [1] 하드웨어 및 TF (센서, 모터)
    # ---------------------------------------------------------
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch', 'ydlidar_launch.py')
        )
    )

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
            'freq': 10.0
        }]
    )
    
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    motor_driver = Node(
        package=pkg_name,
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )

    # [2] Nav2 실행
    # ---------------------------------------------------------
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml') 
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': os.path.join(pkg_share, 'params', 'nav2_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )

    # [3] 통합 지휘관 노드 (MQTT) - 이것만 실행합니다!
    # ---------------------------------------------------------
    # ★ 주의: 여기에 docking_controller가 있으면 안 됩니다! (그래야 IDLE 유지)
    mqtt_node = Node(
        package=pkg_name,
        executable='mqtt_dock_nav.py',
        name='mqtt_dock_nav',
        output='screen'
    )

    return LaunchDescription([
        static_tf,
        lidar_launch,
        rf2o_node,
        motor_driver,
        
        # 순차 실행
        TimerAction(period=3.0, actions=[nav2_launch]),
        TimerAction(period=5.0, actions=[mqtt_node]),
    ])