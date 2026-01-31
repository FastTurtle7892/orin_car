import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. [센서] 라이다 (항상 켜둠 - 주행/위치인식용)
    lidar_config_file = os.path.join(pkg_share, 'config', 'X4-Pro.yaml')
    node_ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[lidar_config_file],
        namespace='/'
    )
    
    # 2. [센서] 오도메트리 (라이다 기반 위치 추정)
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
            'freq': 10.0
        }],
    )

    # 3. [비전] Vision Manager (카메라 스위칭 & AI 통합)
    # USB 캠 충돌 방지를 위해 이 노드가 직접 카메라를 껐다 켭니다.
    node_vision_manager = Node(
        package=pkg_name,
        executable='vision_manager.py',
        name='vision_manager',
        output='screen'
    )

    # 4. [통신] MQTT Bridge (Postman 명령 수신)
    node_mqtt = Node(
        package=pkg_name,
        executable='mqtt_total_bridge.py',
        name='mqtt_bridge',
        output='screen'
    )

    # 5. [하드웨어] 드라이버 (모터가 없어도 에러 로그 확인용으로 실행)
    node_driver = Node(
        package=pkg_name,
        executable='ackermann_driver.py',
        name='ackermann_driver',
        output='screen'
    )
    
    # (참고) TF (Robot State Publisher)가 필요하면 추가 가능
    # 지금은 핵심 로직 테스트이므로 생략 가능

    return LaunchDescription([
        node_ydlidar,
        node_rf2o,
        node_vision_manager,
#  node_mqtt,
        node_driver
    ])
