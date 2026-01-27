import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

# ==========================================
# [설정] 미리 정의된 노드 좌표 (x, y, yaw)
# 유저 파일과 동일하게 유지함
# ==========================================
LOCATIONS = {
    'ORIGIN_GOAL':             ['-1.111', '0.201',  '-1.57'], # 초기 위치
    'NODE_1_COORD':            ['-1.111', '-0.707', '-1.57'], # 노드 1
    'NODE_2_COORD':            ['-1.111', '-1.615', '-1.57'], # 노드 2
    'NODE_3_COORD':            ['1.15',   '-1.615', '1.57'],  # 노드 3
    'NODE_3_WAYPOINT':       ['-0.195', '-2.39',  '0.0'],     
}

def launch_setup(context, *args, **kwargs):
    # 1. 실행 시 입력받은 'start_node' 아규먼트 값 가져오기
    start_node_name = LaunchConfiguration('start_node').perform(context)
    
    # 2. 좌표 조회 (없으면 기본값 ORIGIN_GOAL 사용)
    # [수정] LOCATIONS 키에 맞게 기본값도 'ORIGIN_GOAL'로 안전하게 처리
    if start_node_name in LOCATIONS:
        target_pos = LOCATIONS[start_node_name]
        print(f"✅ [Simulation] Start Location: {start_node_name} -> {target_pos}")
    else:
        print(f"⚠️ [Warning] Unknown node '{start_node_name}'. Defaulting to 'ORIGIN_GOAL'.")
        target_pos = LOCATIONS['ORIGIN_GOAL']

    x_pos, y_pos, yaw = target_pos

    # 3. 패키지 경로 설정
    pkg_name = 'orin_car'
    pkg_share = get_package_share_directory(pkg_name)
    nav2_share = get_package_share_directory('nav2_bringup')
    
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    xacro_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf.xacro')
    
    # 4. Robot State Publisher
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

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': False}]
    )

    # 5. [가상 TF] 로봇 위치 고정 (Odom -> Base_link)
    node_fake_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[x_pos, y_pos, '0.0', yaw, '0.0', '0.0', 'odom', 'base_link'],
        output='screen'
    )

    # 6. [가상 TF] 지도 고정 (Map -> Odom) - 완전 가상 모드용
    node_fake_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # 7. [추가됨] MQTT 브릿지 노드 자동 실행 (파라미터 전달)
    node_mqtt_bridge = Node(
        package='orin_car',
        executable='mqtt_nav_bridge_ssafy.py',
        output='screen',
        parameters=[{
            'init_x': float(x_pos),
            'init_y': float(y_pos),
            'init_yaw': float(yaw)
        }]
    )

    # 8. Nav2 실행
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'false',
            'autostart': 'true'
        }.items()
    )

    return [
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_fake_odom_tf,
        node_fake_map_tf,
        node_mqtt_bridge,  # 브릿지 노드 추가됨!
        nav2_launch
    ]

def generate_launch_description():
    pkg_name = 'orin_car'
    default_map_path = os.path.join(os.path.expanduser('~'), 'maps', 'my_map.yaml')
    pkg_share = get_package_share_directory(pkg_name)
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Launch Arguments 정의
    map_arg = DeclareLaunchArgument('map', default_value=default_map_path)
    params_arg = DeclareLaunchArgument('params_file', default_value=nav2_params_path)
    
    # [수정] 시작 노드 선택 아규먼트 (기본값을 딕셔너리 키 'ORIGIN_GOAL'로 맞춤)
    start_node_arg = DeclareLaunchArgument(
        'start_node', 
        default_value='ORIGIN_GOAL',
        description='Start position node name: ORIGIN_GOAL, NODE_1_COORD, NODE_2_COORD...'
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        start_node_arg,
        OpaqueFunction(function=launch_setup)
    ])