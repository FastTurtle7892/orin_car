import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = "orin_car"
    pkg_share = get_package_share_directory(pkg_name)
    nav2_pkg = "nav2_bringup"

    # ================= [1. 파일 경로 설정] =================
    lidar_config = os.path.join(pkg_share, "config", "X4-Pro.yaml")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params2.yaml")
    map_file = os.path.join(os.path.expanduser("~"), "maps2", "my_map.yaml")
    xacro_file = os.path.join(pkg_share, "urdf", "ackermann_car.urdf.xacro")

    # ================= [2. 로봇 모델(URDF) 처리] =================
    try:
        doc = xacro.process_file(xacro_file)
        robot_desc = {"robot_description": doc.toxml()}
    except Exception as e:
        print(f"❌ Xacro 처리 실패: {e}")
        return LaunchDescription([])

    sim_time_config = {"use_sim_time": False}

    # ================= [3. 공통 하드웨어 & 센서 (즉시 실행)] =================
    node_robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_desc, sim_time_config],
    )

    node_joint_state = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_gui": False}],
    )

    node_ydlidar = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[lidar_config],
        namespace="/",
    )

    node_rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        arguments=["--ros-args", "--log-level", "ERROR"],
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom",
                "publish_tf": True,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,
            }
        ],
    )

    motor_driver = Node(
        package=pkg_name,
        executable="ackermann_driver_test.py",
        name="ackermann_driver",
        output="screen",
    )

    node_mqtt = Node(
        package=pkg_name,
        executable="mqtt_total_control_test.py",
        name="mqtt_final",
        output="screen",
    )

    # ================= [4. 자율주행 스택 (Nav2)] =================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(nav2_pkg), "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "params_file": nav2_params,
            "use_sim_time": "false",
            "autostart": "true",
        }.items(),
    )

    # ================= [5. 통합 테스트 컨트롤러] =================
    docking_ctrl = Node(package=pkg_name, executable="docking_controller_test.py", output="screen")
    driving_ctrl = Node(package=pkg_name, executable="driving_controller_test.py", output="screen")

    # ✅ 영상/마샬러/WebRTC: "카메라 1회 open"을 보장하는 통합 노드
    # - videos 폴더를 orin_car 패키지 안으로 옮긴 뒤, video_stack을 "설치된 executable"로 실행한다.
    # - setup.py(console_scripts) 기준이면 executable='video_stack'
    # - CMakeLists.txt(install PROGRAMS) 기준이면 executable='video_stack.py'
    video_stack = Node(
        package=pkg_name,
        executable="video_stack",
        name="video_stack",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            node_robot_state,
            node_joint_state,
            motor_driver,
            node_ydlidar,
            node_rf2o,
            nav2_launch,
            node_mqtt,
            docking_ctrl,
            video_stack,  # ✅ marshaller_controller_test.py 대신 이거!
            driving_ctrl,
        ]
    )
