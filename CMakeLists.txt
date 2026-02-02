cmake_minimum_required(VERSION 3.8)
project(orin_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# ... (다른 의존성들은 그대로 둬도 됨)


install(PROGRAMS
  scripts/ackermann_driver.py
  scripts/mqtt_nav_bridge_ssafy.py
  scripts/mqtt_path_follower.py
  scripts/docking_ai.py
  scripts/docking_controller.py
  scripts/marshaller_controller.py
  scripts/gesture_ai.py
  scripts/vision_manager.py
  scripts/vision_manager_raw.py
  scripts/mqtt_total_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  final_scripts/docking_controller_final.py
  final_scripts/driving_controller_final.py
  final_scripts/marshaller_controller_final.py
  final_scripts/mqtt_final.py
  final_scripts/vision_final.py
  final_scripts/vision_mqtt_combined.py
  final_scripts/docking_ai_final.py
  DESTINATION lib/${PROJECT_NAME}
)


# Launch, Config, URDF 폴더를 설치 목록에 추가
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
