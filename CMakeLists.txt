cmake_minimum_required(VERSION 3.8)
project(orin_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  test_scripts/mqtt_total_control_test.py
  test_scripts/docking_controller_test.py
  test_scripts/docking_controller_test_v2.py
  test_scripts/marshaller_controller_test.py
  test_scripts/driving_controller_test.py
  test_scripts/ackermann_driver_test.py
  test_scripts/docking_ai_test.py
  test_scripts/docking_ai_test_v2.py
  test_scripts/gesture_ai_test.py
  test_scripts/generate_nav2_json.py
  sample_code/test_gripper.py
  sample_code/motor_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  test_scripts/video_stack.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME video_stack  
)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
