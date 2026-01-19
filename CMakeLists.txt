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
  DESTINATION lib/${PROJECT_NAME}
)


# Launch, Config, URDF 폴더를 설치 목록에 추가
install(DIRECTORY
  launch
  config
  urdf
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
