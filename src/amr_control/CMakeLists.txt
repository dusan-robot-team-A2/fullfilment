cmake_minimum_required(VERSION 3.5)
project(fullfilment)  # 패키지 이름을 fullfilment로 설정

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fullfilment_interfaces REQUIRED)

# Install Python modules
install(PROGRAMS
  amr_contol/test/robot_node.py  # 경로 확인
  DESTINATION lib/${PROJECT_NAME}
)

# Install the package
ament_package()
