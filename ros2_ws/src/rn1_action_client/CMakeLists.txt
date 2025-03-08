cmake_minimum_required(VERSION 3.5)
project(rn1_action_client)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/rn1_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Package exports
ament_package()
