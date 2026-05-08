cmake_minimum_required(VERSION 3.10)
project(ackermann_cmd_mux)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_node SHARED
  src/ackermann_cmd_mux_node.cpp
  src/ackermann_cmd_subscribers.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
  ackermann_msgs
  std_msgs
  yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME}_node
  yaml-cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}_node
  "ackermann_cmd_mux::AckermannCmdMuxNode"
)


install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY plugins
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()