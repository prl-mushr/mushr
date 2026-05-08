cmake_minimum_required(VERSION 3.8)
project(mushr_sim)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build tools
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Export runtime support for interfaces
ament_export_dependencies(rosidl_default_runtime)

# -------------------------------
# Python package + scripts
# -------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/clicked_point_to_reposition
  scripts/fake_vesc_driver_node
  scripts/keyboard_teleop
  scripts/mushr_sim
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------
# Shared resources
# -------------------------------
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
