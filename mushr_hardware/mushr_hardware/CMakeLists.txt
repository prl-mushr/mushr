cmake_minimum_required(VERSION 3.5)
project(mushr_hardware)

## Compile as C++14, supported in ROS 2 Foxy and newer
add_compile_options(-std=c++14)

############################################################
## Find ament_cmake and other package dependencies        ##
############################################################

## In ROS 2, packages are found using ament. If your package
## depends on other ROS 2 packages, list them in the REQUIRED
## COMPONENTS section below. For example:
## find_package(rclcpp REQUIRED)
## find_package(std_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS interface files (msg/srv/action) ##
################################################

## In ROS 2, messages, services, and actions are generated
## using rosidl. To use this, follow these steps:
## * In package.xml:
##   * Add <build_depend> and <exec_depend> on "rosidl_default_generators"
##   * Add <exec_depend> on "rosidl_default_runtime"
##   * Add dependencies for all interface packages referenced
## * In this file:
##   * Add rosidl_default_generators to find_package(...)
##   * Uncomment/add rosidl_generate_interfaces(...)
##   * List every .msg/.srv/.action file to be generated
## * Dependent packages should add this package to
##   rosidl_generate_interfaces(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# set(MSG_FILES
#   "msg/Message1.msg"
#   "msg/Message2.msg"
# )

## Generate services in the 'srv' folder
# set(SRV_FILES
#   "srv/Service1.srv"
#   "srv/Service2.srv"
# )

## Generate actions in the 'action' folder
# set(ACTION_FILES
#   "action/Action1.action"
#   "action/Action2.action"
# )

## Generate all declared interfaces
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${MSG_FILES}
#   ${SRV_FILES}
#   ${ACTION_FILES}
#   DEPENDENCIES builtin_interfaces
# )

#########################################################
## Dynamic parameters and configuration in ROS 2       ##
#########################################################

## ROS 2 does not use dynamic_reconfigure. Instead:
## * Nodes typically declare parameters via rclcpp::Node::declare_parameter
## * Parameters can be updated dynamically if the node registers
##   a parameter change callback
##
## If using rclcpp_components, parameters may also be
## specified via XML/YAML loaded at runtime.

###################################
## ament package configuration    ##
###################################

## The ament_package() macro generates CMake config files for
## downstream packages. You may list:
## * INCLUDE_DIRS: Public header directories
## * LIBRARIES: Libraries provided by this package
## * DEPENDS: Non-ROS libraries needed by dependent packages
# ament_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES mushr_hardware
# #  DEPENDS Boost
# )

###########
## Build ##
###########

## Specify additional include directories — your package’s
## include/ folder should appear before others.
include_directories(
# include
# ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/mushr_hardware.cpp
# )

## If your library depends on rosidl-generated code, use:
## ament_target_dependencies(${PROJECT_NAME} rclcpp builtin_interfaces)

## Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/mushr_hardware_node.cpp)

## Recommended: manually name the target for clarity in ROS 2
## (no prefix removal is needed)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
# )

#############
## Install ##
#############

## All install targets should use ament install destinations.

## Install Python scripts (if any)
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION lib/${PROJECT_NAME}
# )

## Install executables and/or libraries
# install(TARGETS
#   ${PROJECT_NAME}
#   ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

## Install C++ header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.hpp"
# )

## Install other files such as config, launch, maps, etc.
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest-based C++ test
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(${PROJECT_NAME}_test test/test_mushr_hardware.cpp)
# if(TARGET ${PROJECT_NAME}_test)
#   target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
# endif()

## Add ament lint or pytest tests (if needed)
# ament_add_pytest_test(test_mushr_hardware test)

ament_package()
