cmake_minimum_required(VERSION 3.28...4.2)
project(robotiq_2_finger_gripper_driver VERSION 0.0.0 LANGUAGES C CXX)

include(CheckIPOSupported)
include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  msg/Robotiq2FingerCommand.msg
  msg/Robotiq2FingerState.msg
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES std_msgs)
rosidl_get_typesupport_target(msgs_cpp
  ${PROJECT_NAME}_msgs rosidl_typesupport_cpp)

# Defaults when this project is built on its own, as colcon does. A parent
# project or the command line can override them.
if(PROJECT_IS_TOP_LEVEL)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
  endif()
  if(NOT DEFINED CMAKE_COMPILE_WARNING_AS_ERROR)
    set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
  endif()
  if(NOT DEFINED CMAKE_INTERPROCEDURAL_OPTIMIZATION)
    check_ipo_supported(RESULT ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ${ipo_supported})
  endif()
endif()

# Warnings for this project's own targets only; consumers don't inherit them.
add_library(${PROJECT_NAME}_warnings INTERFACE)
set(${PROJECT_NAME}_warning_flags
  -Wall -Wextra -Wconversion -Wshadow -Wold-style-cast -Wpessimizing-move
  -Wuninitialized -Werror=non-virtual-dtor)
target_compile_options(${PROJECT_NAME}_warnings INTERFACE
  "$<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:${${PROJECT_NAME}_warning_flags}>")

# Hardware interface, independent of ROS.
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/robotiq_2_finger_gripper_driver.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_sources(${PROJECT_NAME}
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS include
    FILES
      include/robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    modbus
  PRIVATE $<BUILD_INTERFACE:${PROJECT_NAME}_warnings>)

# ROS node(s), as components and as standalone executables.
add_library(${PROJECT_NAME}_components SHARED
  src/robotiq_2_finger_gripper_driver_node.ros2.cpp)
target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)
target_compile_features(${PROJECT_NAME}_components PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_components
  PUBLIC
    ${PROJECT_NAME}
    rclcpp::rclcpp
    ${msgs_cpp}
  PRIVATE
    rclcpp_components::component
    $<BUILD_INTERFACE:${PROJECT_NAME}_warnings>)
rclcpp_components_register_nodes(${PROJECT_NAME}_components
  "robotiq_2_finger_gripper_driver::Robotiq2FingerDriverNode")

add_executable(${PROJECT_NAME}_node
  src/robotiq_2_finger_gripper_driver_node_main.ros2.cpp)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE ${PROJECT_NAME}_components ${PROJECT_NAME}_warnings)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  FILE_SET HEADERS)
install(TARGETS ${PROJECT_NAME}_components)
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_package()
