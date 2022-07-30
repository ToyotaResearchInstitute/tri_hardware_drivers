cmake_minimum_required(VERSION 3.5)
project(ati_netcanoem_ft_driver)

find_package(ament_cmake REQUIRED)
find_package(common_robotics_utilities REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## We don't depend on Drake, but we do use different build flags if present.
find_package(drake QUIET)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include src SYSTEM ${Eigen3_INCLUDE_DIRS})

## Build options
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
cmake_policy(SET CMP0069 NEW)

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-Werror=non-virtual-dtor)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wpessimizing-move)
add_compile_options(-Wuninitialized)

if(drake_FOUND)
    message(STATUS "Drake found, disabling -march=native")
else()
    message(STATUS "Drake NOT found, enabling -march=native")
    add_compile_options(-march=native)
endif()

## Declare a C++ library
add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/ati_netcanoem_ft_driver.hpp
            src/${PROJECT_NAME}/ati_netcanoem_ft_driver.cpp)
ament_target_dependencies(${PROJECT_NAME} common_robotics_utilities)

## Declare C++ components library
add_library(${PROJECT_NAME}_components SHARED
            src/ati_netcanoem_ft_driver_node.ros2.cpp)
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_components
  geometry_msgs std_srvs rclcpp rclcpp_components)

## Register C++ components
rclcpp_components_register_nodes(
  ${PROJECT_NAME}_components
  "ati_netcanoem_ft_driver::AtiNetCanOemDriverNode"
)

## Declare C++ component executable
add_executable(
  ${PROJECT_NAME}_node src/ati_netcanoem_ft_driver_node_main.ros2.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_components)

## Declare a C++ executable
add_executable(configure_ati_can_ft_sensor src/configure_ati_can_ft_sensor.cpp)
target_link_libraries(configure_ati_can_ft_sensor ${PROJECT_NAME})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_components
  configure_ati_can_ft_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(common_robotics_utilities)
ament_export_dependencies(Eigen3)

ament_package()
