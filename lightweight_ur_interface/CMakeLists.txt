cmake_minimum_required(VERSION 2.8.3)
project(lightweight_ur_interface)

find_package(catkin REQUIRED COMPONENTS
             tri_driver_common
             control_msgs
             geometry_msgs
             roscpp
             rospy
             sensor_msgs
             std_msgs
             std_srvs
             message_generation)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY
                  msg
                  FILES
                  PositionCommand.msg
                  VelocityCommand.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               tri_driver_common
               control_msgs
               geometry_msgs
               roscpp
               sensor_msgs
               std_msgs
               std_srvs
               message_runtime
               DEPENDS
               Eigen3)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS}
                                   ${Eigen3_INCLUDE_DIRS})

## Build options
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-O3)
add_compile_options(-g)

## Declare a C++ library
add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/ur_robot_config.hpp
            include/${PROJECT_NAME}/ur_minimal_realtime_driver.hpp
            src/${PROJECT_NAME}/ur_minimal_realtime_driver.cpp)
add_dependencies(${PROJECT_NAME}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(ur_minimal_hardware_interface
               src/ur_minimal_hardware_interface.cpp)
add_dependencies(ur_minimal_hardware_interface
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_minimal_hardware_interface
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(ur_script_hardware_interface
               src/ur_script_hardware_interface.cpp)
add_dependencies(ur_script_hardware_interface
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_script_hardware_interface
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(ur_position_controller src/ur_position_controller.cpp)
add_dependencies(ur_position_controller
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_position_controller
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(ur_trajectory_controller src/ur_trajectory_controller.cpp)
add_dependencies(ur_trajectory_controller
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_trajectory_controller
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(ur_cartesian_controller src/ur_cartesian_controller.cpp)
add_dependencies(ur_cartesian_controller
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ur_cartesian_controller
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
        ${PROJECT_NAME}
        ur_minimal_hardware_interface
        ur_script_hardware_interface
        ur_position_controller
        ur_trajectory_controller
        ur_cartesian_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
