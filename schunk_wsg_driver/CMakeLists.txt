cmake_minimum_required(VERSION 2.8.3)
project(schunk_wsg_driver)

find_package(catkin REQUIRED COMPONENTS
             tri_driver_common
             roscpp
             message_generation)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES WSGState.msg WSGCommand.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               tri_driver_common
               roscpp
               message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

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
            include/${PROJECT_NAME}/schunk_wsg_driver_common.hpp
            include/${PROJECT_NAME}/schunk_wsg_driver_ethernet.hpp
            include/${PROJECT_NAME}/schunk_wsg_driver_can.hpp
            src/${PROJECT_NAME}/schunk_wsg_driver_common.cpp
            src/${PROJECT_NAME}/schunk_wsg_driver_ethernet.cpp
            src/${PROJECT_NAME}/schunk_wsg_driver_can.cpp)
add_dependencies(${PROJECT_NAME}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/schunk_wsg_driver_node.cpp)
add_dependencies(${PROJECT_NAME}_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN ".svn" EXCLUDE)
