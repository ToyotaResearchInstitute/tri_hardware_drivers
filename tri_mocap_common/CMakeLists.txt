cmake_minimum_required(VERSION 2.8.3)
project(tri_mocap_common)

find_package(catkin REQUIRED COMPONENTS
             geometry_msgs
             message_generation
             std_msgs
             tf2_msgs
             roscpp)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY
                  msg
                  FILES
                  MocapMarker.msg
                  MocapMarkerArray.msg
                  MocapSegment.msg
                  MocapObject.msg
                  MocapState.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
               message_runtime
               geometry_msgs
               std_msgs
               tf2_msgs
               roscpp)

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
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)
add_compile_options(-Werror=non-virtual-dtor)
add_compile_options(-Wold-style-cast)
add_compile_options(-march=native)

## Declare a C++ executable
add_executable(mocap_tf_publisher_node src/mocap_tf_publisher_node.cpp)
add_dependencies(mocap_tf_publisher_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(mocap_tf_publisher_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS mocap_tf_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
