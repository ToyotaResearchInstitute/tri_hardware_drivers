cmake_minimum_required(VERSION 2.8.3)
project(ati_netcanoem_ft_driver)

find_package(catkin REQUIRED COMPONENTS
             tri_driver_common
             geometry_msgs
             roscpp)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(INCLUDE_DIRS
               include
               LIBRARIES
               ${PROJECT_NAME}
               CATKIN_DEPENDS
               tri_driver_common
               geometry_msgs
               roscpp
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
            include/${PROJECT_NAME}/ati_netcanoem_ft_driver.hpp
            src/${PROJECT_NAME}/ati_netcanoem_ft_driver.cpp)
add_dependencies(${PROJECT_NAME}
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/ati_netcanoem_ft_driver_node.cpp)
add_dependencies(${PROJECT_NAME}_node
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(configure_ati_can_ft_sensor src/configure_ati_can_ft_sensor.cpp)
add_dependencies(configure_ati_can_ft_sensor
                 ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(configure_ati_can_ft_sensor
                      ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node configure_ati_can_ft_sensor
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
