cmake_minimum_required(VERSION 2.8.3)
project(reachability_map_tools)
## Find dependencies

set(CATKIN_PACKAGES
    geometry_msgs 
    roscpp
    std_msgs 
    sdf_tools)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES})

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################

## Catkin setup
catkin_package(
    INCLUDE_DIRS    include
    #LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
    DEPENDS         system_lib
                    Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ikfast_solvers
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O2 -g -march=native -Wall -Wextra -Wpedantic -Wconversion")

## Declare a cpp library
#add_library(${PROJECT_NAME} SHARED <files>)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -fopenmp")
add_executable(generate_iiwa14_map_no_collision_checking
    src/generate_iiwa14_map_no_collision_checking.cpp
    ikfast_solvers/iiwa14/ikfast.h
    ikfast_solvers/iiwa14/ikfast0x10000048.Transform6D.0_1_3_4_5_6_f2.cpp)
target_link_libraries(generate_iiwa14_map_no_collision_checking ${catkin_LIBRARIES})
add_dependencies(generate_iiwa14_map_no_collision_checking ${catkin_EXPORTED_TARGETS})


## Mark library for installation
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
