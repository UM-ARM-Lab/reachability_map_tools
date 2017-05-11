cmake_minimum_required(VERSION 2.8.3)
project(reachability_map_tools)
## Find dependencies

set(CATKIN_PACKAGES
    geometry_msgs
    roscpp
    std_msgs
    sdf_tools
    visualization_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)

## System dependencies are found with CMake's conventions
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
    FILES
    DeprecatedReachabilityMap.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################

## Catkin setup
catkin_package(
    INCLUDE_DIRS    include
    #LIBRARIES       ${PROJECT_NAME}
    CATKIN_DEPENDS  ${CATKIN_PACKAGES} message_runtime
    DEPENDS         Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ikfast_solvers
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -g -Wall -Wextra -Wpedantic -Wconversion -fopenmp")

## Declare a cpp library for the deprecated files
add_library(${PROJECT_NAME}_deprecated
    include/${PROJECT_NAME}/deprecated_voxel_grid_serializable_wrapper_example.hpp
    src/${PROJECT_NAME}/deprecated_voxel_grid_serializable_wrapper_example.cpp
)
add_dependencies(${PROJECT_NAME}_deprecated ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_deprecated ${catkin_LIBRARIES})

## Simple reachability map generation node for the IIWA 7 arm
add_executable(generate_iiwa7_map_no_collision_checking
    src/generate_iiwa7_map_no_collision_checking.cpp
    ikfast_solvers/iiwa7/ikfast.h
    ikfast_solvers/iiwa7/ikfast0x10000049.Transform6D.0_1_3_4_5_6_f2.cpp)
target_compile_definitions(generate_iiwa7_map_no_collision_checking PRIVATE IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_link_libraries(generate_iiwa7_map_no_collision_checking ${catkin_LIBRARIES})
add_dependencies(generate_iiwa7_map_no_collision_checking ${catkin_EXPORTED_TARGETS})

## Simple reachability map generation node for the IIWA 7 arm with hand
add_executable(generate_iiwa7_map_no_collision_checking_with_hand
    src/generate_iiwa7_map_no_collision_checking_with_hand.cpp
    ikfast_solvers/iiwa7/ikfast.h
    ikfast_solvers/iiwa7/ikfast0x10000049.Transform6D.0_1_3_4_5_6_f2.cpp)
target_compile_definitions(generate_iiwa7_map_no_collision_checking_with_hand PRIVATE IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_link_libraries(generate_iiwa7_map_no_collision_checking_with_hand ${catkin_LIBRARIES})
add_dependencies(generate_iiwa7_map_no_collision_checking_with_hand ${catkin_EXPORTED_TARGETS})

## benchmark between 2 ik solvers
add_executable(benchmark_iksolver
    src/benchmark_iksolver.cpp
    ikfast_solvers/iiwa7/ikfast.h
    ikfast_solvers/iiwa7/ikfast0x10000049.Transform6D.0_1_3_4_5_6_f2.cpp)
target_compile_definitions(benchmark_iksolver PRIVATE IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_link_libraries(benchmark_iksolver ${catkin_LIBRARIES})
add_dependencies(benchmark_iksolver ${catkin_EXPORTED_TARGETS})

## Simple reachability map generation node for the IIWA 14 arm
add_executable(generate_iiwa14_map_no_collision_checking
    src/generate_iiwa14_map_no_collision_checking.cpp
    ikfast_solvers/iiwa14/ikfast.h
    ikfast_solvers/iiwa14/ikfast0x10000048.Transform6D.0_1_3_4_5_6_f2.cpp)
target_compile_definitions(generate_iiwa14_map_no_collision_checking PRIVATE IKFAST_NO_MAIN IKFAST_CLIBRARY)
target_link_libraries(generate_iiwa14_map_no_collision_checking ${catkin_LIBRARIES})
add_dependencies(generate_iiwa14_map_no_collision_checking ${catkin_EXPORTED_TARGETS})

## Simple reachability map publisher node
add_executable(publish_reachabilty_map
    src/publish_reachabilty_map.cpp
)
target_link_libraries(publish_reachabilty_map ${catkin_LIBRARIES})
add_dependencies(publish_reachabilty_map ${catkin_EXPORTED_TARGETS})


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
