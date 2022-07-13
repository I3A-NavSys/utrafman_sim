cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)
project(siam_sim)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})

set (CMAKE_CXX_STANDARD 17)

#generate_messages(DEPENDENCIES std_msgs)

add_library(AirspaceController SHARED airspace-controller.cc)
add_library(AirspaceController2 SHARED airspace-controller-2.cc)

target_link_libraries(AirspaceController ${GAZEBO_LIBRARIES})
target_link_libraries(AirspaceController2 ${GAZEBO_LIBRARIES})

message(STATUS "CMAKE_MODULE_PATH=${CMAKE_MODULE_PATH}")