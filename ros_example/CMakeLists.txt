cmake_minimum_required(VERSION 2.8.3)
project(lpms_ig1)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
)

link_directories("${IG1_LIB}")

set(lpms_ig1_node_SRCS
    src/lpms_ig1_node.cpp
)

## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lpms_ig1_node ${lpms_ig1_node_SRCS})

target_link_libraries(lpms_ig1_node
  ${catkin_LIBRARIES}
  LpmsIG1_OpenSourceLib.so
 )

add_dependencies(lpms_ig1_node ${catkin_EXPORTED_TARGETS})
