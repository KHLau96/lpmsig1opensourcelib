cmake_minimum_required(VERSION 2.8.3)
project(lpms_ig1)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


generate_messages(
   DEPENDENCIES
   std_msgs
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


add_executable(quat_to_euler_node src/quat_to_euler_node.cpp)
target_link_libraries(quat_to_euler_node ${catkin_LIBRARIES})
add_dependencies(quat_to_euler_node ${catkin_EXPORTED_TARGETS})
