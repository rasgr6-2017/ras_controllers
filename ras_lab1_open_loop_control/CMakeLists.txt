cmake_minimum_required(VERSION 2.8.3)
project(ras_lab1_open_loop_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ras_lab1_open_loop_control
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(open_loop_controller_node src/open_loop_controller_node.cpp)

target_link_libraries(open_loop_controller_node
  ${catkin_LIBRARIES}
)

