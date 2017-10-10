cmake_minimum_required(VERSION 2.8.3)
project(ras_path_segment_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs genmsg actionlib_msgs actionlib)

add_action_files(DIRECTORY action FILES GoToPose.action)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs genmsg actionlib_msgs actionlib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(path_segment_controller src/path_segment_controller.cpp)
add_executable(action_client src/action_client.cpp)


target_link_libraries(path_segment_controller
  ${catkin_LIBRARIES}
)

target_link_libraries(action_client
  ${catkin_LIBRARIES}
)
