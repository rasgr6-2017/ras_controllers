cmake_minimum_required(VERSION 2.8.3)
project(ras_lab1_motor_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs phidgets)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs phidgets
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controller src/motor_controller.cpp)

target_link_libraries(motor_controller
  ${catkin_LIBRARIES}
)

