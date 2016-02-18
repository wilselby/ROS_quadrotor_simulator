cmake_minimum_required(VERSION 2.8.3)
project(quad_joystick_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  sensor_msgs
# trajectory_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs mav_msgs sensor_msgs
  LIBRARIES
)

add_executable(quad_joystick_interface src/joy.cpp)
add_dependencies(quad_joystick_interface ${catkin_EXPORTED_TARGETS})
target_link_libraries(quad_joystick_interface ${catkin_LIBRARIES})
