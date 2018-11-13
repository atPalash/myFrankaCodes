cmake_minimum_required(VERSION 2.8.3)
project(my_franka_pos)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  franka_hw
  geometry_msgs
  franka_msgs
  hardware_interface
  roscpp
  rospy
)

find_package(Franka 0.5.0 REQUIRED)

catkin_package(
  LIBRARIES my_franka
  CATKIN_DEPENDS
    franka_hw
    geometry_msgs
    franka_msgs
    hardware_interface
    roscpp
  DEPENDS Franka
 )

#add_executable(franka_controller_sub src/franka_controller_sub.cpp)
add_executable(franka_get_current_position src/franka_get_current_position.cpp)


#target_link_libraries(franka_controller_sub PUBLIC
#  ${Franka_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

target_link_libraries(franka_get_current_position PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

#target_include_directories(franka_controller_sub SYSTEM PUBLIC
#  ${Franka_INCLUDE_DIRS}
#  ${catkin_INCLUDE_DIRS}
#)

target_include_directories(franka_get_current_position SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
#target_include_directories(franka_controller_sub PUBLIC
 # include
#)
