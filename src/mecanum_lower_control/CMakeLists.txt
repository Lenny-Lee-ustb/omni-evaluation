cmake_minimum_required(VERSION 3.0.2)
project(mecanum_lower_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  message_generation
)

catkin_package(
)

include_directories(
include
../../../devel/include
${catkin_INCLUDE_DIRS}
)

add_executable(Ramp_Test 
src/Ramp_Test.cpp 
src/include/Platform_M3508.hpp
)
target_link_libraries(Ramp_Test 
${catkin_LIBRARIES})

add_executable(mecanum_control 
src/sbus_serial_driver.cpp
src/mecanum_control.cpp 
src/include/Platform_M3508.hpp
)
target_link_libraries(mecanum_control 
${catkin_LIBRARIES})