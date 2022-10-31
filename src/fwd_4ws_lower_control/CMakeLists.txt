cmake_minimum_required(VERSION 3.0.2)
project(fwd_4ws_lower_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

include_directories(
include
../../../devel/include
${catkin_INCLUDE_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fwd_control 
src/fwd_control.cpp
src/include/fwd_control.h
)
target_link_libraries(fwd_control 
${catkin_LIBRARIES})

