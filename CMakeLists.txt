# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(isen_turtlebot)

## Find catkin and any catkin packages
set(OpenCV_DIR /usr/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs OpenCV)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(itb_main src/main.cpp)
target_link_libraries(itb_main ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#C++11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
