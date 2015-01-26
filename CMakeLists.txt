# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(exampleprogram)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES BumperEvent.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(listener_charles src/listener.cpp)
target_link_libraries(listener_charles ${catkin_LIBRARIES})
add_dependencies(listener_charles exampleprogram_generate_messages_cpp)

# %EndTag(FULLTEXT)%
