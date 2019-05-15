cmake_minimum_required(VERSION 2.8.3)
project(ssvep)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  emotiv_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(commands include/commands.cpp)

add_executable(command_node src/main.cpp)

target_link_libraries(command_node
   commands
   ${catkin_LIBRARIES}
 )
