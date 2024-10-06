cmake_minimum_required(VERSION 3.0.2)
project(sketch_follower)

find_package(catkin REQUIRED COMPONENTS roscpp hardware_interface controller_manager)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(arduino_interface arduino/arduino_interface.cpp)
target_link_libraries(arduino_interface ${catkin_LIBRARIES})