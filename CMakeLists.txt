cmake_minimum_required(VERSION 3.8)
project(sketch_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  launch
  config
  robot_description
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  sketch_follower/model/kinematics.py
  sketch_follower/model/dynamics.py
  sketch_follower/control/pid.py
  sketch_follower/control/linear_mpc.py
  sketch_follower/control/deepc.py
  sketch_follower/ros/cursor_publisher.py
  sketch_follower/ros/ros_interface.py
  sketch_follower/controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# add_executable(arduino_interface arduino/arduino_interface.cpp)
# target_link_libraries(arduino_interface ${catkin_LIBRARIES})

ament_package()
