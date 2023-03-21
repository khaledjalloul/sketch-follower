# Sketch Follower

A simple 2-DOF robot arm designed to learn Moveit and inverse kinematics.

The arm was designed using Fusion 360, then a URDF file was assembled based on the exported meshes. Using the Moveit_setup_assistant, the robot package was generated and tested on Rviz and Gazebo.

A simple Python Tkinter GUI was built on which a sketch can be drawn. Then, a C++ controller that uses the MoveGroup library was written to receive the sketch trajectory points and compute a cartesian path to mimic the sketch.

<img src=./sketch_follower/robot_description/demo.gif width=100% style="margin-bottom: 10px">

## How to Launch

For a demo, make sure the ROS Control packages are installed. To install them:

```
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

Clone the repository and build the catkin packages:

```
sudo catkin build
```

Run the project:

```
roslaunch sketch_follower_config demo_gazebo.launch
```