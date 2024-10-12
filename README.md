# Sketch Follower

A 4-DOF robot arm Gazebo simulation created to learn CAD modeling, inverse kinematics, and ROS Control.

The arm was designed using Fusion 360 then assembled through URDF and Xacro. The main Python scripts include:

- **cursor_publisher.py**: Tkinter GUI that records and publishes the cursor position when pressed and displays the current end effector position.
- **ros_interface.py**: Subscribes to the cursor publisher and the simulation joint states, and publishes the joint velocities to the simulation.
- **kinematics.py**: Describes the robot kinematics using numpy and solves the inverse kinematics problem.
- **controller.py**: Controls the Gazebo simulation by reading the desired pose, obtaining the desired joint velocities, then publishing them.

<br/>
<img src=./media/3d_demo.gif width=100%>

## Demo

For a demo, install the necessary dependencies (TODO), build the colcon workspace, and run the `rviz_controller` launch file:

```
ros2 launch sketch_follower rviz_controller.launch.py
```

### [ROS 1 Version](https://github.com/khaledjalloul/sketch-follower_ros/tree/ros1)

