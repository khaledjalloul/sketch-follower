# Sketch Follower

A 4-DOF robot arm simulation created to learn CAD modeling, inverse kinematics, and ROS Control.

The arm was designed using Fusion 360 then assembled through URDF and Xacro. The main Python scripts include:

- **cursor_publisher.py**: Tkinter GUI that records and publishes the cursor position when pressed and displays the current end effector position.
- **ros_interface.py**: Subscribes to the cursor publisher and the simulation joint states, and publishes the joint velocities to the simulation.
- **kinematics.py**: Describes the robot kinematics using numpy and solves the inverse kinematics problem.
- **controller.py**: Controls the arm by reading the desired pose, obtaining the desired joint velocities, then publishing them.

<br/>
<img src=./media/ros2.gif width=100%>

## Demo

For a demo, run docker compose inside the project:

```bash
docker compose up
```

### [ROS 1 Version](https://github.com/khaledjalloul/sketch-follower_ros/tree/ros1)

