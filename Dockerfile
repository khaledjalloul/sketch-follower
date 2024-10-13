FROM ros:jazzy-ros-core

RUN apt update
RUN apt install -y python3-colcon-common-extensions build-essential \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro ros-jazzy-rviz2 python3-pip python3-virtualenv python3-tk

RUN addgroup realtime
RUN usermod -a -G realtime $(whoami)

RUN mkdir -p /home/colcon_ws/src/sketch_follower
WORKDIR /home/colcon_ws

RUN virtualenv env
RUN touch env/COLCON_IGNORE
RUN . env/bin/activate && pip3 install cvxpy pyyaml

COPY . src/sketch_follower

RUN . /opt/ros/jazzy/setup.sh && colcon build