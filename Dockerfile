FROM ros:noetic-ros-core

RUN apt update
RUN apt-get install build-essential python3-catkin-tools python3-pip python3-tk -y
RUN pip3 install -U rosdep cvxpy

RUN apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-xacro \
    ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher -y

RUN mkdir -p /home/catkin_ws/src/sketch_follower
COPY . /home/catkin_ws/src/sketch_follower

WORKDIR /home/catkin_ws

RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -y

RUN catkin config --extend /opt/ros/noetic
RUN catkin build
RUN echo "source /home/catkin_ws/devel/setup.bash" >> /root/.bashrc

ENV DISPLAY=:1