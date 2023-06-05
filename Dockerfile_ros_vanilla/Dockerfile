FROM ros:humble

RUN apt-get update
RUN apt install -y xauth
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> etc/bash.bashrc
RUN apt install ros-${ROS_DISTRO}-rqt* -y