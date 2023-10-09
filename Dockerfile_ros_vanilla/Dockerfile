FROM ros:humble

RUN apt-get update
RUN apt-get upgrade -y
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> etc/bash.bashrc
RUN apt install ros-${ROS_DISTRO}-rqt* -y
RUN apt install ros-${ROS_DISTRO}-image-transport-plugins -y
RUN apt install ros-${ROS_DISTRO}-rosbridge-suite -y