FROM ros:humble

RUN apt-get update
RUN apt-get install gedit -y; exit 0
RUN apt-get update
RUN apt-get install gedit -y
RUN apt install -y xauth
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> etc/bash.bashrc
RUN apt install ros-${ROS_DISTRO}-rqt* -y
RUN apt install ros-${ROS_DISTRO}-image-transport-plugins -y
RUN apt install python3-pip -y
RUN pip install tensorflow