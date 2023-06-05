FROM arm64v8/ros:humble
# update system
RUN apt-get update && apt-get upgrade -y
# make sure git is installed
RUN sudo apt-get install git
# clone WiringPi Repository
RUN git clone https://github.com/WiringPi/WiringPi.git
# Build WiringPi source
RUN cd WiringPi && ./build
# install opencv
# update system
RUN apt-get update
# install open cv
# ignore error if install fails on first attempt
RUN sudo apt install libopencv-dev -y; exit 0
RUN apt-get update
RUN sudo apt install libopencv-dev -y
# source ros2
RUN echo source /opt/ros/${ROS_DISTRO}/setup.bash >> etc/bash.bashrc