FROM arm64v8/ros:humble

# update and upgrade system
RUN apt-get update && apt-get upgrade -y
# install git if not present
RUN apt-get install git
# clone and build WiringPi libary
RUN cd home/ && \
    git clone https://github.com/WiringPi/WiringPi.git && \
    cd WiringPi/ && \
    ./build