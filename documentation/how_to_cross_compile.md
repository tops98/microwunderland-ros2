# Development ROS2 C++ Project on x86/64 Architecture for Raspberrypi

## Summery
This instructions are focusing on compile ros2 c++ projects for arm architecture on a x86/64 architecture with hardware virtualisation using qemu and docker. An alternative option is using the ros cross-compilation features from colcon, which might offer better performance, though also take a lot more steps to setup.

## Prepare the host System
This section asumes that your host system is running ubuntu and already has docker and docker compose installed (in theory the instructions should also work on windows using a ubuntu wsl install). For information on how to install the above see [Docker offical](https://docs.docker.com/engine/install/) 
### Steps to setup docker
1. Pull docker images on host system <br>
```$ docker pull ros/arm:latest``` <br>
**Note:** *If you try to run the container with out setting up qemu you will get the following error: ***"exec format error"*** this error wil also show if you try running any programm that was compiled for a diffrent cpu architecture e.g. an c++ binary that was build on a x86/64 without cross-compilation*

2. Install qemu package <br>
```$ sudo apt-get install qemu binfmt-support qemu-user-static ```

3. Enable qemu hardware virtualization in docker ( this command has to be rerun after every restart)<br> 
```$ docker run --rm --privileged multiarch/qemu-user-static --reset -p yes```

4. Test if docker can run the previously pulled ros/arm image <br>
```$ docker run -it ros/arm:latest```

### Development enviroment
For ease of use it is recommaned to have all configurations for the development enviroment in a Docker-Composefile. See the example down below:
```
services:
  ros2_dev_env:
    container_name: ros2_dev_env
    # load ros arm image for version humble
    image: arm64v8/ros:humble

    # use same network configuration as host
    network_mode: host

    # creates a directory called "mounted_workspace" in the currently open directory if it dose not allready exist and mounts it to a directory called "ros2_workspace" in the docker container
    volumes:
      - ./mounted_workspace:/home/ros2_workspace

    # keeps the container running after it was created
    command: tail -F anything
```
You can run and access the container using the following instructions:
1. Start container in background ( -d )<br>
```$ docker-compose up -d ```

2. Find container ID<br>
```$ docker ps```

3. Acess your containers interactive Terminal using bash<br>
```$ docker exec -it [container_id] bash```

## Prepare the host System
### Install Docker on Raspberry pi
1. Ensure the system is up to date <br>
```$ sudo apt-get update && sudo apt-get upgrade -y```

2. Download setup script for docker <br>
```$ curl -fsSL https://get.docker.com -o get-docker.sh```
<br>
**Note:** It might be necessary to install ***curl***  before running this command. You can do that by  running: ```$ sudo apt-get install curl```

3. Execute the setup script <br>
```$ sudo sh get-Docker.sh```

4. Download setup script for docker <br>
```$ curl -fsSL https://get.docker.com -o get-docker.sh```

5. Add your user (normaly **pi**) to the docker youser group. This is necessary to be able to run docker commands without using sudo every time<br>
```$ sudo usermod -aG docker pi```

6. Test if docker was installed sucessfully <br>
```$ docker run hello-world```
If the installation was sucessfull the message "Hello from Docker !" should appear on your console

### Install Docker-Compose on Raspberry Pi
1. Update package list <br>
```$ sudo apt-get update```

2. Install pip3 and python3<br>
```
sudo apt-get install libffi-dev libssl-dev
sudo apt install python3-dev
sudo apt-get install -y python3 python3-pip
```
3. Get docker-compose over pip3<br>
```$ sudo pip3 install docker-compose```

4. Allow docker to start containers on boot that use the options ***allways*** or ***unless-stopped*** <br>
```$ sudo systemctl enable docker```

### Example docker-composefile for running ros2 projects on Raspberry Pi
```
services:
  ros2_env:
    container_name: ros2_env
    image: arm64v8/ros:humble
    restart: unless-stopped
    network_mode: host
    privileged: true
    devices:
      - /dev/gpiomem
    volumes:
      - ./mounted_workspace:/home/ros2_workspace
    command: tail -F anything
```
**Note:** The argument ***devices*** can be used to grant acess to the pis peripherals e.g. usb, gpio, etc.. In this example we add acess to the gpio system by adding the device "/dev/gpiomem" 

## Deploying a ros compiled ros package to the Rapberry Pi
To deploy a ros2 package to the raspberry pi you can simply copy the folder where your ***build, install log and src*** directories of your package are located to a directory of your liking on your pis home directory. After that you simply have to source your package. One thing to keep in mind though is if you build your code using the colcon option ***symlink-install*** you need to make sure that all sym-links are relative. An easy way to do so is by installing the symlink tool (```sudo apt-get install symlinks```) and run ```sudo symlinks -vs -r -c [path_to_workspace] ``` which converts all absolute symlinks in a directory into relative ones.
