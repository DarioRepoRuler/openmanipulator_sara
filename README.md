# Openmanipulator Sara Docker Wrapper
The goal of this repository is to ensure legacy support of the [openmanipulator sara repo](https://github.com/zang09/open_manipulator_6dof_application).
Openmanipulator Sara extends the low cost arm [Openmanipulator](https://github.com/ROBOTIS-GIT/open_manipulator) to a total of 6 DOF, enhancing the flexibility capabilities of the robot arm. 
With this docker architecture it is possible to use the code written for Ubuntu 16 with ROS1 to run on any Ubuntu distro.
This docker container was successfully tested on Ubuntu 22.04.4.

## Installation
Before you install this repo on your PC and build the docker container you should make sure that:
- [Docker](https://www.docker.com/) is installed on your host machine.
- [Post installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for docker are done. 
- [Nvidia toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) is installed.

For installation please run:
```
git clone https://github.com/DarioRepoRuler/openmanipulator_sara.git
xhost +local:root
cd openmanipulator_sara
docker build -t openmanipulator_sara .
```
This will clone the git repository on you host machine and build the docker container. 

## Deployment
After this you can run the docker container by:
```
docker run -it \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --net=host \
    --privileged \
    openmanipulator_sara
```
To launch the GUI guided controller you first have to enable the communication via ros. Please execute  
```
roscore
```
in the terminal of the docker container.

### Controller
After starting roscore you can start the controller and the simulation gazebo.
```
roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch
```

### GUI
After run controller, launch GUI program to manipulate OpenManipulator SARA.
```
roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
```