# Openmanipulator Sara Docker Wrapper
This repository provides a Docker-based solution to ensure legacy support for the [openmanipulator sara repo](https://github.com/zang09/open_manipulator_6dof_application).
Openmanipulator Sara extends the low cost arm [Openmanipulator](https://github.com/ROBOTIS-GIT/open_manipulator) to a total of 6 DOF, enhancing the flexibility capabilities of the robot arm. 
The Docker architecture in this repository allows the code originally designed for Ubuntu 16 and ROS1 to run seamlessly on any Ubuntu distribution. It has been successfully tested on Ubuntu 22.04.4.


## Prerequisites
Before installing and building the Docker container, ensure the following dependencies are installed:
- [Docker](https://www.docker.com/) Install Docker on your host machine.
- [Post installation steps](https://docs.docker.com/engine/install/linux-postinstall/) Complete these steps to enable non-root Docker usage. 
- [Nvidia toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) Install the toolkit for GPU support.

## Installation
Follow these steps to clone the repository and build the Docker container:
```
# Clone the repository
git clone https://github.com/DarioRepoRuler/openmanipulator_sara.git
# Allow local root access to the X server
xhost +local:root
# Navigate to the repository
cd openmanipulator_sara
# Build the Docker container
docker build -t openmanipulator_sara .
```
This process clones the repository to your host machine and builds the Docker container.

## Deployment
After building the Docker container, you can run it using the following command:
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

### ROS Setup
To prepare ROS in the Docker container, source the setup file from the build directory:
```
cd ~/catkin_ws
source devel/setup.bash
```
### Starting ROS Core
Enable ROS communication by running:
```
roscore
```
This must be done within the docker container.

### Controller and Simulation
After starting `roscore`, you can start the controller and the gazebo simulation in seperate terminale.
### Opening additional terminals
To open a new terminal for the same Docker container:
1. List the running docker containers.
    ```
    docker ps
    ```
    This will return the docker **container name** (which is usually not the same as you named the image).
2. Connect to the container:
    ```
    docker exec -it epic_pare bash
    ```
3. Source ROS in new terminal
    Be sure to always source via:
    ```
    cd ~/catkin_ws
    source devel/setup.bash
    ```
### Launch Controller and Simulation
Run the following commands in separate terminals:

```
# Launch the controller
roslaunch open_manipulator_6dof_controller open_manipulator_6dof_controller.launch use_platform:=false
# Launch the Gazebo simulation
roslaunch open_manipulator_6dof_gazebo open_manipulator_6dof_gazebo.launch
```
![Gazebo simulation](fig/gazebo_openmani_sara.png)

### GUI
Once the controller is running, launch GUI program to manipulate OpenManipulator SARA.
```
roslaunch open_manipulator_6dof_control_gui open_manipulator_6dof_control_gui.launch
```

### Further use
For additional features and further use, please refer to [Openmanipulator Sara](https://github.com/DarioRepoRuler/openmanipulator_sara). This repository provides the same functionality as the original, with the added benefit of container