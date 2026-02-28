# AutonomousSystems2025


## SubTerrain Challenge (Project)

![Schematics of the Drone](DroneforCave.png)

## Requirements
- ubuntu version 24.04
- ros2 jazzy (install guidelines: https://docs.ros.org/en/jazzy/Installation.html)


## Installation

Run the following commands to install all required system packages:
```bash
sudo apt update

sudo apt install -y \
    ros-jazzy-rclcpp \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-eigen \
    ros-jazzy-mav-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-cv-bridge \
    ros-jazzy-geometry-msgs \
    ros-jazzy-depth-image-proc \
    ros-jazzy-octomap \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    libeigen3-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    liboctomap-dev \
    libopencv-dev
```
for trajectory planning so far: 
sudo apt install libompl-dev
sudo apt install ros-jazzy-ompl

If still not working try to fix in Cmake: 

#add
find_package(ompl REQUIRED)

# Find OMPL using pkg-config
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OMPL REQUIRED ompl)




## Run the code

go into the ..\ros2_ws folder. There: 

```bash
source \opt\ros\jazzy\setup.bash
colcon build
source install\setup.bash
```
and then run the main launch file: 
```bash
ros2 launch system_bringup main.launch.py
```


to be installed:
sudo apt-get install ros-jazzy-depth-image-proc

sudo apt-get update && sudo apt-get install -y liboctomap-dev ros-jazzy-octomap
sudo apt-get install -y ros-jazzy-pcl-ros ros-jazzy-pcl-conversions

