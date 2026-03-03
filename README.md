# AutonomousSystems2025 // Project Group 10

## SubTerrain Challenge (Project)

## Requirements
- ubuntu version 24.04.3
- ros2 jazzy (install guidelines: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (using the desktop version and with development tools installed)

## System Setup
The following is a step by step guide on how to set up the project on a fresh virtual machine with Ubuntu 24.04, ros2 jazzy and git installed.
Testing showed that the order of installation and cloning ist important for the build to work.

1. Clone the project repository
```bash
cd # go to desired directory
# clone project repository
git clone git@github.com:Emmaa65/AS_project_group_10.git

```
2. Implement simulation files
This assumes that the ```/Simulation``` folder lies unpacked in ```/Downloads```. 
```bash
# create directory in /src/simulation/
mkdir -p ~/AS_project_group_10/ros2_ws/src/simulation/unity_sim
# copy simulation files into /src/simulation/unity_sim/
cp -r ~/Downloads/Simulation/. ~/AS_project_group_10/ros2_ws/src/simulation/unity_sim/
# make Simulation.x86_64 executable
chmod +x ~/AS_project_group_10/ros2_ws/src/simulation/unity_sim/Simulation.x86_64

```
3. Install required packages
You may install the following packages before cloning the project repo (Step 2).
The ```octomap_rviz_plugins``` (Step 4) however **must** come last
```bash
# install packages, flag -y automatically approves of installation
sudo apt install -y \
libgflags-dev \
libgoogle-glog-dev \
libnlopt-dev \
ros-jazzy-depth-image-proc \
ros-jazzy-vision-opencv \
ros-jazzy-pcl-ros \
libnlopt-cxx-dev \
ros-jazzy-nav2-msgs \
ros-jazzy-nav2-util \
libfcl-dev \
libompl-dev \
ros-jazzy-ompl \
ros-jazzy-octomap \
ros-jazzy-octomap-msgs \
ros-jazzy-octomap-server \
ros-jazzy-octomap-ros

```
4. Clone Octomap flugins for rviz into src
```bash
cd AS_project_group_10/ros2_ws/src/
git clone https://github.com/OctoMap/octomap_rviz_plugins -b ros2
cd ../
```
## Running the simulation
1. Checkout and pull main
```bash
git checkout main
git pull
```
2. Build, source and launch
```bash
# source if not included in .bashrc 
source /opt/ros/jazzy/setup.bash
# build the project and source workspace
colcon build && source install/setup.bash
# execute main launch file
ros2 launch system_bringup main.launch.py

```




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
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-util \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-message-filters \
    ros-jazzy-cv-bridge \
    ros-jazzy-geometry-msgs \
    ros-jazzy-depth-image-proc \
    ros-jazzy-octomap \
    ros-jazzy-octomap-ros \
    ros-jazzy-octomap-rviz-plugins \
    libgflags-dev \
    libgoogle-glog-dev \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    libeigen3-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    liboctomap-dev \
    libopencv-dev \
    ros-jazzy-rviz2 \
    libompl-dev \
    ros-jazzy-vision-opencv
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

