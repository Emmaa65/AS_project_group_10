# AutonomousSystems2025 // Project Group 10

## SubTerrain Challenge (Project)

## Requirements
- Ubuntu 24.04.3
- ROS 2 Jazzy (install guidelines: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) (use the desktop version and install the development tools)

## System Setup
The following is a step-by-step guide on how to set up the project on a fresh virtual machine with Ubuntu 24.04, ROS 2 Jazzy and git installed.
Testing showed that the order of installation and cloning is important for the build to work because some packages depend on ROS 2 repositories being present.

1. Clone the project repository
```bash
cd # go to desired directory
# clone project repository
git clone git@github.com:Emmaa65/AS_project_group_10.git
```
2. Copy simulation files
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
You may install the following packages before cloning the project repo (step 1) and copying the simulation files (step 2).
The ```octomap_rviz_plugins``` (Step 4) however **must** come last, after the ROS 2 Octomap and PCL packages.
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
4. Clone Octomap plugins for RViz into src
```bash
cd AS_project_group_10/ros2_ws/src/
git clone https://github.com/OctoMap/octomap_rviz_plugins -b ros2
cd ../
```
## Running the simulation
1. Checkout and pull main
```bash
# run these from the repo root or in ros2_ws/
git checkout main
git pull
```
2. Build, source and launch
```bash
# source if not included in .bashrc 
source /opt/ros/jazzy/setup.bash
# build inside the workspace
cd ~/AS_project_group_10/ros2_ws
# build the project and source workspace (use --symlink-install for faster iterations)
colcon build && source install/setup.bash
# execute main launch file
ros2 launch system_bringup main.launch.py

```
3. View saved octomap afer simulation has finished
   ```bash
   ros2 launch system_bringup view_saved_octomap.launch.py
   ```
4. Text file with detected objects is stored under:
   ```bash
   ~/AS_project_group_10/ros2_ws/detected_objects.txt
   ``

  


