# waypoint_navigator

<p align="center"><img src="http://i.imgur.com/dDE4BA9.png" height="300"/></p>

This repository contains high-level waypoint-following for micro aerial vehicles (MAVs).
GPS/ENU co-ordinates are accepted as input destinations, and an illustrative example is provided in the [RotorS](https://github.com/ethz-asl/rotors_simulator/wiki) simulator.
This README provides a brief overview of the package and its utilities.

**Authors**: Marija Popović , Enric Galceran  
**Maintainer**: Marija Popovic, mpopovic@ethz.ch  
**Affiliation**: Autonomous Systems Lab, ETH Zurich

## Installation Instructions

To install this package with [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) or [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu):

1. Install additional system dependencies (swap indigo for kinetic as necessary):

```
sudo apt-get install python-wstool python-catkin-tools ros-indigo-cmake-modules
```

2. Set up a catkin workspace (if not already done):

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/indigo
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

3. Install the repository and its dependencies (with rosinstall):

```
cd src
wstool init
wstool set --git waypoint_navigator git@github.com:ethz-asl/waypoint_navigator.git -y
wstool update
wstool merge waypoint_navigator/install/waypoint_navigator.rosinstall
wstool update -j8
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



