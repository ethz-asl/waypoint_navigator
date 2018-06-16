# waypoint_navigator

<p align="center"><img src="http://i.imgur.com/dDE4BA9.png" height="300"/>  <img src="http://i.imgur.com/Hl1lGQM.png" height="300"/></p>

This repository contains high-level waypoint-following for micro aerial vehicles (MAVs).
GPS/ENU co-ordinates are accepted as input destinations, and an illustrative example is provided in the [RotorS](https://github.com/ethz-asl/rotors_simulator/wiki) simulator.
This README provides a brief overview of the package and its utilities.

Please feel free to contact us in case of questions, feedback, or feature ideas. We would love to hear your feedback in order to improve this package.

**Authors**: Marija Popović, Enric Galceran, Raghav Khanna, Inkyu Sa  
**Maintainer**: Marija Popović, mpopovic@ethz.ch  
**Affiliation**: Autonomous Systems Lab, ETH Zurich

## Installation Instructions (Ubuntu)

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
 > **Optional**: You can also install the [rviz_satellite](git@github.com:gareth-cross/rviz_satellite.git) package to visualize satellite maps in rviz for GPS co-ordinates.
 
4. Use [catkin_build](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) to build the repository:

```
catkin build
```

## Node: waypoint_navigator

### Parameters:

* Parameters are stored in a .yaml file defining the trajectory to execute, which should be loaded to the parameter server. See ``trajectory_simple_enu.yaml`` and ``trajectory_simple_gps.yaml`` in the ``paths`` sub-folder for example trajectory files with parameter descriptions.

### Subscribed Topics:

* `/ground_truth/odometry` ([nav_msgs/Odometry](http://docs.ros.org/jade/api/nav_msgs/html/msg/Odometry.html)) - odometry received from MAV.

### Published Topics:

* `/command/pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)) - single pose to go to sent to controller [published at 5Hz in 'poses' path mode].
* `/path_segments` (planning_msgs/PolynomialTrajectory4D) - optimized path segments of the path in "polynomial" mode to go to the trajectory sampler, which interfaces with the controller.

### rviz Topics:

* `/waypoint_navigator_path_points_marker` ([visualization_msgs/Marker](http://docs.ros.org/jade/api/visualization_msgs/html/msg/Marker.html))- rviz markers for waypoints along a path (points)
* `/waypoint_navigator_path_marker` ([visualization_msgs/Marker](http://docs.ros.org/jade/api/visualization_msgs/html/msg/Marker.html))- rviz marker for poses/trajectory paths (line)
* `/waypoint_navigator_polynomial_markers` ([visualization_msgs/MarkerArray](http://docs.ros.org/jade/api/visualization_msgs/html/msg/MarkerArray.html)) - rviz marker for polynomial paths (line)

### Services:

* `/visualize_path` - visualize the mission in rviz
* `/execute_path` - start the mission
* `/execute_path_from_file` - start a new mission, taking new .yaml file name [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) as request
* `/go_to_waypoint` - go to a target waypoint, taking [geometry_msgs/Point](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Point.html) as request
* `/go_to_waypoints` - go to a list of ordered waypoints, taking [geometry_msgs/Point[]](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Point.html) as request
* `/go_to_height` - go to a height at the current (x,y) position, taking [std_msgs/Float64](http://docs.ros.org/jade/api/std_msgs/html/msg/Float64.html) as request
* `/takeoff` - takeoff to a height specified in the .yaml file at the current (x,y) position
* `/land` - land to a height specified in the .yaml file at the current (x,y) position
* `/abort_path` - stop executing mission and stay in place

 > Note: Example request format for a call to `/go_to_waypoints`: `rosservice call /firefly/go_to_waypoints "points: [{x: 3.0, y: 6.0, z: 2.0}, {x: 2.0, y: 9.2, z: 2.1}]"`
 
 > Note: The services `/go_to_waypoint` and `/go_to_waypoints` can be used to command the UAV to go to a target (x,y,z). The heading is set to in either the direction of the next waypoint or 0.0, depending on the mode specified in the initial file ('auto' for the former, 'fixed' or 'zero' for the latter).

## Input Format

The primary function of this package is to read a path (list of waypoints) from a .yaml file, and send commands to execute it to a [Model Predictive Controller](https://github.com/ethz-asl/mav_control_rw) (MPC). The file also contains parameters for trajectory generation (reference speed, reference acceleration, etc.) and other services (take-off height, landing height, etc.). The intermediate_pose_separation parameter, which specifies the maximum allowable distance between waypoints in the trajectory, causing intermediate points to be interpolated. Template examples of trajectories are `trajectory_simple_enu.yaml` and `trajectory_simple_gps.yaml`.


The coordinates ``.yaml`` file should read as an array of floating-point numbers, separated by commas and spaces. There are two options:

1. **ENU points** (`coordinate_type = 'enu'`):
  Array format: [x (East), y (North), z (Up), height (above starting point), yaw angle (wrt. East)]

2. **GPS points** (`coordinate_type = 'gps'`):
  Array format: [Latitude, longitude, height (above initial reference), yaw angle (wrt. East)]
 
The trajectory can be sent to the controller using one of two methods:

1. **Command poses** (`path_mode = poses`): commands to the controller are published on a pose-by-pose basis.
2. **Command polynomial trajectory** (`path_mode = polynomial`): the segments of a [smooth polynomial trajectory](git@github.com:ethz-asl/mav_trajectory_generation.git) are sampled and sent to the controller.
 
## Instructions

This package can be used with any MAV interfaced to our [MPC](https://github.com/ethz-asl/mav_control_rw). In this package, we provide an example of waypoint-following in [RotorS](https://github.com/ethz-asl/rotors_simulator/wiki) in both ENU and GPS (using a simulated GPS receiver from the [hector_gazebo_plugins](http://wiki.ros.org/hector_gazebo_plugins) package). To run the example, please follow the instructions below:

1. In a new command window, type:

 ```
 $ roslaunch waypoint_navigator mav_sim.launch
 ```
 
  By default, this launches the Gazebo RotorS simulator with AscTec Firefly MAV with GPS and IMU sensors. See `descriptions/firefly_base_gps.xacro` for details about this file. For real-life experiments, replace this launch file with one featuring the on-board start-up nodes for your MAV. Remember to ensure time synchronization.
  > **Note**: For GPS, we use the [geodetic_utils](https://github.com/ethz-asl/geodetic_utils) package to establish a GPS reference point for the local frame. These parameters are set on the ROS parameter server: `/gps_ref_latitude`, `/gps_ref_longitude`, `/gps_ref_altitude`.
  
2. If using GPS, wait for it to be initialized.

3. In a new command window, type:

 ```
 $ roslaunch waypoint_navigator waypoint_navigator.launch
 ```
 
  By default, this reads a path (`paths/trajectory_simple_enu.txt`) in ENU co-ordinates and 'polynomial' mode. The trajectory to execute can be changed in the `rosparam` load statement in the launch file.

4. In a new command window, type:

 ```
 $ rosservice call /firefly/visualize_path
 ```
   
  Using the configuration in the ``viz`` folder, you should now be able to see your loaded trajectory displayed in ``rviz``. Here, `firefly` is the MAV name and `waypoint_navigator` node namespace.
   
5. In a new command window, type:

 ```
 $ rosservice call /firefly/execute_path
 ```
 
 This begins execution of the path that was read from the file. You should see the MAV moving along the path.

## .viz configs
The configuration files in the `viz` directory can be loaded directly into rviz. As an example, we provide `waypoint_navigator_sim.viz` which can be used with the `firefly` simulation. The topics can be remapped for your MAV namespace directly.

> **Optional**: For GPS waypoints, the `AerialMapDisplay` tool from [rviz_satellite](https://github.com/gareth-cross/rviz_satellite) scan be used to render aerial imagery tiles from [MapBox](https://www.mapbox.com/) for visualization. Please consult the README in the repository for detailed instructions on how to use this tool.

Example visualization of a partially executed serpentine GPS trajectory on a sugarbeet field in Lindeau-Eschikon, Switzerland (click on images to see detail).

![](http://i63.tinypic.com/33z4etg.png)

![](http://i68.tinypic.com/301hrht.png)

## Example Application

This video demonstrates an example of waypoint-following in action:

<p align="center"><a href="https://www.youtube.com/watch?v=6i6aE--n5TY&feature=youtu.be
" target="_blank"><img src="http://i.imgur.com/LZIlwb5.png" 
alt="Demonstration video" width="600" border="10"/></a></p>

## Notes
Co-ordinate systems:
* [WGS84](https://en.wikipedia.org/wiki/World_Geodetic_System) - Global Positioning System  - [latitude, longtitude, height]
If using a GPS .txt path, initial GPS co-ordinates in file should be the same as the reference parameters specified in mav_simulator_demos/mav_simulator_demos_src/descriptions/firefly_base_gps.xacro for the GPS plug-in.
* [ECEF](https://en.wikipedia.org/wiki/ECEF) - Earth-Centred, Earth-Fixed - [x,y,z] in Earth frame
* [ENU](https://en.wikipedia.org/wiki/Axes_conventions#Ground_reference_frames:_ENU_and_NED)   - East, North, Up            - [x,y,z] in local frame

## Acknowledgement

This work was funded by the European Community’s Horizon 2020 programme under grant agreement no 644227-Flourish and from the Swiss State Secretariat for Education, Research and Innovation (SERI) under contract number 15.0029.  
http://flourish-project.eu

<p align="center"><img src="http://flourish-project.eu/fileadmin/bsdist/theme/img/flourish-logo-v5.svg" width="200" /></p>
