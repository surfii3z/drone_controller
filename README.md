## General information
The waypoint tracking (PD) controller and mission (Python) scripts to navigate DJI tello following the waypoints.

## Dependency

- [tello_driver](https://github.com/surfii3z/tello_driver/tree/thesis)
- [Fast-Planner](https://github.com/surfii3z/Fast-Planner/tree/tello_thesis)


## Installation
```bash
# PID controller package
sudo apt install ros-melodic-pid

# assume pwd is /path/to/catkin_ws/src
git clone -b thesis https://github.com/surfii3z/drone_controller.git
```




## Start the experiment
``` bash
# connect to Tello using WIFI and ROS
roslaunch tello_driver thesis_tello_node.launch

# run ORB_SLAM2 
roslaunch orb_slam2 rgbd_tello_crop.launch

# mission controller
roslaunch drone_controller waypoint_controller.launch

++++
+++
```
