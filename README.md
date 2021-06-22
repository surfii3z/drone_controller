# dobot_controller

# Dependency

tello_driver
`https://github.com/surfii3z/tello_driver/tree/thesis`

robot_localization
`https://github.com/surfii3z/robot_localization/tree/thesis`

drone controller
`https://github.com/surfii3z/drone_controller/tree/thesis`

image_undistort
`https://github.com/surfii3z/image_undistort/tree/thesis`


# Start the experiment
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
