# dobot_controller

# Dependency
openVSLAM
`https://github.com/surfii3z/openvslam/tree/aigc_ekf`

robot_localization
`https://github.com/surfii3z/robot_localization/tree/aigc_ekf`

drone controller
`https://github.com/surfii3z/drone_controller/tree/aigc_ekf`

tello_driver
`https://github.com/surfii3z/tello_driver`

image_undistort
`https://github.com/surfii3z/image_undistort`


# Start the experiment
``` bash
# connect to Tello using WIFI and ROS
roslaunch tello_driver tello_node.launch

# preprocess the video stream
roslaunch image_undistort tello_undistort.launch

# run openVSLAM
roslaunch openvslam mono_tello_undistorted_localization.launch

# mission controller
roslaunch drone_controller aigc_controller.launch

# Here we manually take off the drone

# EKF
roslaunch robot_localization ekf_template.launch

# Start the mission
rosrun drone_controller aigc_mission_fsm.py
```
