# dobot_controller

# Dependency

tello_driver
`https://github.com/surfii3z/tello_driver/tree/moma`

robot_localization
`https://github.com/surfii3z/robot_localization/tree/moma`

drone controller
`https://github.com/surfii3z/drone_controller/tree/moma`

image_undistort
`https://github.com/surfii3z/image_undistort/tree/moma`


# Start the experiment
``` bash
# connect to Tello using WIFI and ROS
roslaunch tello_driver moma_tello_node.launch


# run ORB_SLAM2 with MAP
roslaunch orb_slam2 mono_tello.launch

# mission controller
roslaunch drone_controller moma_controller.launch

# Here we manually take off the drone

# EKF
roslaunch robot_localization ekf_template.launch

# Start the mission
rosrun drone_controller moma_mission.py
```
