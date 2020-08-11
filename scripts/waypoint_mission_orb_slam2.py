#!/usr/bin/env python

import rospy
import sys
import math
import numpy as np

# MESSAGES
from std_msgs.msg import Empty, Float64, UInt8
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Path

from tello_driver.msg import TelloStatus
from tello_driver.srv import MoveUp, MoveDown


# SERVICES
from drone_controller.srv import SetRefPose, MoveDroneW

ROS_RATE = 30   # 30 Hz

class WayPoint():
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def L2_distance_from(self, position):
        return math.sqrt((self.x - position.x) ** 2 + (self.y - position.y) ** 2 + (self.z - position.z) ** 2)
    
    def L2_2Ddistance_from(self, position):
        return math.sqrt((self.x - position.x) ** 2 + (self.y - position.y) ** 2)


class WaypointsMission():
    def __init__(self):
        rospy.init_node("waypoint_mission", anonymous=True)
        self.rate = rospy.Rate(ROS_RATE)
        self.idx_wp = 0
        self.wps = [WayPoint(0.0, 0.0, 2.0, 0), WayPoint(0.0, 3.0, 2.0, 0), WayPoint(2.0, 3.0, 1.0, 0), WayPoint(2.0, 0., 1.0, 0)]
        self.home_wp = WayPoint(0.0, 0.0, 1.0, 0)
        self.current_position = Point(0, 0, 0)
        self.height = 0
        self.position_control_command = Twist()
        self.vision_control_command = Twist()
        self.zero_control_command = Twist()
        self.path_msg = Path()
        self.scale = 1
        self.is_scale_calibrate = False
        
        # PUBLISHER
        self.pub_take_off = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.pub_control_command = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.pub_fast_mode = rospy.Publisher('/tello/fast_mode', Empty, queue_size=1)
        self.pub_orb_path = None
        

        # rospy.loginfo("Waiting for /set_ref_pose from drone_controller node")
        # rospy.wait_for_service('/set_ref_pose')
        
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)
        self.move_drone_call = rospy.ServiceProxy('/move_drone_w', MoveDroneW)
        self.srv_cli_up = rospy.ServiceProxy('/tello/up', MoveUp)
        self.srv_cli_down = rospy.ServiceProxy('/tello/down', MoveDown)

        # SUBSCRIBER
        rospy.loginfo("Waiting for ORB_SLAM2 to get image from tello")
        rospy.wait_for_message('/tello/image_repub', Image)
        self.sub_pose = rospy.Subscriber('/orb_pose', PoseStamped, self.cb_pose)
        self.sub_pos_ux = rospy.Subscriber('/pid_roll/control_effort', Float64, self.cb_pos_ux)
        self.sub_pos_uy = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.cb_pos_uy)
        self.sub_pos_uz = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.cb_pos_uz)
        self.sub_pos_uyaw = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.cb_pos_uyaw)

        self.tello_status_sub = rospy.Subscriber('/tello/status', TelloStatus, self.cb_tello_status)

        rospy.sleep(1)
    
    def run(self):
        self.take_off()
        self.start_mission()
        self.return_home()
        self.land()

    # CALLBACK FUNCTIONS
    def cb_pose(self, msg):
        msg.header.frame_id = "map"
        self.path_msg.header = msg.header
        msg.pose.position.x = msg.pose.position.x * self.scale
        msg.pose.position.y = msg.pose.position.y * self.scale
        msg.pose.position.z = msg.pose.position.z * self.scale

        self.path_msg.poses.append(msg)
        

        # update current position
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y

        if not self.is_scale_calibrate:
            self.current_position.z = msg.pose.position.z
        else:
            self.current_position.z = self.height   # from height sensor
            self.pub_orb_path.publish(self.path_msg)

    def cb_pos_ux(self, msg):
        self.position_control_command.linear.x = msg.data

    def cb_pos_uy(self, msg):
        self.position_control_command.linear.y = msg.data

    def cb_pos_uz(self, msg):
        self.position_control_command.linear.z = msg.data

    def cb_pos_uyaw(self, msg):
        self.position_control_command.angular.z = msg.data

    def cb_tello_status(self, msg):
        self.height = msg.height_m

    def calibrate_scale(self):
        rospy.loginfo("Waiting for ORB_SLAM2 to initialize the map")
        rospy.wait_for_message('/orb_pose', PoseStamped)
        rospy.loginfo("ORB_SLAM2 map is initialized")
        rospy.loginfo("Calibrating the scale")
        N = 30

        # starting point
        start_orb_height = np.empty(N)
        start_sensor_height = np.empty(N)
        for i in range(N):
            start_orb_height[i] = self.current_position.z
            start_sensor_height[i] = self.height
            rospy.sleep(0.1)

        # moving up
        self.move_up(80)
        # BUG: a lot of time the drone won't go up by the first command
        if self.height - start_sensor_height[-1] < 0.1:
            rospy.logwarn("The move_up command did not work. Move up again")
            self.move_up(80)
        up_orb_height = np.empty(N)
        up_sensor_height = np.empty(N)
        for i in range(N):
            up_orb_height[i] = self.current_position.z
            up_sensor_height[i] = self.height
            rospy.sleep(0.1)   

        # moving down
        self.move_down(80)
        down_orb_height = np.empty(N)
        down_sensor_height = np.empty(N)
        for i in range(N):
            down_orb_height[i] = self.current_position.z
            down_sensor_height[i] = self.height
            rospy.sleep(0.1)

        try:
            scale_factor_orb_up = (np.median(up_sensor_height) - np.median(start_sensor_height)) / (np.median(up_orb_height) - np.median(start_orb_height))
            scale_factor_orb_down = (np.median(up_sensor_height) - np.median(down_sensor_height)) / (np.median(up_orb_height) - np.median(down_orb_height))
        except ZeroDivisionError:
            rospy.logerr("calibrate scale: zero division")
            return -1

        if scale_factor_orb_up < 0 or abs(scale_factor_orb_down / scale_factor_orb_up - 1) > 0.1:
            rospy.logwarn("The scale calibration is bad. Landing the drone")
            rospy.logwarn("scale ratio = {}".format(scale_factor_orb_up / scale_factor_orb_down))
            
            return -1

        self.scale = 0.5 * (scale_factor_orb_up + scale_factor_orb_down)
        self.is_scale_calibrate = True
        self.pub_orb_path = rospy.Publisher("/orb_path", Path, queue_size=1)    # initialize the orb path publisher 

        return scale_factor_orb_up
    


    def move_up(self, cm):
        try:
            rospy.loginfo("Start to move up %d cm" % cm)
            res = self.srv_cli_up(cm)
            rospy.loginfo(res)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    
    def move_down(self, cm):
        try:
            rospy.loginfo("Start to move down %d cm" % cm)
            res = self.srv_cli_down(cm)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    
    def start_mission(self):
        rospy.loginfo("Prepare to start the mission")
        rospy.sleep(1)
        rospy.loginfo("Mission started")
        # self.enter_fast_mode()
        self.set_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():
            self.pub_control_command.publish(self.position_control_command)
            if (self.is_next_target_wp_reached()):
                if self.is_mission_finished():
                    return
                self.update_idx_wp()
                self.set_waypoint(self.wps[self.idx_wp])
            self.rate.sleep()

    def update_idx_wp(self):
        self.idx_wp = self.idx_wp + 1
        rospy.loginfo("Update next waypoint index to %d" % self.idx_wp)

    def is_next_target_wp_reached(self):
        if (self.wps[self.idx_wp].L2_distance_from(self.current_position) < 0.30):
            return True
        else:
            return False
    
    def is_home_reached(self):
        return self.home_wp.L2_2Ddistance_from(self.current_position) < 0.10
    
    def is_mission_finished(self):
        # index start from zero
        return self.idx_wp == len(self.wps) - 1

    def take_off(self):
        rospy.loginfo("Taking Off")
        take_off_msg = Empty()
        self.pub_take_off.publish(take_off_msg)
        rospy.sleep(3)
        rospy.loginfo("Taking Off: Finish")

    def enter_fast_mode(self):
        rospy.loginfo("Entering Fast Mode")
        self.pub_fast_mode.publish(Empty())
        rospy.loginfo("Entering Fast Mode: Finish")

    def land(self):
        rospy.loginfo("Landing: Prepare")
        self.pub_control_command.publish(self.zero_control_command)
        rospy.sleep(1)
        rospy.loginfo("Landing: Start")
        land_msg = Empty()
        self.pub_land.publish(land_msg)
        rospy.loginfo("Landing: Finish")

    def set_waypoint(self, wp):
        try:
            res = self.update_target_call(wp.x, wp.y, wp.z, wp.yaw)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def return_home(self):
        rospy.loginfo("Mission finished: Returning Home")
        self.set_home_waypoint()
        while not rospy.is_shutdown():
            self.pub_control_command.publish(self.position_control_command)
            if (self.is_home_reached()):
                return
            self.rate.sleep()
    
    def set_home_waypoint(self):
        try:
            res = self.update_target_call(self.home_wp.x, self.home_wp.y, self.home_wp.z, self.home_wp.yaw)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        auto_racer = WaypointsMission()
        auto_racer.take_off()
        scale = auto_racer.calibrate_scale()
        rospy.loginfo(scale)        
        
        if scale == -1:
            auto_racer.land()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        auto_racer.land()
