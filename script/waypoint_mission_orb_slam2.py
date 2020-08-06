#!/usr/bin/env python

import rospy
import sys
import math

# MESSAGES
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from sensor_msgs.msg import Image
from tello_driver.msg import TelloStatus

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
        
        # PUBLISHER
        self.take_off_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.control_command_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.fast_mode_pub = rospy.Publisher('/tello/fast_mode', Empty, queue_size=1)

        rospy.loginfo("Waiting for /set_ref_pose from drone_controller node")
        rospy.wait_for_service('/set_ref_pose')
        
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)
        self.move_drone_call = rospy.ServiceProxy('/move_drone_w', MoveDroneW)

        # SUBSCRIBER
        rospy.loginfo("Waiting for ORB_SLAM2 to get image from tello")
        rospy.wait_for_message('/tello/image_repub', Image)
        self.pose_sub = rospy.Subscriber('/orb_pose', PoseStamped, self.poseCallback)
        self.pos_ux_sub = rospy.Subscriber('/pid_roll/control_effort', Float64, self.posUxCallback)
        self.pos_uy_sub = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.posUyCallback)
        self.pos_uz_sub = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.posUzCallback)
        self.pos_uyaw_sub = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.posUyawCallback)

        self.tello_status_sub = rospy.Subscriber('/tello/status', TelloStatus, self.statusCallback)
    
    def run(self):
        self.take_off()
        self.start_mission()
        self.return_home()
        self.land()

    # CALLBACK FUNCTIONS
    def poseCallback(self, msg):
        self.current_position = msg.pose.position
    
    def posUxCallback(self, msg):
        self.position_control_command.linear.x = msg.data
    
    def posUyCallback(self, msg):
        self.position_control_command.linear.y = msg.data
    
    def posUzCallback(self, msg):
        self.position_control_command.linear.z = msg.data
    
    def posUyawCallback(self, msg):
        self.position_control_command.angular.z = msg.data

    def statusCallback(self, msg):
        self.height = msg.height_m

    def calibrate_scale(self):
        rospy.loginfo("Calibrating the scale")
        self.update_target_call(0, 0, 1, 0)
        rospy.loginfo("Waiting for ORB_SLAM2 to initialize the map")
        rospy.wait_for_message('/orb_pose', PoseStamped)
        start_orb_height = self.current_position.z
        start_sensor_height = self.height
        self.move_up(0.5)
        end_orb_height = self.current_position.z
        end_sensor_height = self.height
        scale_factor_orb = (end_sensor_height - start_sensor_height) / (end_orb_height - start_orb_height)
        return scale_factor_orb

    def move_up(self, m):
        try:
            res = self.move_drone_call(0, 0, m, 0)
            check = 0   
            while not rospy.is_shutdown():
                self.control_command_pub.publish(self.position_control_command)
                if (abs(self.position_control_command.linear.z)) < 0.08:    # check z command magnitude should be near 0
                    check += 1
                if check > 300: # if the command is near 0 for 300 cycles, terminated
                    break
                self.rate.sleep()
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
    
    def start_mission(self):
        rospy.loginfo("Prepare to start the mission")
        rospy.sleep(1)
        rospy.loginfo("Mission started")
        self.enter_fast_mode()
        self.set_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():
            self.control_command_pub.publish(self.position_control_command)
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
        self.take_off_pub.publish(take_off_msg)
        rospy.loginfo("Taking Off: Finish")

    def enter_fast_mode(self):
        rospy.loginfo("Entering Fast Mode")
        self.fast_mode_pub.publish(Empty())
        rospy.loginfo("Entering Fast Mode: Finish")

    def land(self):
        rospy.loginfo("Landing: Prepare")
        self.control_command_pub.publish(self.zero_control_command)
        rospy.sleep(1)
        rospy.loginfo("Landing: Start")
        land_msg = Empty()
        self.land_pub.publish(land_msg)
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
            self.control_command_pub.publish(self.position_control_command)
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
        auto_racer.take_off()
    except rospy.ROSInterruptException:
        pass