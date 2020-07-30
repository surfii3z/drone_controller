#!/usr/bin/env python

import rospy
import sys
import math

# MESSAGES
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist

# SERVICES
from drone_controller.srv import SetRefPose

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
        self.position_control_command = Twist()
        self.vision_control_command = Twist()
        
        # PUBLISHER
        self.take_off_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.control_command_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

        rospy.loginfo("Waiting for /set_ref_pose from drone_controller node")
        rospy.wait_for_service('/set_ref_pose')
        
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)

        # SUBSCRIBER
        rospy.loginfo("Waiting for /mocap_node/Robot_4/pose from mocap_optitrack node")
        rospy.wait_for_message('/mocap_node/Robot_4/pose', PoseStamped)
        self.pose_sub = rospy.Subscriber('/mocap_node/Robot_4/pose', PoseStamped, self.poseCallback)
        self.pos_ux_sub = rospy.Subscriber('/pid_roll/control_effort', Float64, self.posUxCallback)
        self.pos_uy_sub = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.posUyCallback)
        self.pos_uz_sub = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.posUzCallback)
        self.pos_uyaw_sub = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.posUyawCallback)

        # MISSION
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

    def start_mission(self):
        rospy.loginfo("Prepare to start the mission")
        rospy.sleep(1)
        rospy.loginfo("Mission started")
        self.take_off()
        self.set_next_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():
            self.control_command_pub.publish(self.position_control_command)
            if (self.is_next_target_wp_reached()):
                if self.is_mission_finished():
                    return
                self.update_idx_wp()
                self.set_next_waypoint(self.wps[self.idx_wp])
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

    def land(self):
        rospy.loginfo("Landing: Prepare")
        zero_control_command = Twist()
        zero_control_command.linear.x = 0
        zero_control_command.linear.y = 0
        zero_control_command.linear.z = 0
        zero_control_command.angular.z = 0
        self.control_command_pub.publish(zero_control_command)
        rospy.sleep(1)
        rospy.loginfo("Landing: Start")
        land_msg = Empty()
        self.land_pub.publish(land_msg)
        rospy.loginfo("Landing: Finish")

    def set_next_waypoint(self, wp):
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
    except rospy.ROSInterruptException:
        pass