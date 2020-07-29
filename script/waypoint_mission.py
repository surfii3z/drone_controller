#!/usr/bin/env python

import rospy
import sys
import math

# MESSAGES
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, Point

# SERVICES
from drone_controller.srv import *

ROS_RATE = 30

class WayPoint():
    def __init__(self, x, y, z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw

    def L2_distance_from(self, position):
        return math.sqrt((self.x - position.x) ** 2 + (self.y - position.y) ** 2 + (self.z - position.z) ** 2)

class WaypointsMission():
    def __init__(self):
        rospy.init_node("waypoint_mission", anonymous=True)
        self.rate = rospy.Rate(ROS_RATE)
        self.idx_wp = 0
        self.wps = [WayPoint(0.1, 0.2, 0.8, 0), WayPoint(0.1, 0.4, 0.8, 0)]
        self.current_position = Point(0, 0, 0)

        # PUBLISHER
        self.take_off_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)

        rospy.loginfo("Waiting for /set_ref_pose from drone_controller node")
        rospy.wait_for_service('/set_ref_pose')
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)

        # SUBSCRIBER
        self.pose_sub = rospy.Subscriber('/mocap_node/Robot_4/pose', PoseStamped, self.poseCallback)

        # START_MISSION
        self.take_off()
        self.set_next_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():
            if (self.next_target_wp_reached()):
                self.set_next_waypoint(self.wps[self.idx_wp])
            self.rate.sleep()

    def poseCallback(self, msg):
        self.current_position = msg.pose.position

    def update_idx_wp(self):
        self.idx_wp = (self.idx_wp + 1) % len(self.wps)
        rospy.loginfo("Update next waypoint index to %d" % self.idx_wp)

    def next_target_wp_reached(self):
        if (self.wps[self.idx_wp].L2_distance_from(self.current_position) < 0.10):
            self.update_idx_wp()

    def take_off(self):
        rospy.loginfo("Taking Off")
        take_off_msg = Empty()
        self.take_off_pub.publish(take_off_msg)
        rospy.loginfo("Taking Off: Finish")

    def land(self):
        rospy.loginfo("Landing")
        land_msg = Empty()
        self.land_pub.publish(land_msg)
        rospy.loginfo("Landing: Finish")

    def set_next_waypoint(self, wp):
        try:
            res = self.update_target_call(wp.x, wp.y, wp.z, wp.yaw)
            rospy.loginfo(res)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
    
    def return_home(self):
        home_wp = WayPoint(0, 0, 0, 0)
        try:
            res = self.update_target_call(home_wp.x, home_wp.y, home_wp.z, home_wp.yaw)
            rospy.loginfo(res)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        auto_racer = WaypointsMission()
    except rospy.ROSInterruptException:
        pass