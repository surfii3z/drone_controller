#!/usr/bin/env python

import math
import numpy as np
import rospy
import copy
import tf
from waypoint_interpolation import path_generator

# MESSAGES
from std_msgs.msg import Empty, Float64, Header
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage

# from quadrotor_msgs package in Fast-planner
from quadrotor_msgs.msg import PositionCommand

# SERVICES
from drone_controller.srv import SetRefPose

ROS_RATE = 10   # Hz
EFK_TWIST_SCALE = 0.5
SCALE = 1

def shutdown_handler():
    rospy.loginfo("Shut down")

def deg_to_rad(deg):
    '''
        Convert angle in degree to radian
    '''
    deg = normalize_yaw(deg)
    return deg * math.pi / 180

def normalize_yaw(deg):
    while(deg > 180):
        deg -= 2 * 180
    while(deg < -180):
        deg += 2 * 180
    return deg

def get_rpy(wp):
    '''
        return the [roll, pitch, yaw] of the waypoint (wp)
                type: list of len(3) = len([roll, pitch, yaw])
    '''
    assert(isinstance(wp, PoseStamped)), "wp should be a PoseStamped"
    q = wp.pose.orientation
    return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

class AutoRacer():
    def __init__(self):
        rospy.init_node("thesis_mission", anonymous=True)
        self.rate = rospy.Rate(ROS_RATE)
        self.idx_wp = 0
        self.frame_id = "odom"
        # self.is_2d_mode = True
        self.is_2d_mode = False

        
        self.home_wp = PoseStamped(header=Header(stamp=rospy.Time.now(), 
                                                 frame_id=self.frame_id
                                                ),
                                   pose=Pose(position=Point(0, 0, 1),
                                             orientation=Quaternion(0, 0, 0, 1)
                                             )
                                  )
        self.wps = []

        self.current_pose = copy.deepcopy(self.home_wp)
        self.height = 0

        # Information from gate detection algorithm
        self.object_count = 0

        self.pos_ctrl_cmd = Twist()
        self.pos_ctrl_cmd_ekf = Twist()
        self.zero_control_command = Twist()
        self.slam_path_msg = Path()

        # PUBLISHER
        self.pub_take_off = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.pub_ctrl_cmd = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.pub_ctrl_cmd_to_ekf = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_wps_path = rospy.Publisher("/wps_path", Path, queue_size=1)
        
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)

        # SUBSCRIBER
        self.sub_pos_ux = rospy.Subscriber('/pid_roll/control_effort', Float64, self.cb_pos_ux)
        self.sub_pos_uy = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.cb_pos_uy)
        self.sub_pos_uz = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.cb_pos_uz)
        self.sub_pos_uyaw = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.cb_pos_uyaw)

        rospy.wait_for_message('/planning/pos_cmd', PositionCommand)
        self.sub_pos_cmd = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.cb_pos_cmd)

        rospy.sleep(1)


    # CALLBACK FUNCTIONS
    def cb_pos_ux(self, msg):
        self.pos_ctrl_cmd.linear.x = msg.data

    def cb_pos_uy(self, msg):
        self.pos_ctrl_cmd.linear.y = msg.data

    def cb_pos_uz(self, msg):
        self.pos_ctrl_cmd.linear.z = msg.data

    def cb_pos_uyaw(self, msg):
        self.pos_ctrl_cmd.angular.z = msg.data
    
    def cb_pos_cmd(self, msg):
        q_temp = tf.transformations.quaternion_from_euler(0, 0, 0).tolist()
        wp = PoseStamped(Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
                         Pose(position=Point(0, 0, 0), 
                              orientation=Quaternion(*q_temp)))

        wp.pose.position.x = msg.position.x * SCALE
        wp.pose.position.y = msg.position.y * SCALE
        wp.pose.position.z = msg.position.z * SCALE if not self.is_2d_mode else 0 
        rospy.loginfo("POS_CMD: {} {} {}".format(wp.pose.position.x, wp.pose.position.y, wp.pose.position.z))

        self.wps = wp

        
    # HELPER FUNCTIONS

    def run_mission(self):
        '''
            MISSION ONE: following the waypoint using the pre-built map and EKF
            From Start to in front of the tunnel
            Just follow the waypoint using PD control
        '''

        rospy.sleep(1)
        rospy.loginfo("M1: Started")

        while not rospy.is_shutdown():
            self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
            self.set_waypoint(self.wps)
            self.rate.sleep()

    def take_off(self):
        rospy.loginfo("Taking Off")
        take_off_msg = Empty()
        self.pub_take_off.publish(take_off_msg)
        rospy.sleep(3)
        rospy.loginfo("Taking Off: Finish")

    def land(self):
        rospy.loginfo("Landing: Prepare")
        self.pub_ctrl_cmd.publish(self.zero_control_command)
        self.pub_ctrl_cmd_to_ekf.publish(self.zero_control_command)
        rospy.sleep(1)
        rospy.loginfo("Landing: Start")
        land_msg = Empty()
        self.pub_land.publish(land_msg)
        rospy.loginfo("Landing: Finish")
        rospy.on_shutdown(shutdown_handler)

    def set_waypoint(self, wp):
        assert(isinstance(wp, PoseStamped)), "wp should be a PoseStamped"
        try:
            _, _, yaw = get_rpy(wp)
            res = self.update_target_call(wp.pose.position.x, \
                                          wp.pose.position.y, \
                                          wp.pose.position.z, \
                                          yaw)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def run(self):
        '''
            Start the mission

        '''
        
        self.run_mission()
        self.land()

    # INTERNAL_FUNCTION
    
    def _set_ctrl_cmd_zeros(self, ctrl_cmd):
        '''
            set zero Twist()
        '''
        ctrl_cmd.linear.x = 0
        ctrl_cmd.linear.y = 0
        ctrl_cmd.linear.z = 0
        ctrl_cmd.angular.x = 0
        ctrl_cmd.angular.y = 0
        ctrl_cmd.angular.z = 0
    

if __name__ == '__main__':
    try:
        auto_racer = AutoRacer()
        auto_racer.run()
        
    except rospy.ROSInterruptException:
        auto_racer.land()
