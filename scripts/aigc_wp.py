#!/usr/bin/env python

import math
import numpy as np
import rospy
import copy
import tf
from waypoint_interpolation import path_generator

# MESSAGES
from std_msgs.msg import Empty, Float64, Header
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage

from tello_driver.msg import TelloStatus
from tello_driver.srv import MoveUp, MoveDown

# SERVICES
from drone_controller.srv import SetRefPose, MoveDroneW

ROS_RATE = 30   # Hz
EFK_TWIST_SCALE = 0.5

def shutdown_handler():
    rospy.loginfo("Shut down")

def deg_to_rad(deg):
    deg = normalize_yaw(deg)
    return deg * math.pi / 180

def normalize_yaw(deg):
    while(deg > 180):
        deg -= 2 * 180
    while(deg < -180):
        deg += 2 * 180
    return deg

class AutoRacer():
    def __init__(self):
        rospy.init_node("waypoint_mission", anonymous=True)
        self.rate = rospy.Rate(ROS_RATE)
        self.idx_wp = 0
        self.frame_id = "odom"
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
        self.zero_object_count = 0
        self.cur_target_bbox = None

        self.pos_ctrl_cmd = Twist()
        self.pos_ctrl_cmd_ekf = Twist()
        self.vision_control_command = Twist()
        self.zero_control_command = Twist()
        self.slam_path_msg = Path()

        # PUBLISHER
        self.pub_take_off = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.pub_ctrl_cmd = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.pub_ctrl_cmd_to_ekf = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_fast_mode = rospy.Publisher('/tello/fast_mode', Empty, queue_size=1)
        self.pub_wps_path = rospy.Publisher("/wps_path", Path, queue_size=1)
        self.pub_err_x_img = rospy.Publisher("/err_x_img", Float64, queue_size=1)
        self.pub_err_y_img = rospy.Publisher("/err_y_img", Float64, queue_size=1)
        
        # rospy.loginfo("Waiting for /set_ref_pose from drone_controller node")
        # rospy.wait_for_service('/set_ref_pose')
        
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)
        self.move_drone_call = rospy.ServiceProxy('/move_drone_w', MoveDroneW)
        self.srv_cli_up = rospy.ServiceProxy('/tello/up', MoveUp)
        self.srv_cli_down = rospy.ServiceProxy('/tello/down', MoveDown)

        # SUBSCRIBER
        # rospy.loginfo("Waiting for openvslam pose")
        # rospy.wait_for_message('/openvslam/camera_pose', PoseStamped)
        
        # self.sub_pose   = rospy.Subscriber('/openvslam/camera_pose', PoseWithCovarianceStamped, self.cb_pose)
        self.sub_ekf_pose = rospy.Subscriber('/tf', TFMessage, self.cb_ekf_pose)
        self.sub_pos_ux = rospy.Subscriber('/pid_roll/control_effort', Float64, self.cb_pos_ux)
        self.sub_pos_uy = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.cb_pos_uy)
        self.sub_pos_uz = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.cb_pos_uz)
        self.sub_pos_uyaw = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.cb_pos_uyaw)

        self.tello_status_sub = rospy.Subscriber('/tello/status', TelloStatus, self.cb_tello_status)

        rospy.sleep(1)

    # CALLBACK FUNCTIONS
    # def cb_pose(self, msg):
    #     self.current_pose = msg
    
    def cb_ekf_pose(self, msg):
        self.current_pose.header = msg.transforms[0].header
        self.current_pose.pose.position.x = msg.transforms[0].transform.translation.x
        self.current_pose.pose.position.y = msg.transforms[0].transform.translation.y
        self.current_pose.pose.position.z = msg.transforms[0].transform.translation.z
        self.current_pose.pose.orientation = msg.transforms[0].transform.rotation

    def cb_pos_ux(self, msg):
        self.pos_ctrl_cmd.linear.x = msg.data
        self.pos_ctrl_cmd_ekf.linear.x = msg.data * EFK_TWIST_SCALE

    def cb_pos_uy(self, msg):
        self.vision_control_command.linear.y = msg.data
        self.pos_ctrl_cmd.linear.y = msg.data
        self.pos_ctrl_cmd_ekf.linear.y = msg.data * EFK_TWIST_SCALE

    def cb_pos_uz(self, msg):
        self.pos_ctrl_cmd.linear.z = msg.data
        self.pos_ctrl_cmd_ekf.linear.z = msg.data * EFK_TWIST_SCALE

    def cb_pos_uyaw(self, msg):
        self.vision_control_command.angular.y = msg.data
        self.pos_ctrl_cmd.angular.z = msg.data
        self.pos_ctrl_cmd_ekf.angular.z = msg.data * EFK_TWIST_SCALE

    def cb_tello_status(self, msg):
        self.height = msg.height_m
        
    # HELPER FUNCTIONS
    def add_wp(self, x, y, z, yaw):
        q_temp = tf.transformations.quaternion_from_euler(0, 0, yaw).tolist()
        wp = PoseStamped(Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
                         Pose(position=Point(x, y, z), 
                              orientation=Quaternion(*q_temp)))
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z if not self.is_2d_mode else 0 
        self.wps.append(wp)
    
    def start_mission(self):
        # self.set_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():

            self.pub_wps_path.publish(self.wps_path)
            self.rate.sleep()

    def update_idx_wp(self):
        self.idx_wp = self.idx_wp + 1
        rospy.loginfo("Update next waypoint index to %d" % self.idx_wp)

    def is_next_target_wp_reached(self, th=0.30):
        return self._L2_2Ddistance_from(self.wps[self.idx_wp]) < th
    
    def is_home_reached(self, th=0.30):
        return self._L2_2Ddistance_from(self.home_wp) < th
    
    def is_mission_finished(self):
        # index start from zero
        return self.idx_wp == len(self.wps) - 1
    
    def initialize_wps(self):
        # NWU coordinate

        # start

        self.add_wp( 0.50, -0.15, 1.15, deg_to_rad(0))

        # right side of the wall
        self.add_wp( 1.24, -0.70, 1.15, deg_to_rad(0))
        self.add_wp( 3.10, -0.53, 1.05, deg_to_rad(0))
        self.add_wp( 4.00,  0.80, 1.05, deg_to_rad(0))

        # poles
        self.add_wp( 5.00,  1.10, 1.05, deg_to_rad(0))
        self.add_wp( 6.60,  1.62, 0.85, deg_to_rad(0))

        # trees
        self.add_wp( 9.00,  1.50, 0.85, deg_to_rad(0))
        self.add_wp(11.20,  2.60, 1.00, deg_to_rad(0))

        # before tunnel
        self.add_wp(12.50,  2.80, 1.15, deg_to_rad( -5))
        self.add_wp(13.00,  3.00, 1.15, deg_to_rad(-30))
        self.add_wp(14.00,  2.80, 1.15, deg_to_rad(-75))
        self.add_wp(14.65,  1.80, 1.15, deg_to_rad(-110))
        

        self.wps_path = path_generator(self.wps, 0.05)
        self.wps = self.wps_path.poses
        rospy.loginfo(len(self.wps))

        rospy.loginfo("Finish interpolating the waypoints")
    
    def set_waypoint(self, wp):
        assert(isinstance(wp, PoseStamped)), "wp should be a PoseStamped"
        try:
            _, _, yaw = self._get_rpy(wp)
            res = self.update_target_call(wp.pose.position.x, \
                                          wp.pose.position.y, \
                                          wp.pose.position.z, \
                                          yaw)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def _get_rpy(self, wp):
        '''
            return the [roll, pitch, yaw] of the waypoint (wp)
                   type: list of len(3) = len([roll, pitch, yaw])
        '''
        assert(isinstance(wp, PoseStamped)), "wp should be a PoseStamped"
        q = wp.pose.orientation
        return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))

    def run(self):
        self.initialize_wps()
        self.start_mission()
    
if __name__ == '__main__':
    auto_racer = AutoRacer()
   
    auto_racer.run()