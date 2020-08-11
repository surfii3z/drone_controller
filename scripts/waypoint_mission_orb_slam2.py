#!/usr/bin/env python

import math
import numpy as np
import rospy
import copy
import tf
from waypoint_interpolation import WaypointInterpolation

# MESSAGES
from std_msgs.msg import Empty, Float64, Header
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Image
from nav_msgs.msg import Path

from tello_driver.msg import TelloStatus
from tello_driver.srv import MoveUp, MoveDown


# SERVICES
from drone_controller.srv import SetRefPose, MoveDroneW

ROS_RATE = 30   # 30 Hz

def shutdown_handler():
    rospy.loginfo("Shut down")

class WaypointsMission():
    def __init__(self):
        rospy.init_node("waypoint_mission", anonymous=True)
        self.rate = rospy.Rate(ROS_RATE)
        self.idx_wp = 0
        self.frame_id = "map"
        
        self.home_wp = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
                                   pose=Pose(position=Point(0, 0, 1),
                                                             orientation=Quaternion(0, 0, 0, 1)))
        self.wps = []

        self.current_pose = copy.deepcopy(self.home_wp)
        self.height = 0
        self.position_control_command = Twist()
        self.vision_control_command = Twist()
        self.zero_control_command = Twist()
        self.orb_path_msg = Path()
        
        self.scale = 1
        self.is_scale_calibrate = False
        self.z_bias = 0  # ORB_SLAM2 is initialized during the take-off, so z=0 is not ground

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

    # CALLBACK FUNCTIONS
    def cb_pose(self, msg):
        msg.header.frame_id = "map"
        self.orb_path_msg.header = msg.header
        msg.pose.position.x = msg.pose.position.x * self.scale
        msg.pose.position.y = msg.pose.position.y * self.scale
        msg.pose.position.z = msg.pose.position.z * self.scale

        self.orb_path_msg.poses.append(msg)
        
        self.current_pose = msg
        # update current position

        if self.is_scale_calibrate:
            self.current_pose.pose.position.z = msg.pose.position.z + self.z_bias
            self.pub_orb_path.publish(self.orb_path_msg)

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
        
    # HELPER FUNCTIONS
    def add_wp(self, x, y, z, yaw):
        q_temp = tf.transformations.quaternion_from_euler(0, 0, yaw).tolist()
        wp = PoseStamped(Header(stamp=rospy.Time.now(), frame_id=self.frame_id),
                         Pose(position=Point(x, y, z), 
                              orientation=Quaternion(*q_temp)))
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z
        self.wps.append(wp)
        

    def calibrate_scale(self, N_sample=60, sampling_freq=10, distance=80):
        '''
            parameters:
                N_sample: number of sample at each height
                sampling_freq: frequency of the sampling
                distance: distance (cm) to go up and down

            function: it calculates scale of monocular orb_slam and height bias

            type: float
        '''
        rospy.loginfo("Waiting for ORB_SLAM2 to initialize the map")
        rospy.wait_for_message('/orb_pose', PoseStamped)
        rospy.loginfo("ORB_SLAM2 map is initialized")
        rospy.loginfo("Calibrating the scale")
        N_sample = 60
        sampling_freq = 10 #Hz

        # starting point
        start_orb_height = np.empty(N_sample)
        start_sensor_height = np.empty(N_sample)
        for i in range(N_sample):
            start_orb_height[i] = self.current_pose.pose.position.z
            start_sensor_height[i] = self.height
            rospy.sleep(1/sampling_freq)

        # moving up
        self.move_up(distance)
        # BUG: a lot of time the drone won't go up by the first command
        if self.height - start_sensor_height[-1] < 0.1:
            rospy.logwarn("The move_up command did not work. Move up again")
            self.move_up(distance)
        up_orb_height = np.empty(N_sample)
        up_sensor_height = np.empty(N_sample)
        for i in range(N_sample):
            up_orb_height[i] = self.current_pose.pose.position.z
            up_sensor_height[i] = self.height
            rospy.sleep(1/sampling_freq)   

        # moving down
        self.move_down(distance)
        down_orb_height = np.empty(N_sample)
        down_sensor_height = np.empty(N_sample)
        for i in range(N_sample):
            down_orb_height[i] = self.current_pose.pose.position.z
            down_sensor_height[i] = self.height
            rospy.sleep(1/sampling_freq)

        try:
            scale_factor_orb_up = (np.median(up_sensor_height) - np.median(start_sensor_height)) / (np.median(up_orb_height) - np.median(start_orb_height))
            scale_factor_orb_down = (np.median(up_sensor_height) - np.median(down_sensor_height)) / (np.median(up_orb_height) - np.median(down_orb_height))
        except ZeroDivisionError:
            rospy.logwarn("Scale Calibration error: zero division")
            return -1

        # quality check, if not passed land
        if scale_factor_orb_up < 0 or abs(scale_factor_orb_down / scale_factor_orb_up - 1) > 0.1:
            rospy.logwarn("The scale calibration is bad. Landing the drone")
            rospy.logwarn("Scale ratio = {}".format(scale_factor_orb_up / scale_factor_orb_down))
            return -1

        # SCALE calculation
        self.scale = 0.5 * (scale_factor_orb_up + scale_factor_orb_down)
        self.is_scale_calibrate = True


        z_bias_start = self._calculate_z_bias(start_sensor_height, start_orb_height, self.scale)
        z_bias_up = self._calculate_z_bias(up_sensor_height, up_orb_height, self.scale)
        z_bias_down = self._calculate_z_bias(down_sensor_height, down_orb_height, self.scale)

        # bias calculation
        self.z_bias = (z_bias_start + z_bias_up + z_bias_down) / 3

        # initialize the orb path publisher, only for visualization in RVIZ
        self.pub_orb_path = rospy.Publisher("/orb_path", Path, queue_size=1)

        return 0
    
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

    def is_next_target_wp_reached(self, th=0.30):
        return self._L2_distance_from(self.wps[self.idx_wp]) < th
    
    def is_home_reached(self, th=0.10):
        return self._L2_2Ddistance_from(self.home_wp) < th
    
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
        rospy.on_shutdown(shutdown_handler)

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
            _, _, yaw = self._get_rpy(self.home_wp)
            res = self.update_target_call(self.home_wp.pose.position.x, \
                                          self.home_wp.pose.position.y, \
                                          self.home_wp.pose.position.z, \
                                          yaw)
            return res
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
    
    def initialize_wps(self):
        self.add_wp(0, 1, 1, 0)
        self.add_wp(1, 1, 1, math.pi / 2)
        self.add_wp(1, 0, 1, math.pi)

    def run(self):
        self.initialize_wps()
        self.take_off()
        self.start_mission()
        self.return_home()
        self.land()

    # INTERNAL_FUNCTION
    
    def _get_rpy(self, wp):
        '''
            return the [roll, pitch, yaw] of the waypoint (wp)
                   type: list of len(3) = len([roll, pitch, yaw])
        '''
        assert(isinstance(wp, PoseStamped)), "wp should be a PoseStamped"
        q = wp.pose.orientation
        return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    
    def _calculate_z_bias(self, sensor_heights, orb_heights, orb_scale):
        return np.median(sensor_heights) - np.median(orb_heights * orb_scale)
    
    def _L2_distance_from(self, target_wp):
        assert (isinstance(target_wp, PoseStamped)), "target_wp must be a PoseStamped"
        return math.sqrt((self.current_pose.pose.position.x - target_wp.pose.position.x) ** 2 + \
                         (self.current_pose.pose.position.y - target_wp.pose.position.y) ** 2 + \
                         (self.current_pose.pose.position.z - target_wp.pose.position.z) ** 2)
    
    def _L2_2Ddistance_from(self, target_wp):
        assert (isinstance(target_wp, PoseStamped)), "target_wp must be a PoseStamped"
        return math.sqrt((self.current_pose.pose.position.x - target_wp.pose.position.x) ** 2 + \
                         (self.current_pose.pose.position.y - target_wp.pose.position.y) ** 2)
    

if __name__ == '__main__':
    try:
        auto_racer = WaypointsMission()
        auto_racer.take_off()
        scale_status = auto_racer.calibrate_scale()
        rospy.loginfo(auto_racer.scale)        
        
        if scale_status == -1:
            auto_racer.land()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        auto_racer.land()
