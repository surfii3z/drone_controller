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

from tello_driver.msg import TelloStatus
from tello_driver.srv import MoveUp, MoveDown

# SERVICES
from drone_controller.srv import SetRefPose, MoveDroneW

ROS_RATE = 30   # Hz
EFK_TWIST_SCALE = 0.5

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
        rospy.init_node("waypoint_mission", anonymous=True)
        self.rate = rospy.Rate(ROS_RATE)
        self.idx_wp = 0
        self.frame_id = "odom"
        self.is_2d_mode = True
        
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
        self.sub_ekf_pose = rospy.Subscriber('/tf', TFMessage, self.cb_ekf_pose)
        self.sub_pos_ux = rospy.Subscriber('/pid_roll/control_effort', Float64, self.cb_pos_ux)
        self.sub_pos_uy = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.cb_pos_uy)
        self.sub_pos_uz = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.cb_pos_uz)
        self.sub_pos_uyaw = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.cb_pos_uyaw)

        rospy.sleep(1)


    # CALLBACK FUNCTIONS
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
        self.pos_ctrl_cmd.linear.y = msg.data
        self.pos_ctrl_cmd_ekf.linear.y = msg.data * EFK_TWIST_SCALE

    def cb_pos_uz(self, msg):
        self.pos_ctrl_cmd.linear.z = msg.data
        self.pos_ctrl_cmd_ekf.linear.z = msg.data * EFK_TWIST_SCALE

    def cb_pos_uyaw(self, msg):
        self.pos_ctrl_cmd.angular.z = msg.data
        self.pos_ctrl_cmd_ekf.angular.z = msg.data * EFK_TWIST_SCALE
        
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
    

    def run_mission_one(self):
        '''
            MISSION ONE: following the waypoint using the pre-built map and EKF
            From Start to in front of the tunnel
            Just follow the waypoint using PD control
        '''
        rospy.loginfo("M1: Prepare to start")
        self.mission_one_wps()

        rospy.sleep(1)
        rospy.loginfo("M1: Started")
        self.set_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():
            self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
            self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)
            self.pub_wps_path.publish(self.wps_path)

            if (self.is_next_target_wp_reached(th=0.70)):
                if self.is_mission_finished():
                    rospy.loginfo("Mission ONE finished")
                    self.pub_ctrl_cmd.publish(self.zero_control_command)    # STOP
                    return
                self.update_idx_wp()
                self.set_waypoint(self.wps[self.idx_wp])
            self.rate.sleep()
    

    def run_mission_two(self):
        '''
            MISSION TWO: Cannot either do localization or EKF (control input is not in m/s, heading is bad)
            Go inside tunnul, turn right and then go out
            Use only velocity command because it cannot localize itself
            # ctrl cmd coordinate is ENU
        '''
        rospy.loginfo("M2: Prepare to start")
        rospy.sleep(1)
        rospy.loginfo("M2: Started")

        # Turn right a litter before tunnel
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd)
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd_ekf)
        self.pos_ctrl_cmd.angular.z = 0.8
        self.pos_ctrl_cmd.linear.z = 1.0
        self.pos_ctrl_cmd_ekf.angular.z = self.pos_ctrl_cmd.angular.z * EFK_TWIST_SCALE
        self.pos_ctrl_cmd_ekf.linear.z = self.pos_ctrl_cmd.linear.z * EFK_TWIST_SCALE
        self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
        self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)
        rospy.sleep(0.8)


        # Go in fast
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd)
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd_ekf)
        self.pos_ctrl_cmd.linear.y = 1.0
        self.pos_ctrl_cmd_ekf.linear.y = self.pos_ctrl_cmd.linear.y * EFK_TWIST_SCALE
        self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
        self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)
        rospy.sleep(3.9)

        # Go in (slowdown))
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd)
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd_ekf)
        self.pos_ctrl_cmd.linear.y = 0.1
        self.pos_ctrl_cmd_ekf.linear.y = self.pos_ctrl_cmd.linear.y * EFK_TWIST_SCALE
        self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
        self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)
        rospy.sleep(1.5)
        rospy.loginfo("M2: Finish going in")

        # Turn right
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd)
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd_ekf)
        self.pos_ctrl_cmd.angular.z = 1.0
        self.pos_ctrl_cmd.linear.y = 0.5
        self.pos_ctrl_cmd_ekf.linear.y = self.pos_ctrl_cmd.linear.y * EFK_TWIST_SCALE
        self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
        self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)
        rospy.sleep(3)
        rospy.loginfo("M2: Finish turning right")

        # Go out
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd)
        self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd_ekf)
        self.pos_ctrl_cmd.linear.y = 1.2
        self.pos_ctrl_cmd.angular.z = 0.1
        self.pos_ctrl_cmd.linear.z = 0.2

        self.pos_ctrl_cmd_ekf.linear.y = self.pos_ctrl_cmd.linear.y * EFK_TWIST_SCALE
        self.pos_ctrl_cmd_ekf.angular.z = self.pos_ctrl_cmd.angular.z * EFK_TWIST_SCALE
        self.pos_ctrl_cmd_ekf.linear.z = self.pos_ctrl_cmd_ekf.linear.z * EFK_TWIST_SCALE

        self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
        self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)

        rospy.sleep(3)
        rospy.loginfo("M2: Finish going out")

        rospy.loginfo("Mission TWO finished")
        self.pub_ctrl_cmd.publish(self.zero_control_command)    # STOP


    def run_mission_three(self):
        '''
            MISSION THREE: Relocalization using the pre-built map, following the waypoint
                           Basically MISSION ONE + MISSION TWO
            - Go to the position of the window (it should be able to localize itself again now)
                using PD control to follow the waypoint
            - When reaching the end point, it should move forward, up and right (simultaneously)
                because it cannot localize itself near the windows
            # ctrl cmd coordinate is ENU
        '''

        rospy.loginfo("M3: Prepare to start")
        self.mission_three_wps()

        rospy.sleep(1)
        rospy.loginfo("M3: Started")
        self.set_waypoint(self.wps[self.idx_wp])
        
        while not rospy.is_shutdown():
            self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
            self.pub_ctrl_cmd_to_ekf.publish(self.pos_ctrl_cmd_ekf)
            self.pub_wps_path.publish(self.wps_path)

            if (self.is_next_target_wp_reached(th=0.70)):
                if self.is_mission_finished():
                    self._set_ctrl_cmd_zeros(self.pos_ctrl_cmd)
                    # move forward, up and right (simultaneously)
                    self.pos_ctrl_cmd.linear.y = 1.0
                    self.pos_ctrl_cmd.linear.x = 0.5
                    self.pos_ctrl_cmd.linear.z = 0.5
                    self.pub_ctrl_cmd.publish(self.pos_ctrl_cmd)
                    rospy.sleep(3)
                    rospy.loginfo("Mission THREE finished")
                    self.pub_ctrl_cmd.publish(self.zero_control_command)    # STOP
                    return
                self.update_idx_wp()
                self.set_waypoint(self.wps[self.idx_wp])
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
    
    def mission_one_wps(self):
        '''
            Set the waypoint for Mission ONE, from start to in front of the tunnel
            # NWU coordinate
        '''
        self.idx_wp = 0
        

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
        self.add_wp(14.65,  2.00, 1.15, deg_to_rad(-120))

        self.wps_path = path_generator(self.wps, 0.05)
        self.wps = self.wps_path.poses
        rospy.loginfo("M1: Finish interpolating the waypoints")

    def mission_three_wps(self):
        '''
            Set the waypoint for Mission Three, from outside of the tunnel to the window
            # NWU coordinate
        '''
        self.idx_wp = 0
        self.wps = []

        # windows
        self.add_wp(10.80, -1.35, 1.15, deg_to_rad(-180))
        self.add_wp( 10.80, -0.70, 1.15, deg_to_rad(-180))

        self.wps_path = path_generator(self.wps, 0.05)
        self.wps = self.wps_path.poses
        rospy.loginfo("M3: Finish interpolating the waypoints")

        

    def run(self):
        '''
            Start the mission

        '''
        
        self.run_mission_one()
        self.run_mission_two()
        self.run_mission_three()
        self.land()

    # INTERNAL_FUNCTION
    
    def _L2_2Ddistance_from(self, target_wp):
        assert (isinstance(target_wp, PoseStamped)), "target_wp must be a PoseStamped"
        return math.sqrt((self.current_pose.pose.position.x - target_wp.pose.position.x) ** 2 + \
                         (self.current_pose.pose.position.y - target_wp.pose.position.y) ** 2)
    
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
