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
from sensor_msgs.msg import Image
from nav_msgs.msg import Path

from tello_driver.msg import TelloStatus
from tello_driver.srv import MoveUp, MoveDown
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount

# SERVICES
from drone_controller.srv import SetRefPose, MoveDroneW

ROS_RATE = 30   # Hz

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

class BBox():
    def __init__(self, bbox_msg):
        self.bbox_width = bbox_msg.xmax - bbox_msg.xmin
        self.bbox_height = bbox_msg.ymax - bbox_msg.ymin
        self.cx = (bbox_msg.xmax + bbox_msg.xmin) / 2.0
        self.cy = (bbox_msg.ymax + bbox_msg.ymin) / 2.0
        self.h_to_w_ratio = self.bbox_width / self.bbox_height
        self.object_class = bbox_msg.Class
        self.prob = bbox_msg.probability

    def get_bbox_area(self):
        return self.bbox_width * self.bbox_height

class AutoRacer():
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

        # Information from gate detection algorithm
        first_detected_img = rospy.wait_for_message("/darknet_ros/detection_image", Image)
        self.object_count = 0
        self.zero_object_count = 0
        self.detected_img_width = first_detected_img.width   # pylint: disable=no-member
        self.detected_img_height = first_detected_img.height # pylint: disable=no-member
        self.cur_target_bbox = None

        self.position_control_command = Twist()
        self.vision_control_command = Twist()
        self.zero_control_command = Twist()
        # self.optitrack_path_msg = Path()


        # PUBLISHER
        self.pub_take_off = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.pub_control_command = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)
        self.pub_fast_mode = rospy.Publisher('/tello/fast_mode', Empty, queue_size=1)
        # self.pub_path = rospy.Publisher("/drone_path", Path, queue_size=1)
        self.pub_err_x_img = rospy.Publisher("/err_x_img", Float64, queue_size=1)
        self.pub_err_y_img = rospy.Publisher("/err_y_img", Float64, queue_size=1)
        
        rospy.loginfo("Waiting for /set_ref_pose from drone_controller node")
        rospy.wait_for_service('/set_ref_pose')
        
        self.update_target_call = rospy.ServiceProxy('/set_ref_pose', SetRefPose)
        self.move_drone_call = rospy.ServiceProxy('/move_drone_w', MoveDroneW)
        self.srv_cli_up = rospy.ServiceProxy('/tello/up', MoveUp)
        self.srv_cli_down = rospy.ServiceProxy('/tello/down', MoveDown)

        # SUBSCRIBER
        rospy.logwarn("Waiting for /vrpn_client_node/Tello_jed/pose message from Optitrack")
        rospy.wait_for_message('/vrpn_client_node/Tello_jed/pose', PoseStamped)
        rospy.loginfo("/vrpn_client_node/Tello_jed/posepose message received")
        
        self.sub_pose = rospy.Subscriber('/vrpn_client_node/Tello_jed/pose', PoseStamped, self.cb_pose)
        self.sub_pos_ux = rospy.Subscriber('/pid_roll/control_effort', Float64, self.cb_pos_ux)
        self.sub_pos_uy = rospy.Subscriber('/pid_pitch/control_effort', Float64, self.cb_pos_uy)
        self.sub_pos_uz = rospy.Subscriber('/pid_thrust/control_effort', Float64, self.cb_pos_uz)
        self.sub_pos_uyaw = rospy.Subscriber('/pid_yaw/control_effort', Float64, self.cb_pos_uyaw)
        self.sub_ux_img = rospy.Subscriber("/pid_ximg/control_effort", Float64, self.cb_ux_img)
        self.sub_uz_img = rospy.Subscriber("/pid_yimg/control_effort", Float64, self.cb_uz_img)

        self.sub_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.cb_bbox)
        self.sub_obj_count = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.cb_obj_count)

        self.tello_status_sub = rospy.Subscriber('/tello/status', TelloStatus, self.cb_tello_status)

        rospy.sleep(1)

    # CALLBACK FUNCTIONS
    def cb_pose(self, msg):
        if self.frame_id != "world":
            msg.header.frame_id = self.frame_id

        # self.optitrack_path_msg.header = msg.header
        # self.optitrack_path_msg.poses.append(msg)
        # self.pub_path.publish(self.optitrack_path_msg)
        
        # update current position
        self.current_pose = msg
    
    def cb_bbox(self, msg):
        num_bboxes = len(msg.bounding_boxes)
        if num_bboxes == 0:
            # This callback function should NOT be called in this case
            assert(False), "This callback function should NOT be called in this case"
        elif num_bboxes == 1:
            # only one bbox
            self.cur_target_bbox = BBox(msg.bounding_boxes[0])
        else:
            # more than one box
            area = [BBox(msg.bounding_boxes[i]).get_bbox_area() for i in range(num_bboxes)]
            max_idx = area.index(max(area))
            self.cur_target_bbox = BBox(msg.bounding_boxes[max_idx])

    
        temp_err_x = (self.cur_target_bbox.cx - self.detected_img_width  / 2.0) / self.detected_img_width 
        temp_err_y = -(self.cur_target_bbox.cy - (self.detected_img_height - 200) / 2.0) / self.detected_img_height

        # temp_err_x = (self.cur_target_bbox.cx - self.detected_img_width  / 2.0) / self.cur_target_bbox.bbox_width 
        # temp_err_y = -(self.cur_target_bbox.cy - (self.detected_img_height - 100) / 2.0) / self.cur_target_bbox.bbox_width 

        # rospy.loginfo("temp_err_x = {}".format(temp_err_x))
        # rospy.loginfo("temp_err_y = {}".format(temp_err_y))

        if abs(temp_err_x) < 0.025:
            temp_err_x = 0
        if abs(temp_err_y) < 0.025:
            temp_err_y = 0

        err_gate_x = Float64(temp_err_x)
        err_gate_y = Float64(temp_err_y)

        self.pub_err_x_img.publish(err_gate_x)
        self.pub_err_y_img.publish(err_gate_y)
    
    def cb_obj_count(self, msg):
        # pass
        self.object_count = msg.count
        if self.object_count == 0:
            self.zero_object_count += 1
        else:
            self.zero_object_count = 0

    def cb_pos_ux(self, msg):
        self.position_control_command.linear.x = msg.data

    def cb_pos_uy(self, msg):
        self.vision_control_command.linear.y = msg.data
        self.position_control_command.linear.y = msg.data

    def cb_pos_uz(self, msg):
        self.position_control_command.linear.z = msg.data

    def cb_pos_uyaw(self, msg):
        self.vision_control_command.angular.y = msg.data
        self.position_control_command.angular.z = msg.data
    
    def cb_ux_img(self, msg):
        self.vision_control_command.linear.x = msg.data

    def cb_uz_img(self, msg):
        self.vision_control_command.linear.z = msg.data

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
            if self.object_count == 0:
                self.vision_control_command = Twist()
                self.pub_control_command.publish(self.position_control_command)
            else:
                # self.vision_control_command.linear.y = self.position_control_command.linear.y
                self.pub_control_command.publish(self.vision_control_command)

            # self.pub_control_command.publish(self.position_control_command)

            if (self.is_next_target_wp_reached(th=0.45)):
                # if self.is_mission_finished():
                    # return
                self.update_idx_wp()
                self.set_waypoint(self.wps[self.idx_wp])
            self.rate.sleep()

    def update_idx_wp(self):
        self.idx_wp = (self.idx_wp + 1) % (len(self.wps))
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
                rospy.loginfo("Mission finished: Home Reached")
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
        # start
        self.add_wp(-1.70, -0.60, 0.51, deg_to_rad(-90))

        # gate 1
        self.add_wp(1.33 - 0.30, -0.7, 0.60, deg_to_rad(-90))
        self.add_wp(1.33 + 0.60, -0.7, 0.60, deg_to_rad(-90))

        # U-turn after gate1
        self.add_wp(2.12, 0.00, 0.51, deg_to_rad(0))
        self.add_wp(2.00, 0.80, 0.51, deg_to_rad(90))
        # self.add_wp(1.60, 0.80, 0.51, deg_to_rad(90))

        # gate 2
        self.add_wp(-0.80 + 0.30, 0.76, 0.52, deg_to_rad(90))
        self.add_wp(-0.80 - 0.60, 0.76, 0.52, deg_to_rad(90))

        # U-turn after gate2
        self.add_wp(-2.30, 0.06, 0.52, deg_to_rad(130))
        self.add_wp(-2.00, -0.60, 0.52, deg_to_rad(180))
        

        self.wps = path_generator(self.wps, 0.08).poses

        

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
        auto_racer = AutoRacer()

        auto_racer.run()

        rospy.spin()
        
    except rospy.ROSInterruptException:
        auto_racer.land()
