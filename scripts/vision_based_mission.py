#!/usr/bin/env python

import rospy
import sys
import math

# MESSAGES
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from sensor_msgs.msg import Image

ROS_RATE = 30
FORWARD_SPEED = 1.0

class BBox():
    def __init__(self, bbox_msg):
        self.width = bbox_msg.xmax - bbox_msg.xmin
        self.height = bbox_msg.ymax - bbox_msg.ymin
        self.cx = (bbox_msg.xmax + bbox_msg.xmin) / 2.0
        self.cy = (bbox_msg.ymax + bbox_msg.ymin) / 2.0
        self.cls = bbox_msg.Class
        self.prob = bbox_msg.probability

    def get_bbox_area(self):
        return self.width * self.height


class VisionBasedMission():
    def __init__(self):
        
        rospy.init_node("vision_based_mission", anonymous=True)
        first_detected_img = rospy.wait_for_message("/darknet_ros/detection_image", Image)
        self.rate = rospy.Rate(ROS_RATE)
        self.object_count = 0
        self.zero_object_count = 0
        self.detected_img_width = first_detected_img.width   # pylint: disable=no-member
        self.detected_img_height = first_detected_img.height # pylint: disable=no-member
        self.cur_target_bbox = None
        self.vision_control_command = Twist()

        self.err_x_img_pub = rospy.Publisher("/err_x_img", Float64, queue_size=1)
        self.err_y_img_pub = rospy.Publisher("/err_y_img", Float64, queue_size=1)
        self.zero_setpoint_pub = rospy.Publisher("/pid_zero_setpoint", Float64, queue_size=1)

        self.take_off_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/tello/land', Empty, queue_size=1)
        self.control_command_pub = rospy.Publisher('/tello/cmd_vel', Twist, queue_size=1)

        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bboxCallback)
        self.obj_count_sub = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.objCountCallback)

        self.ux_img_sub = rospy.Subscriber("/pid_ximg/control_effort", Float64, self.uxImgCallback)
        self.uz_img_sub = rospy.Subscriber("/pid_yimg/control_effort", Float64, self.uzImgCallback)
        
        self.start_mission()
        self.land()

    def bboxCallback(self, msg):
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

        err_gate_x = Float64((self.cur_target_bbox.cx - self.detected_img_width  / 2.0) / self.detected_img_width)
        err_gate_y = Float64(-(self.cur_target_bbox.cy - (self.detected_img_height - 200) / 2.0) / self.detected_img_height)

        self.err_x_img_pub.publish(err_gate_x)
        self.err_y_img_pub.publish(err_gate_y)

    def objCountCallback(self, msg):
        # pass
        self.object_count = msg.count
        if self.object_count == 0:
            self.zero_object_count += 1
        else:
            self.zero_object_count = 0

    def uxImgCallback(self, msg):
        self.vision_control_command.linear.x = msg.data

    def uzImgCallback(self, msg):
        self.vision_control_command.linear.z = msg.data

    def start_mission(self):
        rospy.loginfo("Prepare to start the mission")
        rospy.sleep(3)
        rospy.loginfo("Mission started")
        self.take_off()
        
        while not rospy.is_shutdown():
            self.zero_setpoint_pub.publish(Float64(0.0))
            if self.zero_object_count < 120:
                rospy.loginfo(self.zero_object_count)
                if self.object_count == 0:
                    forward_control_command = Twist()
                    forward_control_command.linear.x = 0
                    forward_control_command.linear.y = FORWARD_SPEED * 2
                    forward_control_command.linear.z = 0
                    forward_control_command.angular.z = 0
                    self.control_command_pub.publish(forward_control_command)
                    rospy.sleep(0.5)
                else:
                    self.control_command_pub.publish(self.vision_control_command)
            else:
                rospy.loginfo("Abort the mission: cannot find a gate")
                return
            self.rate.sleep()

    def take_off(self):
        rospy.loginfo("Taking Off")
        self.vision_control_command.linear.y = FORWARD_SPEED # go forward
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


if __name__ == '__main__':
    try:
        auto_racer = VisionBasedMission()
    except rospy.ROSInterruptException:
        pass
