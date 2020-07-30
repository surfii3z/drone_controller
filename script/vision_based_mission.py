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
        self.detected_img_width = first_detected_img.width   # pylint: disable=no-member
        self.detected_img_height = first_detected_img.height # pylint: disable=no-member
        self.cur_target_bbox = None

        self.bbox_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bboxCallback)
        self.obj_count_sub = rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.objCountCallback)
        
        while not rospy.is_shutdown():
            self.rate.sleep()

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

        err_gate_x = self.cur_target_bbox.cx - self.detected_img_width  / 2.0
        err_gate_y = self.cur_target_bbox.cy - self.detected_img_height / 2.0

        

    def objCountCallback(self, msg):
        # pass
        self.object_count = msg.count

if __name__ == '__main__':
    try:
        auto_racer = VisionBasedMission()
    except rospy.ROSInterruptException:
        pass
