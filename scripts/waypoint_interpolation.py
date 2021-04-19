#!/usr/bin/env python

import rospy
import math
import copy
import time

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Point, Quaternion
from nav_msgs.msg import Path
import tf

def deg_to_rad(deg):
    deg = normalize_yaw(deg)
    return deg * math.pi / 180

def normalize_yaw(rad):
    while(rad > math.pi):
        rad -= 2 * math.pi
    while(rad < -math.pi):
        rad += 2 * math.pi
    return rad

class WaypointInterpolation:
    def __init__(self, wp_start, wp_end, dis_res):
        '''
            parameters:
                wp_start: starting point, type: PoseStamped
                wp_end  : end point,    type: PoseStamped
                dis_res : distance_resolution, type: (positive) int, float
        '''
        assert(isinstance(wp_start, PoseStamped)), "wp_start should be a PoseStamped"
        assert(isinstance(wp_end, PoseStamped))  , "wp_end should be a PoseStamped"
        assert(isinstance(dis_res, (int, float)) and dis_res > 0), "dis_res should be a positive real number" 
        
        self.wp_start = wp_start
        self.wp_end = wp_end
        self.dis_res = dis_res
        self.frame_id = self.wp_start.header.frame_id

        self.dis_vec = self._get_displacement_vector()  
        self.dir_vec = self._get_direction_vector()
        self.distance = self._get_distance()
        self.rpy_start = self._get_rpy(self.wp_start)
        self.rpy_end = self._get_rpy(self.wp_end)
        
        self.path = None
        self._interpolate()

    def get_path(self):
        return self.path
    
    def _get_displacement_vector(self):
        '''
            return the vector from the starting point to the end point (direction and magnitude)
                   type: Vector3
        '''
        dis_vec = Vector3()
        dis_vec.x = self.wp_end.pose.position.x - self.wp_start.pose.position.x
        dis_vec.y = self.wp_end.pose.position.y - self.wp_start.pose.position.y
        dis_vec.z = self.wp_end.pose.position.z - self.wp_start.pose.position.z
        return dis_vec

    def _get_direction_vector(self):
        '''
            return the normalized direction vector from starting point to the end point3
                   type: float
        '''
        dir_vec = copy.deepcopy(self.dis_vec)
        magnitude = self._L2_norm(dir_vec)
        dir_vec.x = dir_vec.x / magnitude
        dir_vec.y = dir_vec.y / magnitude
        dir_vec.z = dir_vec.z / magnitude
        return dir_vec

    def _get_distance(self):
        '''
            return the distance from the starting point to the end point
                   type: float
        '''
        return self._L2_norm(self.dis_vec)

    def _L2_norm(self, vec):
        '''
            return the L2-norm of a vector
                   type: float
        '''
        assert(isinstance(vec, Vector3)), "vec should be a Vector3"
        return math.sqrt(vec.x ** 2 + vec.y ** 2 + vec.z **2)

    def _get_rpy(self, wp):
        '''
            return the [roll, pitch, yaw] of the waypoint (wp)
                   type: list of len(3) = len([roll, pitch, yaw])
        '''
        assert(isinstance(wp, PoseStamped)), "wp should be a PoseStamped"
        q = wp.pose.orientation
        return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
    
    def _interpolate(self):
        '''
            interpolate the waypoint from the starting point to the end point 
            and put it in nav_msgs/Path message
        '''
        self.path = Path()
        self.path.header = self.wp_start.header
        step = int(math.ceil(self.distance / self.dis_res)) 
        
        if step == 1:   # no need to interpolate            
            self.path.poses.append(self.wp_start)
            self.path.poses.append(self.wp_end)

        else:
            roll_diff = self.rpy_end[0] - self.rpy_start[0]
            pitch_diff = self.rpy_end[1] - self.rpy_start[1]
            yaw_diff = self.rpy_end[2] - self.rpy_start[2]
            yaw_diff = normalize_yaw(yaw_diff)
            roll_res = roll_diff / step
            pitch_res = pitch_diff / step
            yaw_res = yaw_diff / step
            position_list = [[None, None, None] for n in range(step)]
            quaternion_list = [None for n in range(step)]

            # linear interpolatation
            # this includes the first point
            for n in range(step):
                # distance in each axis
                position_list[n][0] = self.wp_start.pose.position.x + n * self.dis_res * self.dir_vec.x 
                position_list[n][1] = self.wp_start.pose.position.y + n * self.dis_res * self.dir_vec.y
                position_list[n][2] = self.wp_start.pose.position.z + n * self.dis_res * self.dir_vec.z

                # angle
                roll = self.rpy_start[0] + n * roll_res
                pitch = self.rpy_start[1] + n * pitch_res
                yaw = self.rpy_start[2] + n * yaw_res
                quaternion_list[n] = tf.transformations.quaternion_from_euler(roll, pitch, yaw).tolist()

            # append the last point in the list
            position_list.append([self.wp_end.pose.position.x, \
                                  self.wp_end.pose.position.y, \
                                  self.wp_end.pose.position.z])

            quaternion_list.append([self.wp_end.pose.orientation.x, \
                                    self.wp_end.pose.orientation.y, \
                                    self.wp_end.pose.orientation.z, \
                                    self.wp_end.pose.orientation.w])
            
            for i in range(len(position_list)):
                self.path.poses.append(PoseStamped(header=Header(stamp=rospy.Time.now(), 
                                                                 frame_id=self.frame_id),
                                                   pose=Pose(position=Point(*position_list[i]), 
                                                             orientation=Quaternion(*quaternion_list[i]))
                                                    )
                                        )

def path_generator(wps, dis_res):
    assert(isinstance(wps, list) and isinstance(wps[0], PoseStamped)), "wps should be a list of waypoints"
    assert(len(wps) > 1), "the list lenght should be more than one"
    num_wp = len(wps)
    final_path = Path()
    final_path.header = wps[0].header
    for n in range(num_wp - 1):
        wp_interpolator = WaypointInterpolation(wps[n], wps[n+1], dis_res)
        temp_path = wp_interpolator.get_path()
        poses_list = temp_path.poses

        # prevent redundancy
        if (n != num_wp - 2):
            poses_list.pop()    # delete last element, otherwise redundant

        final_path.poses = final_path.poses + poses_list


    return final_path
    

if __name__ == '__main__':
    try:
        rospy.init_node("waypoint_interpolation", anonymous=True)

        path_pub = rospy.Publisher("/path", Path, queue_size=1)

        wp1 = PoseStamped()
        wp1.header.seq = 0
        wp1.header.stamp = rospy.Time.now()
        wp1.header.frame_id = "map"
        wp1.pose.position.x = 0.0
        wp1.pose.position.y = 0.0
        wp1.pose.position.z = 0.0
        wp1.pose.orientation.w = 1.0

        wp2 = PoseStamped()
        wp2.header.seq = 1
        wp2.header.stamp = rospy.Time.now()
        wp2.header.frame_id = "map"
        wp2.pose.position.x = 3.0
        wp2.pose.position.y = 0.0
        wp2.pose.position.z = 0.2
        wp2.pose.orientation.z = 1.0

        wp3 = PoseStamped()
        wp3.header.seq = 2
        wp3.header.stamp = rospy.Time.now()
        wp3.header.frame_id = "map"
        wp3.pose.position.x = 3.0
        wp3.pose.position.y = 3.0
        wp3.pose.position.z = 0.4
        wp3.pose.orientation.w = 1.0

        wp4 = PoseStamped()
        wp4.header.seq = 3
        wp4.header.stamp = rospy.Time.now()
        wp4.header.frame_id = "map"
        wp4.pose.position.x = 0.0
        wp4.pose.position.y = 3.0
        wp4.pose.position.z = 0.6
        wp4.pose.orientation.z = 1.0

        wp5 = PoseStamped()
        wp5.header.seq = 3
        wp5.header.stamp = rospy.Time.now()
        wp5.header.frame_id = "map"
        wp5.pose.position.x = 0.0
        wp5.pose.position.y = 0.0
        wp5.pose.position.z = 0.8
        wp5.pose.orientation.w = 1.0

        
        interpolator = WaypointInterpolation(wp1, wp2, 0.3)
        

        # print(interpolator.path)
        
        print("==========================================")

        wps = [wp1, wp2, wp3, wp4, wp5]
        # wps = [wp1, wp2]
        start_time = time.time()
        test = path_generator(wps, 0.8)
        print("Time used = {} seconds".format(time.time() - start_time))
        print(test)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            path_pub.publish(test)
            # print("finish publish")
            rate.sleep()
    
        # print("test")
        # print(a)
        # euler1 = tf.transformations.euler_from_quaternion((0, 0, 0, 1))
        # euler2 = tf.transformations.euler_from_quaternion((0, 0, 1, 0))
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.57, 'sxyz')
        # print(euler1, euler2)
        # print(quaternion)
        # waypoint_interpolation(wp1, wp2)
    except rospy.ROSInterruptException:
        pass