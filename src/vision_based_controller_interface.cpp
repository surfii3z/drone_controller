#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include "darknet_ros_msgs/ObjectCount.h"

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>


// services
#include "drone_controller/MoveDroneW.h"
#include "drone_controller/SetRefPose.h"

namespace controller_interface
{
    // references
    static double x_ref_w = 0.0;
    static double y_ref_w = 0.0;
    static double alt_ref_w = 0.0;
    static double yaw_ref_w = 0.0;

    // current states
    static double x_cur_w = 0.0;
    static double y_cur_w = 0.0;  
    static double alt_cur_w = 0.0;

    // control effort
    static double ux_cur_w = 0.0;
    static double uy_cur_w = 0.0;  
    static double uz_cur_w = 0.0;
    static double uyaw_cur_w = 0.0;

    static double qx_cur_w = 0.0;
    static double qy_cur_w = 0.0;
    static double qz_cur_w = 0.0;
    static double qw_cur_w = 1.0;
    
    static double roll_cur_w = 0.0;
    static double pitch_cur_w = 0.0;
    static double yaw_cur_w = 0.0;
    

    static ros::Time time_stamp;

    // for triggering the controller
    bool reference_is_not_set = true;
}
using namespace controller_interface;

double get_yaw_from_quadternion(double qx, double qy, double qz, double qw)
{
    //ref: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
    // return atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy* qy - qz * qz);
    return atan2(2.0 * (qw * qz + qx * qy), 1 - 2.0 *(qy * qy + qz * qz));
}

bool move_drone_w(drone_controller::MoveDroneW::Request  &req,
                  drone_controller::MoveDroneW::Response &res)
{
    x_ref_w += req.dx;
    y_ref_w += req.dy;
    alt_ref_w += req.dalt;
    yaw_ref_w += req.dyaw;
    res.command_status = true;
    ROS_INFO("Move drone with offset (dx, dy, dalt, dyaw) = (%lf, %lf, %lf, %lf).", req.dx, req.dy, req.dalt, req.dyaw);

    return true;
}

bool set_ref_pose(drone_controller::SetRefPose::Request  &req,
                  drone_controller::SetRefPose::Response &res)
{
    x_ref_w = req.x_ref_w;
    y_ref_w = req.y_ref_w;
    alt_ref_w = req.alt_ref_w;
    yaw_ref_w = req.yaw_ref_w;
    res.update_status = true;
    ROS_INFO("Set the reference pose to (x, y, alt, yaw) = (%lf, %lf, %lf, %lf).\n", x_ref_w, y_ref_w, alt_ref_w, yaw_ref_w);
    reference_is_not_set = false;

    return true;
}
// void stateCallback(const geometry_msgs::PoseStamped& current_pose_read)

void countBBoxCallback(const darknet_ros_msgs::ObjectCount& msg)
{
    printf("%d\n", msg.count);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_based_controller_interface");
    ros::NodeHandle vb_ctrl_interface_node;

    while (ros::ok() && ros::Time(0) == ros::Time::now())
    {
      ROS_INFO("vision_based_controller_interface spinning waiting for time to become non-zero");
      sleep(1);
    }

        
    ROS_INFO("vision_based_controller_interface node has started");

    // Advertise position error to PID controller, and the zero_setpoint
    std_msgs::Float64 err_x_b_msg;
    ros::Publisher err_x_b_pub = vb_ctrl_interface_node.advertise<std_msgs::Float64>("/err_x_img", 1);

    std_msgs::Float64 err_y_b_msg;
    ros::Publisher err_y_b_pub = vb_ctrl_interface_node.advertise<std_msgs::Float64>("/err_y_img", 1);

    std_msgs::Float64 err_yaw_b_msg;
    ros::Publisher err_yaw_b_pub = vb_ctrl_interface_node.advertise<std_msgs::Float64>("/err_yaw_img", 1);

    std_msgs::Float64 zero_setpoint_msg;
    zero_setpoint_msg.data = 0.0;
    ros::Publisher zero_setpoint_pub = vb_ctrl_interface_node.advertise<std_msgs::Float64>("/pid_zero_setpoint", 1);

    // Subscribe to bbox msgs from darknet_ros
    // ros::Subscriber bbox_sub = vb_ctrl_interface_node.subscribe("/darknet_ros/bounding_boxes", 1, bboxCallback);
    ros::Subscriber count_bbox_sub = vb_ctrl_interface_node.subscribe("/darknet_ros/found_object", 1, countBBoxCallback);

    double hz = 100.0;

    int loop_counter = 0;
    ros::Rate loop_rate(hz);  // Control rate in Hz

    double err_x_ref_w = 0;
    double err_y_ref_w = 0;

    while (ros::ok())
    {
        // ROS_INFO("(x, y, z, yaw): %.03lf %.03lf %.03lf %.03lf", x_cur_w, y_cur_w, alt_cur_w, yaw_cur_w);
        yaw_cur_w = get_yaw_from_quadternion(qx_cur_w, qy_cur_w, qz_cur_w, qw_cur_w);


        err_x_ref_w = x_ref_w - x_cur_w;
        err_y_ref_w = y_ref_w - y_cur_w;

        // err_x_b_msg.data = err_x_ref_w * cos(yaw_cur_w) + err_y_ref_w * sin(yaw_cur_w); // check sign
        // err_y_b_msg.data = -err_x_ref_w * sin(yaw_cur_w) + err_y_ref_w * cos(yaw_cur_w); // check sign
        // err_alt_b_msg.data = alt_ref_w - alt_cur_w; 
        // err_yaw_b_msg.data = yaw_ref_w - yaw_cur_w;

        // torelance error
        // if (abs(err_x_b_msg.data) < 0.03)
            // err_x_b_msg.data = 0.0;
        // if (abs(err_y_b_msg.data) < 0.03)
            // err_y_b_msg.data = 0.0;
        // if (abs(err_alt_b_msg.data) < 0.03)
            // err_alt_b_msg.data = 0.0;
        // if (abs(err_yaw_b_msg.data) < 0.1) // 5 degrees torelance
            // err_yaw_b_msg.data = 0.0;

        // err_x_b_pub.publish(err_x_b_msg);       // the PID controller outputs body velocity command
        // err_y_b_pub.publish(err_y_b_msg);       // the PID controller outputs body velocity command
        // err_alt_b_pub.publish(err_alt_b_msg);   // the PID controller outputs body velocity command
        // err_yaw_b_pub.publish(err_yaw_b_msg);   // the PID controller outputs body angular velocity command

        // ROS_INFO("(ex, ey, ez, eyaw): %.03lf %.03lf %.03lf %.03lf", 
                    // err_x_ref_w, err_y_ref_w, err_alt_b_msg.data, err_yaw_b_msg.data);

        zero_setpoint_pub.publish(zero_setpoint_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}