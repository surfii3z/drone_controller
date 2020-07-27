#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

// services
#include "drone_controller/UpdateRefPos.h"
#include "drone_controller/UpdateRefAlt.h"
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

double get_roll_from_quadternion(double qx, double qy, double qz, double qw)
{
    //ref: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
    return atan2(2.0 * (qw * qx + qy * qz), 1 - 2.0 *(qx * qx + qy * qy));
}

double get_pitch_from_quadternion(double qx, double qy, double qz, double qw)
{
    //ref: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
    double sinp = 2. * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        return  copysign(M_PI_2, sinp);
    else
        return  asin(sinp);
}

bool update_ref_pos(drone_controller::UpdateRefPos::Request  &req,
                    drone_controller::UpdateRefPos::Response &res)
{
    x_ref_w = req.x_ref_w;
    y_ref_w = req.y_ref_w;
    res.update_status = true;
    ROS_INFO("Update the reference position to %lf, %lf.", x_ref_w, y_ref_w);

    return true;
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

bool update_ref_alt(drone_controller::UpdateRefAlt::Request  &req,
                    drone_controller::UpdateRefAlt::Response &res)
{
    alt_ref_w = req.alt_ref_w;
    res.update_status = true;
    ROS_INFO("Update the reference altitude to %lf.", alt_ref_w);

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
    ROS_INFO("Set the reference pose to (x, y, alt, yaw) = (%lf, %lf, %lf, %lf).", x_ref_w, y_ref_w, alt_ref_w, yaw_ref_w);
    reference_is_not_set = false;

    return true;
}

void stateCallback(const geometry_msgs::PoseStamped& current_pose_read)
{
    /*
        read current drone pose w.r.t. world frame
    */
    time_stamp = current_pose_read.header.stamp;

    x_cur_w = current_pose_read.pose.position.x;
    y_cur_w = current_pose_read.pose.position.y;
    alt_cur_w = current_pose_read.pose.position.z;

    qx_cur_w = current_pose_read.pose.orientation.x;
    qy_cur_w = current_pose_read.pose.orientation.y;
    qz_cur_w = current_pose_read.pose.orientation.z;
    qw_cur_w = current_pose_read.pose.orientation.w;
}

void uxCallback(const std_msgs::Float64& current_ux_w)
{
    ux_cur_w = current_ux_w.data;
}

void uyCallback(const std_msgs::Float64& current_uy_w)
{
    uy_cur_w = current_uy_w.data;
}

void uzCallback(const std_msgs::Float64& current_uz_w)
{
    uz_cur_w = current_uz_w.data;
}

void uyawCallback(const std_msgs::Float64& current_uyaw_w)
{
    uyaw_cur_w = current_uyaw_w.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_interface");
    ros::NodeHandle ctrl_interface_node;

    while (ros::ok() && ros::Time(0) == ros::Time::now())
    {
      ROS_INFO("controller_interface spinning waiting for time to become non-zero");
      sleep(1);
    }

        
    ROS_INFO("controller_interface node has started");

    // Advertise position error to PID controller, and the zero_setpoint

    // to go to the reference point in X axis (drone heading), it needs to adjust pitch
    std_msgs::Float64 err_x_b_msg;
    ros::Publisher err_x_b_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/err_x_b", 1);
    // to go to the reference point in Y axis , it needs to adjust roll
    std_msgs::Float64 err_y_b_msg;
    ros::Publisher err_y_b_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/err_y_b", 1);

    std_msgs::Float64 err_yaw_b_msg;
    ros::Publisher err_yaw_b_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/err_yaw_b", 1);

    std_msgs::Float64 err_alt_b_msg;
    ros::Publisher err_alt_b_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/err_alt_b", 1);

    std_msgs::Float64 zero_setpoint_msg;
    zero_setpoint_msg.data = 0.0;
    ros::Publisher zero_setpoint_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/pid_zero_setpoint", 1);

    geometry_msgs::Twist cmd_vel_msg;
    ros::Publisher cmd_vel_pub = ctrl_interface_node.advertise<geometry_msgs::Twist>("/tello/cmd_vel", 1);

    // Subscribe to position reference
    ros::Subscriber state_w_sub = ctrl_interface_node.subscribe("/state_w", 1, stateCallback);
    
    ros::Subscriber ux_w_sub = ctrl_interface_node.subscribe("/pid_roll/control_effort", 1, uxCallback);
    ros::Subscriber uy_w_sub = ctrl_interface_node.subscribe("/pid_pitch/control_effort", 1, uyCallback);
    ros::Subscriber uz_w_sub = ctrl_interface_node.subscribe("/pid_thrust/control_effort", 1, uzCallback);
    ros::Subscriber uyaw_w_sub = ctrl_interface_node.subscribe("/pid_yaw/control_effort", 1, uyawCallback);

    // Advertise service to update position reference
    ros::ServiceServer update_ref_pos_srv = ctrl_interface_node.advertiseService("/update_ref_pos", update_ref_pos);
    ros::ServiceServer update_ref_alt_srv = ctrl_interface_node.advertiseService("/update_ref_alt", update_ref_alt);
    ros::ServiceServer set_ref_pose_srv = ctrl_interface_node.advertiseService("/set_ref_pose", set_ref_pose);
    ros::ServiceServer movr_drone_srv = ctrl_interface_node.advertiseService("/move_drone_w", move_drone_w);

    double hz = 100.0;

    int loop_counter = 0;
    ros::Rate loop_rate(hz);  // Control rate in Hz

    double err_x_ref_w = 0;
    double err_y_ref_w = 0;

    ROS_INFO("Please specify the reference point using rosservice");
    while(reference_is_not_set)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {
        // ROS_INFO("REF: %lf %lf", x_ref_w, y_ref_w);
        
        yaw_cur_w = get_yaw_from_quadternion(qx_cur_w, qy_cur_w, qz_cur_w, qw_cur_w);


        err_x_ref_w = x_ref_w - x_cur_w;
        err_y_ref_w = y_ref_w - y_cur_w;

        err_x_b_msg.data = err_x_ref_w * cos(yaw_cur_w) + err_y_ref_w * sin(yaw_cur_w); // check sign
        err_y_b_msg.data = -err_x_ref_w * sin(yaw_cur_w) + err_y_ref_w * cos(yaw_cur_w); // check sign

        err_alt_b_msg.data = alt_ref_w - alt_cur_w; 

        err_x_b_pub.publish(err_x_b_msg);       // the PID controller outputs body velocity command
        err_y_b_pub.publish(err_y_b_msg);       // the PID controller outputs body velocity command
        err_alt_b_pub.publish(err_alt_b_msg);   // the PID controller outputs body velocity command

        err_yaw_b_msg.data = yaw_ref_w - yaw_cur_w;
        err_yaw_b_pub.publish(err_yaw_b_msg);   // the PID controller outputs body angular velocity command

        zero_setpoint_pub.publish(zero_setpoint_msg);

        cmd_vel_msg.linear.x = ux_cur_w * cos(yaw_cur_w) + uy_cur_w * sin(yaw_cur_w);
        cmd_vel_msg.linear.y = ux_cur_w * cos(yaw_cur_w) + uy_cur_w * sin(yaw_cur_w);
        cmd_vel_msg.linear.z = uz_cur_w;
        cmd_vel_msg.angular.z = uyaw_cur_w;

        cmd_vel_pub.publish(cmd_vel_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}