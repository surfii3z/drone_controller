#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "drone_controller/UpdateRefPos.h"
#include "drone_controller/UpdateRefAlt.h"


namespace controller_interface
{
    static double x_ref_w = 0.0;
    static double y_ref_w = 0.0;
    static double alt_ref_w = 0.0;
    static double yaw_ref_w = 0.0;


    static double x_cur_w = 0.0;
    static double y_cur_w = 0.0;  
    static double alt_cur_w = 0.0;

    static double qx_cur_w = 0.0;
    static double qy_cur_w = 0.0;
    static double qz_cur_w = 0.0;
    static double qw_cur_w = 0.0;
    
    static double yaw_cur_w = 0.0;

    static ros::Time time_stamp;
}
using namespace controller_interface;

double get_yaw_from_quadternion(double qx, double qy, double qz, double qw)
{
    //ref: https://robotics.stackexchange.com/questions/16471/get-yaw-from-quaternion
    return atan2(2.0 * (qw * qz + qx * qy), qw * qw + qx * qx - qy* qy - qz * qz);
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

bool update_ref_alt(drone_controller::UpdateRefAlt::Request  &req,
                    drone_controller::UpdateRefAlt::Response &res)
{
    alt_ref_w = req.alt_ref_w;
    res.update_status = true;
    ROS_INFO("Update the reference altitude to %lf.", alt_ref_w);

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
    std_msgs::Float64 x_ref_b_msg;
    ros::Publisher x_ref_b_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/pid_pitch/setpoint", 1);
    // to go to the reference point in Y axis , it needs to adjust roll
    std_msgs::Float64 y_ref_b_msg;
    ros::Publisher y_ref_b_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/pid_roll/setpoint", 1);

    std_msgs::Float64 yaw_ref_w_msg;
    ros::Publisher yaw_ref_w_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/pid_yaw/setpoint", 1);

    std_msgs::Float64 alt_ref_w_msg;
    ros::Publisher alt_ref_w_pub = ctrl_interface_node.advertise<std_msgs::Float64>("/pid_thrust/setpoint", 1);

    // Subscribe to position reference
    ros::Subscriber state_w_sub = ctrl_interface_node.subscribe("/state_w", 1, stateCallback);
    // ros::Subscriber cur_pos_w_sub = ctrl_interface_node.subscribe("cur_pos_w", 1, curPosCallback);

    // Advertise service to update position reference
    ros::ServiceServer update_ref_pos_srv = ctrl_interface_node.advertiseService("/update_ref_pos", update_ref_pos);
    ros::ServiceServer update_ref_alt_srv = ctrl_interface_node.advertiseService("/update_ref_alt", update_ref_alt);

    double hz = 100.0;

    int loop_counter = 0;
    ros::Rate loop_rate(hz);  // Control rate in Hz

    while (ros::ok())
    {
        // ROS_INFO("REF: %lf, %lf", x_ref, y_ref);
        ros::spinOnce();

        yaw_cur_w = get_yaw_from_quadternion(qx_cur_w, qy_cur_w, qz_cur_w, qw_cur_w);
        x_ref_b_msg.data = x_ref_w * cos(yaw_cur_w) + y_ref_w * sin(yaw_cur_w);
        x_ref_b_pub.publish(x_ref_b_msg);

        y_ref_b_msg.data = x_ref_w * (-sin(yaw_cur_w)) + y_ref_w * cos(yaw_cur_w);
        y_ref_b_pub.publish(y_ref_b_msg);

        alt_ref_w_msg.data = alt_ref_w;
        alt_ref_w_pub.publish(alt_ref_w_msg);

        yaw_ref_w_msg.data = yaw_ref_w;
        yaw_ref_w_pub.publish(yaw_ref_w_msg);

        // zero_setpoint_pub.publish(zero_setpoint);

        loop_rate.sleep();
    }

}