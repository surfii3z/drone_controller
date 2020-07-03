#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include "drone_controller/UpdateRefPos.h"


namespace position_controller_interface
{
    static double x_ref = 0.0;
    static double y_ref = 0.0;

    static double x_cur = 0.0;
    static double y_cur = 0.0;
    static double yaw_cur = 0.0;
}
using namespace position_controller_interface;


bool update_ref_pos(drone_controller::UpdateRefPos::Request  &req,
                    drone_controller::UpdateRefPos::Response &res)
{
    x_ref = req.x_ref_w;
    y_ref = req.y_ref_w;
    res.update_status = true;
    ROS_INFO("Update the reference position to %lf, %lf.", x_ref, y_ref);

    return true;
}


// void refPosCallback(const geometry_msgs::Pose2D& reference_position_input)
// {
//     x_ref = reference_position_input.x;
//     y_ref = reference_position_input.y;
//     ROS_INFO("Update the reference position to %lf, %lf.", x_ref, y_ref);
//     if (reference_position_input.theta != 0.0)
//     {
//         ROS_WARN("This controller only takes reference position (x,y) as the input. Theta will not be used.");
//     }
// }

void curPosCallback(const geometry_msgs::Pose2D& current_pose_read)
{
    /*
        read drone current position
    */
    x_cur = current_pose_read.x;
    y_cur = current_pose_read.y;
    yaw_cur = current_pose_read.theta;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_controller_interface");
    ros::NodeHandle pos_ctrl_node;

    while (ros::ok() && ros::Time(0) == ros::Time::now())
    {
      ROS_INFO("position_controller_interface spinning waiting for time to become non-zero");
      sleep(1);
    }

    ROS_INFO("position controller node has started");

    // Advertise position error to PID controller, and the zero_setpoint

    // err in x axis can be reduce using pitch
    std_msgs::Float64 err_x_b;
    ros::Publisher err_x_b_pub = pos_ctrl_node.advertise<std_msgs::Float64>("/pid_pitch/input", 1);

    // err in y axis can be reduce using pitch
    std_msgs::Float64 err_y_b;
    ros::Publisher err_y_b_pub = pos_ctrl_node.advertise<std_msgs::Float64>("/pid_roll/input", 1);

    // the position error should go to zero, send this set point to PID that controll roll and pitch
    std_msgs::Float64 zero_setpoint;
    zero_setpoint.data = 0.0;
    ros::Publisher zero_setpoint_pub = pos_ctrl_node.advertise<std_msgs::Float64>("pid_zero_setpoint", 1);

    // Subscribe to position reference
    // ros::Subscriber ref_pos_w_sub = pos_ctrl_node.subscribe("ref_pos_w", 1, refPosCallback);
    ros::Subscriber cur_pos_w_sub = pos_ctrl_node.subscribe("cur_pos_w", 1, curPosCallback);

    // Advertise service to update position reference
    ros::ServiceServer update_ref_pos_srv = pos_ctrl_node.advertiseService("/update_ref_pos", update_ref_pos);

    double hz = 100.0;

    int loop_counter = 0;
    ros::Rate loop_rate(hz);  // Control rate in Hz

    double err_x_w = 0.0;
    double err_y_w = 0.0;

    while (ros::ok())
    {
        // ROS_INFO("REF: %lf, %lf", x_ref, y_ref);
        ros::spinOnce();

        err_x_w = x_ref - x_cur;
        err_y_w = y_ref - y_cur;

        err_x_b.data = err_x_w * cos(yaw_cur) + err_y_w * sin(yaw_cur);
        err_x_b_pub.publish(err_x_b);

        err_y_b.data = -err_x_w * sin(yaw_cur) + err_y_w * cos(yaw_cur);
        err_y_b_pub.publish(err_y_b);

        zero_setpoint_pub.publish(zero_setpoint);

        loop_rate.sleep();
    }

}