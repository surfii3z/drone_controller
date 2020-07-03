#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include "drone_controller/UpdateRefAlt.h"

namespace altitude_controller_interface
{
    static double alt_ref_w = 0.0;

    static double alt_cur_w = 0.0;
}

using namespace altitude_controller_interface;

bool update_ref_alt(drone_controller::UpdateRefAlt::Request  &req,
                    drone_controller::UpdateRefAlt::Response &res)
{
    alt_ref_w = req.alt_ref_w;
    res.update_status = true;
    ROS_INFO("Update the reference altitude to %lf.", alt_ref_w);

    return true;
}

void curAltCallback(const std_msgs::Float64& current_alt_read)
{
    /*
        read drone current altitude
    */
    alt_cur_w = current_alt_read.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "altitude_controller_interface");
    ros::NodeHandle alt_ctrl_node;

    while (ros::ok() && ros::Time(0) == ros::Time::now())
    {
      ROS_INFO("altitude_controller_interface spinning waiting for time to become non-zero");
      sleep(1);
    }

    ROS_INFO("altitude controller node has started");

    // Advertise altitude error to PID controller, and the zero_setpoint

    // err in x axis can be reduce using pitch
    std_msgs::Float64 err_alt_b;
    ros::Publisher err_alt_b_pub = alt_ctrl_node.advertise<std_msgs::Float64>("/pid_thrust/input", 1);

    // the altitude error should go to zero, send this set point to PID that controll roll and pitch
    std_msgs::Float64 zero_setpoint;
    zero_setpoint.data = 0.0;
    ros::Publisher zero_setpoint_pub = alt_ctrl_node.advertise<std_msgs::Float64>("pid_zero_setpoint", 1);

    // Subscribe to altitude reference
    ros::Subscriber cur_alt_w_sub = alt_ctrl_node.subscribe("cur_alt_w", 1, curAltCallback);

    // Advertise service to update altitude reference
    ros::ServiceServer update_ref_alt_srv = alt_ctrl_node.advertiseService("/update_ref_alt", update_ref_alt);

    double hz = 100.0;

    int loop_counter = 0;
    ros::Rate loop_rate(hz);  // Control rate in Hz

    double err_alt_w = 0.0;

    while (ros::ok())
    {
        // ROS_INFO("REF: %lf, %lf", x_ref, y_ref);
        ros::spinOnce();

        err_alt_b.data = alt_ref_w - alt_cur_w;
        err_alt_b_pub.publish(err_alt_b);

        zero_setpoint_pub.publish(zero_setpoint);

        loop_rate.sleep();
    }

}