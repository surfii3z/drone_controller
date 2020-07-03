#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

namespace joy_controller
{
    static double axis1 = 0.0;
    static double axis2 = 0.0;
    static double axis3 = 0.0;
    static double axis4 = 0.0;

    static double cmd_roll = 0.0;
    static double cmd_pitch = 0.0;
    static double cmd_yaw = 0.0;
    static double cmd_thrust = 0.0;
}
using namespace joy_controller;

void cmdRollCallback(const std_msgs::Float64& cmd_roll_read)
{
    cmd_roll = cmd_roll_read.data;
}

void cmdPitchCallback(const std_msgs::Float64& cmd_pitch_read)
{
    cmd_pitch = cmd_pitch_read.data;
}

void cmdThrustCallback(const std_msgs::Float64& cmd_thrust_read)
{
    cmd_thrust = cmd_thrust_read.data;
}

void cmdYawCallback(const std_msgs::Float64& cmd_yaw_read)
{
    cmd_yaw = cmd_yaw_read.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_interface");
    ros::NodeHandle joy_controller_node;

    while (ros::ok() && ros::Time(0) == ros::Time::now())
    {
      ROS_INFO("controller_interface spinning waiting for time to become non-zero");
      sleep(1);
    }

        
    ROS_INFO("controller_interface node has started");

    // Advertise
    sensor_msgs::Joy joy_ctrl_cmd_msg;
    ros::Publisher joy_ctrl_cmd_pub = joy_controller_node.advertise<sensor_msgs::Joy>("/joy_ctrl_cmd", 1);
    // ros::Publisher joy_ctrl_cmd_pub = joy_controller_node.advertise<sensor_msgs::Joy>("/control_nodes/joy", 1);

    // Subscribe to control command
    ros::Subscriber cmd_roll_sub = joy_controller_node.subscribe("/roll_control/control_effort", 1, cmdRollCallback);
    ros::Subscriber cmd_pitch_sub = joy_controller_node.subscribe("/pitch_control/control_effort", 1, cmdPitchCallback);
    // ros::Subscriber cmd_roll_sub = joy_controller_node.subscribe("/pid_roll/control_effort", 1, cmdRollCallback);
    // ros::Subscriber cmd_pitch_sub = joy_controller_node.subscribe("/pid_pitch/control_effort", 1, cmdPitchCallback);
    ros::Subscriber cmd_thrust_sub = joy_controller_node.subscribe("/pid_thrust/control_effort", 1, cmdThrustCallback);
    ros::Subscriber cmd_yaw_sub = joy_controller_node.subscribe("/pid_yaw/control_effort", 1, cmdYawCallback);
    

    // Advertise service to update position reference


    double hz = 100.0;

    int loop_counter = 0;
    ros::Rate loop_rate(hz);  // Control rate in Hz

    while (ros::ok())
    {
        joy_ctrl_cmd_msg.axes = {(float)cmd_yaw, (float)cmd_thrust, 0.0, (float)cmd_roll, (float)cmd_pitch, 0.0, 0.0, 0.0};    // 3:roll, 4:pitch
        joy_ctrl_cmd_msg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        joy_ctrl_cmd_pub.publish(joy_ctrl_cmd_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

}