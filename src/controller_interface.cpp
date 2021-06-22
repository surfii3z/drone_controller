/* NOTE: /openvslam/camera_pose coordinate is NWU: N is +x, W is +y, and U is +z
 *       /tello/cmd_vel coordinate         is ENU: E is +x, N is +y, and U is +z
 */

#include <ros/ros.h>
#include <math.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

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

void stateCallback(const geometry_msgs::PoseStamped& current_pose_read)
{
    /*
        read current drone pose w.r.t. world frame
    */
    time_stamp = current_pose_read.header.stamp;

    x_cur_w = current_pose_read.pose.position.x;
    y_cur_w = current_pose_read.pose.position.y;
    alt_cur_w = current_pose_read.pose.position.z;
    // alt_cur_w = current_pose_read.pose.position.y;

    qx_cur_w = current_pose_read.pose.orientation.x;
    qy_cur_w = current_pose_read.pose.orientation.y;
    qz_cur_w = current_pose_read.pose.orientation.z;
    qw_cur_w = current_pose_read.pose.orientation.w;
}

// void stateCallback(const geometry_msgs::PoseWithCovarianceStamped& current_pose_read)
// {
//     /*
//         read current drone pose w.r.t. world frame
//     */
//     time_stamp = current_pose_read.header.stamp;

//     x_cur_w = current_pose_read.pose.pose.position.x;
//     y_cur_w = current_pose_read.pose.pose.position.y;
//     alt_cur_w = current_pose_read.pose.pose.position.z;
//     // alt_cur_w = current_pose_read.pose.position.y;

//     qx_cur_w = current_pose_read.pose.pose.orientation.x;
//     qy_cur_w = current_pose_read.pose.pose.orientation.y;
//     qz_cur_w = current_pose_read.pose.pose.orientation.z;
//     qw_cur_w = current_pose_read.pose.pose.orientation.w;
// }

// void stateTFCallback(const tf2_msgs::TFMessage msg)
// {
//     /*
//         read current drone pose w.r.t. world frame
//     */
//     time_stamp = msg.transforms[0].header.stamp;

//     x_cur_w = msg.transforms[0].transform.translation.x;
//     y_cur_w = msg.transforms[0].transform.translation.y;
//     alt_cur_w = msg.transforms[0].transform.translation.z;
//     // alt_cur_w = current_pose_read.pose.position.y;

//     qx_cur_w = msg.transforms[0].transform.rotation.x;
//     qy_cur_w = msg.transforms[0].transform.rotation.y;
//     qz_cur_w = msg.transforms[0].transform.rotation.z;
//     qw_cur_w = msg.transforms[0].transform.rotation.w;
// }

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

    // Subscribe to position reference
    // ros::Subscriber state_w_sub = ctrl_interface_node.subscribe("/openvslam/camera_pose", 1, stateCallback);
    ros::Subscriber state_w_sub = ctrl_interface_node.subscribe("/orb_slam/pose", 1, stateCallback);
    
    // ros::Subscriber state_w_sub = ctrl_interface_node.subscribe("/tf", 1, stateTFCallback);
    // ros::Subscriber state_w_sub = ctrl_interface_node.subscribe("/vrpn_client_node/Tello_jed/pose", 1, stateCallback);

    // Advertise service to update position reference
    ros::ServiceServer set_ref_pose_srv = ctrl_interface_node.advertiseService("/set_ref_pose", set_ref_pose);
    ros::ServiceServer move_drone_srv = ctrl_interface_node.advertiseService("/move_drone_w", move_drone_w);

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
        ROS_INFO("(x, y, z, yaw): %.03lf %.03lf %.03lf %.03lf", x_cur_w, y_cur_w, alt_cur_w, yaw_cur_w);
        yaw_cur_w = get_yaw_from_quadternion(qx_cur_w, qy_cur_w, qz_cur_w, qw_cur_w);

        /* openvslam pose coordinate is NWU
         * if the err_x_b is plus (target point is front) => pitch is plus 
         *                                                => drone moves forward
         * if the err_y_b is plus (target point is left)  => roll is plus  
         *                                                => drone moves right
         *                                                => therefore the sign should be the opposite
         */

        err_x_ref_w = x_ref_w - x_cur_w;
        err_y_ref_w = y_ref_w - y_cur_w;

        err_x_b_msg.data =  err_x_ref_w * cos(yaw_cur_w) + err_y_ref_w * sin(yaw_cur_w); // check sign
        err_y_b_msg.data = -err_x_ref_w * sin(yaw_cur_w) + err_y_ref_w * cos(yaw_cur_w); // check sign
        err_y_b_msg.data = err_y_b_msg.data * -1;


        err_alt_b_msg.data = alt_ref_w - alt_cur_w; 
        err_yaw_b_msg.data = yaw_ref_w - yaw_cur_w;

        // torelance error
        if (abs(err_x_b_msg.data) < 0.03)
            err_x_b_msg.data = 0.0;
        if (abs(err_y_b_msg.data) < 0.03)
            err_y_b_msg.data = 0.0;
        if (abs(err_alt_b_msg.data) < 0.03)
            err_alt_b_msg.data = 0.0;
        if (abs(err_yaw_b_msg.data) < 0.1) // 5 degrees torelance
            err_yaw_b_msg.data = 0.0;

        err_x_b_pub.publish(err_x_b_msg);       // the PID controller outputs body velocity command
        err_y_b_pub.publish(err_y_b_msg);       // the PID controller outputs body velocity command

        err_alt_b_pub.publish(err_alt_b_msg);   // the PID controller outputs body velocity command
        err_yaw_b_pub.publish(err_yaw_b_msg);   // the PID controller outputs body angular velocity command

        ROS_INFO("(ex, ey, ez, eyaw): %.03lf %.03lf %.03lf %.03lf", 
                    err_x_ref_w, err_y_ref_w, err_alt_b_msg.data, err_yaw_b_msg.data);

        zero_setpoint_pub.publish(zero_setpoint_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

}