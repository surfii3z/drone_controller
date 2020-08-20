#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#define MAX_KEEP 200

class path_visualizer
{
private:
    ros::NodeHandle nh;
    ros::Publisher optitrack_path_pub;
    ros::Publisher orb_path_pub;

    ros::Subscriber optitrack_pose_sub;
    ros::Subscriber orb_pose_sub;

    nav_msgs::Path optitrack_path_msg;
    nav_msgs::Path orb_path_msg;
    

public:
    void cb_optitrack_pose(const geometry_msgs::PoseStamped msg)
    {
        optitrack_path_msg.header.frame_id = "map";

        if (optitrack_path_msg.poses.size() > MAX_KEEP)
            optitrack_path_msg.poses.erase(optitrack_path_msg.poses.begin());

        optitrack_path_msg.poses.push_back(msg);
        optitrack_path_pub.publish(optitrack_path_msg);
    }

    void cb_orb_pose(const geometry_msgs::PoseStamped msg)
    {
        orb_path_msg.header.frame_id = "map";
        if (orb_path_msg.poses.size() > MAX_KEEP)
            orb_path_msg.poses.erase(orb_path_msg.poses.begin());
        orb_path_msg.poses.push_back(msg);
        orb_path_pub.publish(orb_path_msg);
    }

    path_visualizer()
    {
        optitrack_path_pub = nh.advertise<nav_msgs::Path>("/optitrack_path", 1);
        optitrack_pose_sub = nh.subscribe("/vrpn_client_node/Tello_jed/pose", 1, &path_visualizer::cb_optitrack_pose, this);

        orb_path_pub = nh.advertise<nav_msgs::Path>("/orb_path", 1);
        orb_pose_sub = nh.subscribe("/scaled_orb_pose", 1, &path_visualizer::cb_orb_pose, this);
    }
    
};




////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
    // Initialize ROS node
    ros::init(argc, argv, "pose_to_path_node");
    ros::start();

    path_visualizer node;
    ROS_INFO("pose_to_path_node is initialized.");
    ros::spin();
    ros::shutdown();

  return 0;
}