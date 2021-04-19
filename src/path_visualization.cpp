#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>

#define MAX_KEEP 500

class path_visualizer
{
private:
    ros::NodeHandle nh;
    ros::Publisher optitrack_path_pub;
    ros::Publisher orb_path_pub;
    ros::Publisher ekf_path_pub;

    ros::Subscriber optitrack_pose_sub;
    ros::Subscriber orb_pose_sub;
    ros::Subscriber ekf_pose_sub;

    nav_msgs::Path optitrack_path_msg;
    nav_msgs::Path orb_path_msg;
    nav_msgs::Path ekf_path_msg;

    geometry_msgs::PoseStamped orb_pose_stamped_msg;
    geometry_msgs::PoseStamped ekf_pose_stamped_msg;
    
    

public:
    // void cb_optitrack_pose(const geometry_msgs::PoseWithCovarianceStamped msg)
    // {
    //     optitrack_path_msg.header.frame_id = "map";

    //     // if (optitrack_path_msg.poses.size() > MAX_KEEP)
    //     // optitrack_path_msg.poses.erase(optitrack_path_msg.poses.begin());

    //     optitrack_path_msg.poses.push_back(orb_pose_stamped_msg);
    //     optitrack_path_pub.publish(optitrack_path_msg);
    // }

    void cb_orb_pose(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
        if (isnan(msg.pose.pose.position.x))
        {
            ROS_WARN("Pose is nan");
            return;
        }

        orb_pose_stamped_msg.header = msg.header;
        orb_pose_stamped_msg.pose = msg.pose.pose; 
        orb_path_msg.header.frame_id = "odom";
        // if (orb_path_msg.poses.size() > MAX_KEEP)
            // orb_path_msg.poses.erase(orb_path_msg.poses.begin());

        
        
        orb_path_msg.poses.push_back(orb_pose_stamped_msg);
        orb_path_pub.publish(orb_path_msg);
    }
    
    void cb_tf_pose(const tf2_msgs::TFMessage msg)
    {
        ekf_pose_stamped_msg.header = msg.transforms[0].header;
        ekf_pose_stamped_msg.pose.position.x = msg.transforms[0].transform.translation.x;
        ekf_pose_stamped_msg.pose.position.y = msg.transforms[0].transform.translation.y;
        ekf_pose_stamped_msg.pose.position.z = msg.transforms[0].transform.translation.z;
        ekf_pose_stamped_msg.pose.orientation = msg.transforms[0].transform.rotation;

        ekf_path_msg.header.frame_id = "odom";
        ekf_path_msg.poses.push_back(ekf_pose_stamped_msg);
        ekf_path_pub.publish(ekf_path_msg);
    }

    path_visualizer()
    {
        // optitrack_path_pub = nh.advertise<nav_msgs::Path>("/optitrack_path", 1);
        // optitrack_pose_sub = nh.subscribe("/vrpn_client_node/Tello_orb_jed/pose", 1, &path_visualizer::cb_optitrack_pose, this);

        ekf_path_pub = nh.advertise<nav_msgs::Path>("/ekf_path", 1);
        ekf_pose_sub = nh.subscribe("/tf", 1, &path_visualizer::cb_tf_pose, this);

        orb_path_pub = nh.advertise<nav_msgs::Path>("/slam_path", 1);
        orb_pose_sub = nh.subscribe("/orb_slam/pose_with_cov", 1, &path_visualizer::cb_orb_pose, this);
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