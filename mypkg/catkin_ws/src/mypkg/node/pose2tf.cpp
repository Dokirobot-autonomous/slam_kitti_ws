#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>

#include "pose_cov_ops.h"

#include "slam_fusion/PathWithCovariance.h"

//#define USE_SLAM_NAMES "SPTAM,LIMO,ORB,NDT"


c

// -----------------------------------
//	Main Function
// -----------------------------------
int main(int argc, char **argv) {

    // debug mode
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Initialize ROS
    ros::init(argc, argv, "slam_fusion");
    ros::NodeHandle nh;

    SlamFusion fusion;
    fusion.initialize(nh);

    std::vector<std::string> topic_names = fusion.pose_topic_names;

    ros::Subscriber sub_sptam_pose,sub_limo_pose,sub_orb_pose,sub_ndt_pose,sub_fin;

    if(fusion.is_use_slam[0]){
        sub_sptam_pose = nh.subscribe(POSE_NAME_SPTAM, 1, &SlamFusion::callback_sptam, &fusion);
    }
    if(fusion.is_use_slam[1]){
        sub_limo_pose = nh.subscribe(POSE_NAME_LIMO, 1, &SlamFusion::callback_limo_path, &fusion);
    }
    if(fusion.is_use_slam[2]){
        sub_orb_pose = nh.subscribe(POSE_NAME_ORB, 1, &SlamFusion::callback_orb_pose, &fusion);
    }
    if(fusion.is_use_slam[3]){
        sub_ndt_pose = nh.subscribe(POSE_NAME_NDT, 1, &SlamFusion::callback_ndt_pose, &fusion);
    }
    if(fusion.is_use_slam[4]){
        sub_ndt_pose = nh.subscribe(POSE_NAME_LOAM, 1, &SlamFusion::callback_loam_pose, &fusion);
    }
    sub_fin=nh.subscribe("finish",1,&SlamFusion::callback_finish,&fusion);
    ros::Subscriber sub_groundtruth = nh.subscribe(POSE_NAME_GROUNDTRUTH, 1, &SlamFusion::callback_groundtruth, &fusion);


    ros::spin();


    return (0);
}
