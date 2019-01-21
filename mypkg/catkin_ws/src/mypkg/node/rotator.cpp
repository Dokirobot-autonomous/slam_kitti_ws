//
// Created by ohashi on 19/01/19.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <rosbag/bag.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Empty.h>

//#define USE_SLAM_NAMES "SPTAM,LIMO,ORB,NDT"


// -----------------------------------
//	Main Function
// -----------------------------------
int main(int argc, char **argv) {

    // debug mode
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Initialize ROS
    ros::init(argc, argv, "rotator");
    ros::NodeHandle nh;

    rosbag::Bag bag;
    bag.open("test.bag",rosbag::bagmode::Write);




    ros::spin();


    return (0);
}
