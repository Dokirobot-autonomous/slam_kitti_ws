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
#include <tf/transform_listener.h>


#define POSE_TOPIC_NAMES "sptam/robot/pose,estimate/active_path,camera_pose,current_pose"
#define GROUNDTRUTH_TOPIC_NAME "/groundtruth_pose/pose"

class SlamFusion {

private:

    ros::NodeHandle nh;

    std::vector<nav_msgs::Path> paths;
    nav_msgs::Path groundtruth_path;

    std::vector<bool> get_latest_pose_;

    bool existAllPose();

    std::string pose_topic_names;

    ros::Publisher pub_groundtruth_path;
    tf::TransformListener listener;


public:

    // Functions

    void initialize(ros::NodeHandle _nh);

    void publishGroundtruthPath();

    void callback_sptam(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void callback_limo_path(const nav_msgs::Path::ConstPtr &msg);
    void callback_orb_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callback_ndt_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callback_groundtruth(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void fusion();

    std::vector<std::string> getPoseTopicNames();

};

void SlamFusion::initialize(ros::NodeHandle _nh) {

    ROS_INFO("Initialize slam_fusion");

    nh = _nh;
    paths.resize(4);
    get_latest_pose_ = std::vector<bool>(4, false);

    pub_groundtruth_path=nh.advertise<nav_msgs::Path>("/groundtruth_pose/path",10);

    if (!nh.getParam("pose_topic_names", pose_topic_names)) {
        pose_topic_names = POSE_TOPIC_NAMES;
    }

}

void SlamFusion::publishGroundtruthPath(){
    ROS_DEBUG_STREAM("Publish groundtruth path");
    pub_groundtruth_path.publish(groundtruth_path);
}

void SlamFusion::callback_sptam(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::string str = getPoseTopicNames()[0];
    ROS_DEBUG_STREAM("callback " << str);

    geometry_msgs::PoseStamped tmp;
    tmp.header = msg->header;
    tmp.pose = msg->pose.pose;
    paths[0].poses.push_back(tmp);

    get_latest_pose_[0] = true;

    /* Poseの確認 */
    geometry_msgs::PoseStamped pose_transformed;
    try {
        listener.transformPose("local_cs",tmp.header.stamp,tmp,msg->header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_DEBUG_STREAM("sptam local_cs pose \n"<<pose_transformed);

    if (existAllPose()) {
        fusion();
    }
}

void SlamFusion::callback_limo_path(const nav_msgs::Path::ConstPtr &msg) {

    std::string str = getPoseTopicNames()[1];
    ROS_DEBUG_STREAM("callback " << str);
    ROS_DEBUG_STREAM("size of pose in limo_path: " << msg->poses.size());

    paths[1] = *msg;

    /* Poseの確認 */
    geometry_msgs::PoseStamped pose_transformed;
    try {
        listener.transformPose("local_cs",msg->header.stamp,msg->poses.back(),msg->header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_DEBUG_STREAM("limo local_cs pose \n"<<pose_transformed);


    get_latest_pose_[1] = true;
    if (existAllPose()) {
        fusion();
    }
}

void SlamFusion::callback_orb_pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = getPoseTopicNames()[2];
    ROS_DEBUG_STREAM("callback " << str);

    paths[2].poses.push_back(*msg);

    /* Poseの確認 */
    geometry_msgs::PoseStamped pose_transformed;
    try {
        listener.transformPose("local_cs",msg->header.stamp,*msg,msg->header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_DEBUG_STREAM("orb local_cs pose \n"<<pose_transformed);

    get_latest_pose_[2] = true;
    if (existAllPose()) {
        fusion();
    }

}

void SlamFusion::callback_ndt_pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = getPoseTopicNames()[3];
    ROS_DEBUG_STREAM("callback " << str);

    paths[3].poses.push_back(*msg);

    /* Poseの確認 */
    geometry_msgs::PoseStamped pose_transformed;
    try {
        listener.transformPose("local_cs",msg->header.stamp,*msg,msg->header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_DEBUG_STREAM("ndt local_cs pose \n"<<pose_transformed);

    get_latest_pose_[2] = true;
    if (existAllPose()) {
        fusion();
    }

}



void SlamFusion::callback_groundtruth(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = GROUNDTRUTH_TOPIC_NAME;
    ROS_DEBUG_STREAM("callback " << str);

    groundtruth_path.poses.push_back(*msg);
    groundtruth_path.header=msg->header;

//    publishGroundtruthPath();

}

bool SlamFusion::existAllPose() {
    auto itr = std::find(get_latest_pose_.begin(), get_latest_pose_.end(), false);
    if (itr == get_latest_pose_.end()) {
        ROS_DEBUG_STREAM("Getting All Packages' Pose");
        get_latest_pose_ = std::vector<bool>(3, false);
        return true;
    } else {
        ROS_DEBUG_STREAM("Non exist: ");
        ROS_DEBUG_STREAM(pose_topic_names << get_latest_pose_[0] << get_latest_pose_[1] << get_latest_pose_[2]);
        return false;
    }

}

void SlamFusion::fusion() {
    ROS_INFO("Fuse Poses");


}


/*
 * PoseのTopic名をsplit
 */
std::vector<std::string> SlamFusion::getPoseTopicNames() {
    std::vector<std::string> out;
    std::string item;
    for (char ch: pose_topic_names) {
        if (ch == ',') {
            if (!item.empty())
                out.push_back(item);
            item.clear();
        } else {
            item += ch;
        }
    }
    if (!item.empty())
        out.push_back(item);

    return out;
}


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

    std::vector<std::string> topic_names = fusion.getPoseTopicNames();

    ros::Subscriber sub_sptam_pose = nh.subscribe(topic_names[0], 1, &SlamFusion::callback_sptam, &fusion);
    ros::Subscriber sub_limo_pose = nh.subscribe(topic_names[1], 1, &SlamFusion::callback_limo_path, &fusion);
    ros::Subscriber sub_orb_pose = nh.subscribe(topic_names[2], 1, &SlamFusion::callback_orb_pose, &fusion);
    ros::Subscriber sub_ndt_pose = nh.subscribe(topic_names[3], 1, &SlamFusion::callback_ndt_pose, &fusion);
    ros::Subscriber sub_groundtruth = nh.subscribe(GROUNDTRUTH_TOPIC_NAME, 1, &SlamFusion::callback_groundtruth, &fusion);

    ROS_INFO("hgoe");

    ros::spin();

    return (0);
}
