#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <rosbag/bag.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


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

bool USE_SPTAM=false;
bool USE_LIMO=false;
bool USE_ORB=false;
bool USE_NDT=false;
bool USE_LOAM=true;

#define POSE_NAME_SPTAM "sptam/robot/pose"
#define POSE_NAME_LIMO "/estimate/active_path_with_covariance"
#define POSE_NAME_ORB "/vehicle_pose_with_covariance"
#define POSE_NAME_NDT "/current_pose_with_covariance"
#define POSE_NAME_LOAM "/integrated_to_init"
#define POSE_NAME_GROUNDTRUTH "/groundtruth_pose/pose"

/*
 * PoseのTopic名をsplit
 */
std::vector<std::string> const split(const std::string &str){

    std::vector<std::string> out;

    std::string item;
    for (char ch: str) {
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

}

// 平均算出
double getMean(const std::vector<double> &v) {
    int size = v.size();
    double sum = 0;
    for (int i = 0; i < size; i++){
        sum += v[i];
    }
    return sum / size;
}

// 中央値
double getMedian(const std::vector<double> &v) {
    int size = v.size();
    std::vector<double> _v(v.size());
    std::copy(v.begin(), v.end(), back_inserter(_v));
    double tmp;
    for (int i = 0; i < size - 1; i++){
        for (int j = i + 1; j < size; j++) {
            if (_v[i] > _v[j]){
                tmp = _v[i];
                _v[i] = _v[j];
                _v[j] = tmp;
            }
        }
    }
    if (size % 2 == 1) {
        return _v[(size - 1) / 2];
    } else {
        return (_v[(size / 2) - 1] + _v[size / 2]) / 2;
    }
}

//
void quat2rpy(const tf::Quaternion &q,std::vector<double> *rpy){
    tf::Matrix3x3 m(q);
    double r,p,y;
    m.getRPY(r,p,y);
    *rpy={r,p,y};
}
void quat2rpy(const geometry_msgs::Quaternion &gq,std::vector<double> *rpy){

    tf::Quaternion q(gq.x,gq.y,gq.z,gq.w);
    quat2rpy(q,rpy);
}
void poseRpy2poseQuat(const std::vector<double> &pose_rpy,tf::Transform *transform){
    transform->setOrigin(tf::Vector3(pose_rpy[0], pose_rpy[1], pose_rpy[2]));
    tf::Quaternion quaternion;
    quaternion.setRPY(pose_rpy[3], pose_rpy[4], pose_rpy[5]);
    transform->setRotation(quaternion);
}
void poseRpy2poseQuat(const std::vector<double> &pose_rpy,geometry_msgs::Pose *pose_quat){
    tf::Transform tf;
    poseRpy2poseQuat(pose_rpy,&tf);
    pose_quat->position.x=tf.getOrigin().x();
    pose_quat->position.y=tf.getOrigin().y();
    pose_quat->position.z=tf.getOrigin().z();
    pose_quat->orientation.x=tf.getRotation().x();
    pose_quat->orientation.y=tf.getRotation().y();
    pose_quat->orientation.z=tf.getRotation().z();
    pose_quat->orientation.w=tf.getRotation().w();
}
void poseQuat2poseRpy(const tf::Transform &pose_quat,std::vector<double> *pose_rpy){

    std::vector<double> rpy;
    quat2rpy(pose_quat.getRotation(),&rpy);
    *pose_rpy={pose_quat.getOrigin().x(),pose_quat.getOrigin().y(),pose_quat.getOrigin().z(),
               rpy[0],rpy[1],rpy[2]};
}
void poseQuat2poseRpy(const geometry_msgs::Pose &pose_quat,std::vector<double> *pose_rpy) {

    std::vector<double> rpy;
    quat2rpy(pose_quat.orientation,&rpy);
    *pose_rpy={pose_quat.position.x,pose_quat.position.y,pose_quat.position.z,
               rpy[0],rpy[1],rpy[2]};

}

/***** 座標変換 *****/
void transform(const tf::TransformListener &listener,const geometry_msgs::PoseWithCovarianceStamped& origin,geometry_msgs::PoseWithCovarianceStamped *out){

    // Poseの変換
    geometry_msgs::PoseStamped pose_origin,pose_transformed;
    pose_origin.header=pose_transformed.header=origin.header;
    pose_origin.pose=origin.pose.pose;
    try {
        listener.transformPose("local_cs",pose_origin.header.stamp,pose_origin,pose_origin.header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    /* Covarianceの変換 */
    // 回転量を算出
    std::vector<double> rpy_origin,rpy_transformed,rpy_diff;
    quat2rpy(pose_origin.pose.orientation,&rpy_origin);
    quat2rpy(pose_transformed.pose.orientation,&rpy_transformed);
    rpy_diff={rpy_transformed[0]-rpy_origin[0],rpy_transformed[1]-rpy_origin[1],rpy_transformed[2]-rpy_origin[2]};
    Eigen::Matrix3d rot;
    rot= Eigen::AngleAxisd(rpy_diff[0], Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(rpy_diff[1], Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(rpy_diff[2], Eigen::Vector3d::UnitZ());

    Eigen::MatrixXd cov_origin(6,6);
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            cov_origin(j,i)=origin.pose.covariance[j*6+i];
        }
    }

    Eigen::MatrixXd convert(6,6);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            convert(j,i)=rot(j,i);
        }
    }
    for(int i=3;i<6;i++){
        for(int j=0;j<3;j++){
            convert(j,i)=convert(i,j)=0.0;
        }
    }
    for(int i=3;i<6;i++){
        for(int j=3;j<6;j++){
            if(i==j)    convert(i,j)=1;
            else        convert(i,j)=0;
        }
    }

    Eigen::MatrixXd cov_transformed(6,6);
    cov_transformed=convert*cov_origin*convert.transpose();

    out->header=pose_transformed.header;
    out->pose.pose=pose_transformed.pose;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            out->pose.covariance[j*6+i]=cov_transformed(j,i);
        }
    }

}
void transform(const tf::TransformListener &listener,const nav_msgs::Odometry& origin,geometry_msgs::PoseWithCovarianceStamped *out){

    // Poseの変換
    geometry_msgs::PoseStamped pose_origin,pose_transformed;
    pose_origin.header=pose_transformed.header=origin.header;
    pose_origin.pose=origin.pose.pose;
    try {
        listener.transformPose("local_cs",pose_origin.header.stamp,pose_origin,pose_origin.header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    /* Covarianceの変換 */
    // 回転量を算出
    std::vector<double> rpy_origin,rpy_transformed,rpy_diff;
    quat2rpy(pose_origin.pose.orientation,&rpy_origin);
    quat2rpy(pose_transformed.pose.orientation,&rpy_transformed);
    rpy_diff={rpy_transformed[0]-rpy_origin[0],rpy_transformed[1]-rpy_origin[1],rpy_transformed[2]-rpy_origin[2]};
    Eigen::Matrix3d rot;
    rot= Eigen::AngleAxisd(rpy_diff[0], Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(rpy_diff[1], Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(rpy_diff[2], Eigen::Vector3d::UnitZ());

    Eigen::MatrixXd cov_origin(6,6);
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            cov_origin(j,i)=origin.pose.covariance[j*6+i];
        }
    }

    Eigen::MatrixXd convert(6,6);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            convert(j,i)=rot(j,i);
        }
    }
    for(int i=3;i<6;i++){
        for(int j=0;j<3;j++){
            convert(j,i)=convert(i,j)=0.0;
        }
    }
    for(int i=3;i<6;i++){
        for(int j=3;j<6;j++){
            if(i==j)    convert(i,j)=1;
            else        convert(i,j)=0;
        }
    }

    Eigen::MatrixXd cov_transformed(6,6);
    cov_transformed=convert*cov_origin*convert.transpose();

    out->header=pose_transformed.header;
    out->pose.pose=pose_transformed.pose;
    for(int i=0;i<6;i++){
        for(int j=0;j<6;j++){
            out->pose.covariance[j*6+i]=cov_transformed(j,i);
        }
    }

}


class SlamFusion {

private:

    ros::NodeHandle nh;

    std::vector<slam_fusion::PathWithCovariance> paths;  // groundtruth座標に変換してからpush_back
    nav_msgs::Path groundtruth_path;
//    nav_msgs::Path path_mean,path_median;

    std::vector<bool> get_latest_pose_;

    bool existAllPose();

    ros::Publisher pub_groundtruth_path;
//    ros::Publisher pub_mean_pose,pub_mean_path;
//    ros::Publisher pub_median_pose,pub_median_path;
    std::vector<ros::Publisher> pub_slam_pose,pub_slam_path;

    tf::TransformListener listener;

//    std::vector<std::string> use_slam_names;

    ros::Time current_time;


public:
    int slam_num;
    std::vector<std::string> pose_topic_names;
    std::vector<bool> is_use_slam;


    // Functions

public:

    // Functions

    void initialize(ros::NodeHandle _nh);

    void callback_sptam(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void callback_limo_path(const slam_fusion::PathWithCovariance::ConstPtr &msg);
    void callback_orb_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void callback_ndt_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void callback_loam_pose(const nav_msgs::Odometry::ConstPtr &msg);
    void callback_groundtruth(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callback_finish(const std_msgs::EmptyConstPtr &msg);

    void fusion();

    void publish();

    void publishAllPoseWithCovariance();


//    inline std::vector<std::string> use_slam_names() const {
//        return use_slam_names;
//    }

};

void SlamFusion::initialize(ros::NodeHandle _nh) {

    ROS_INFO("Initialize slam_fusion");

    nh = _nh;

    is_use_slam={USE_SPTAM,USE_LIMO,USE_ORB,USE_NDT,USE_LOAM};

    std::string str;
    slam_num=0;
    if(is_use_slam[0]){
        slam_num++;
    }
    if(is_use_slam[1]){
        slam_num++;
    }
    if(is_use_slam[2]){
        slam_num++;
    }
    if(is_use_slam[3]){
        slam_num++;
    }
    if(is_use_slam[4]){
        slam_num++;
    }

    paths.resize(is_use_slam.size());
    get_latest_pose_ = std::vector<bool>(is_use_slam.size(), false);
    pub_slam_pose.resize(is_use_slam.size());
    pub_slam_path.resize(is_use_slam.size());

    pub_groundtruth_path=nh.advertise<nav_msgs::Path>("/groundtruth_pose/path",10);
//    pub_mean_pose=nh.advertise<geometry_msgs::PoseStamped>("/fusion/mean_pose",10);
//    pub_mean_path=nh.advertise<nav_msgs::Path>("/fusion/mean_path",10);
//    pub_median_pose=nh.advertise<geometry_msgs::PoseStamped>("/fusion/median_pose",10);
//    pub_median_path=nh.advertise<nav_msgs::Path>("/fusion/median_path",10);
    if(is_use_slam[0]){
        pub_slam_pose[0]=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_cs/sptam_pose",10);
        pub_slam_path[0]=nh.advertise<nav_msgs::Path>("/local_cs/sptam_path",10);
    }
    if(is_use_slam[1]){
        pub_slam_pose[1]=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_cs/limo_pose",10);
        pub_slam_path[1]=nh.advertise<nav_msgs::Path>("/local_cs/limo_path",10);
    }
    if(is_use_slam[2]){
        pub_slam_pose[2]=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_cs/orb_pose",10);
        pub_slam_path[2]=nh.advertise<nav_msgs::Path>("/local_cs/orb_path",10);
    }
    if(is_use_slam[3]){
        pub_slam_pose[3]=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_cs/ndt_pose",10);
        pub_slam_path[3]=nh.advertise<nav_msgs::Path>("/local_cs/ndt_path",10);
    }
    if(is_use_slam[4]){
        pub_slam_pose[4]=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_cs/loam_pose",10);
        pub_slam_path[4]=nh.advertise<nav_msgs::Path>("/local_cs/loam_path",10);
    }


}

void SlamFusion::callback_sptam(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_SPTAM;
    ROS_DEBUG_STREAM("callback " << str << " : " << msg->header.stamp.toSec());

    /***** 座標変換 *****/
    geometry_msgs::PoseWithCovarianceStamped pose_transformed;
    transform(listener,*msg,&pose_transformed);

    paths[0].poses_with_covariance.push_back(pose_transformed);
    paths[0].header=pose_transformed.header;

    get_latest_pose_[0] = true;


    if (existAllPose()) {
        fusion();
        publish();
    }
}

void SlamFusion::callback_limo_path(const slam_fusion::PathWithCovariance::ConstPtr &msg) {

    std::string str = POSE_NAME_LIMO;
    ROS_DEBUG_STREAM("callback " << str << " : " << msg->header.stamp.toSec());

    geometry_msgs::PoseWithCovarianceStamped pose_tmp;

    geometry_msgs::PoseWithCovarianceStamped pose_transformed;
    if(paths[1].poses_with_covariance.empty()){
//        int mi=0;
        for(int i=0;i<msg->poses_with_covariance.size();i++){
            /***** 座標変換 *****/
            geometry_msgs::PoseWithCovarianceStamped pose_transformed;
            transform(listener,msg->poses_with_covariance[i],&pose_transformed);

            if (i==1){
                pose_tmp=pose_transformed;
            }

            paths[1].poses_with_covariance.push_back(pose_transformed);
        }
    }
    else{
        int mi=1;   // msg->pose_with_covariance[0]の共分散は0を含むため除外
        for(int pi=0;pi<paths[1].poses_with_covariance.size();pi++){
            if(paths[1].poses_with_covariance[pi].header.stamp.toSec()==msg->poses_with_covariance[mi].header.stamp.toSec()){
                /***** 座標変換 *****/
                transform(listener,msg->poses_with_covariance[mi],&pose_transformed);
                paths[1].poses_with_covariance[pi]=pose_transformed;

                if (mi==1){
                    pose_tmp=pose_transformed;
                }

                mi++;
                if(mi>=msg->poses_with_covariance.size()){
                    break;
                }
            }
        }
        for(int i=mi;i<msg->poses_with_covariance.size();i++){
            /***** 座標変換 *****/
            geometry_msgs::PoseWithCovarianceStamped pose_transformed;
            transform(listener,msg->poses_with_covariance[i],&pose_transformed);

            paths[1].poses_with_covariance.push_back(pose_transformed);
        }

    }

//
//    paths[1].poses_with_covariance.clear();
//    for(int i=0;i<msg->poses_with_covariance.size();i++){
//        /***** 座標変換 *****/
//        geometry_msgs::PoseWithCovarianceStamped pose_transformed;
//        transform(listener,msg->poses_with_covariance[i],&pose_transformed);
//
//        paths[1].poses_with_covariance.push_back(pose_transformed);
//    }
//
//    paths[1].header=msg->header;

    ROS_DEBUG_STREAM("size of pose in limo_path: " << msg->poses_with_covariance.size());

//    pub_slam_pose[1].publish(paths[1].poses_with_covariance.back());
    pub_slam_pose[1].publish(pose_tmp);
//    pub_slam_pose[1].publish(msg->poses_with_covariance[1]);

    get_latest_pose_[1] = true;
    if (existAllPose()) {
        fusion();
        publish();
    }
}

void SlamFusion::callback_orb_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_ORB;
    ROS_DEBUG_STREAM("callback " << str << " : " << msg->header.stamp.toSec());

    /***** 座標変換 *****/

    geometry_msgs::PoseWithCovarianceStamped pose_transformed;
    transform(listener,*msg,&pose_transformed);

    paths[2].poses_with_covariance.push_back(pose_transformed);
    paths[2].header=pose_transformed.header;

    get_latest_pose_[2] = true;
    if (existAllPose()) {
        fusion();
        publish();
    }

}

void SlamFusion::callback_ndt_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_NDT;
    ROS_DEBUG_STREAM("callback " << str << " : " << msg->header.stamp.toSec());

    /***** 座標変換 *****/
    geometry_msgs::PoseWithCovarianceStamped pose_transformed;
    transform(listener,*msg,&pose_transformed);

    paths[3].poses_with_covariance.push_back(pose_transformed);
    paths[3].header=pose_transformed.header;

    get_latest_pose_[3] = true;
    if (existAllPose()) {
        fusion();
        publish();
    }

}

void SlamFusion::callback_loam_pose(const nav_msgs::Odometry::ConstPtr &msg) {

    std::string str = POSE_NAME_LOAM;
    ROS_DEBUG_STREAM("callback " << str << " : " << msg->header.stamp.toSec());

    /***** 座標変換 *****/

    geometry_msgs::PoseWithCovarianceStamped pose_transformed;
    transform(listener,*msg,&pose_transformed);

    paths[4].poses_with_covariance.push_back(pose_transformed);
    paths[4].header=pose_transformed.header;

    get_latest_pose_[4] = true;
    if (existAllPose()) {
        fusion();
        publish();
    }

}

void SlamFusion::callback_groundtruth(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_GROUNDTRUTH;
    ROS_DEBUG_STREAM("callback " << str << " : " << msg->header.stamp.toSec());

    groundtruth_path.poses.push_back(*msg);
    groundtruth_path.header=msg->header;

}

bool SlamFusion::existAllPose() {
    if(!is_use_slam[0]){
        get_latest_pose_[0]=true;
    }
    if(!is_use_slam[1]){
        get_latest_pose_[1]=true;
    }
    if(!is_use_slam[2]){
        get_latest_pose_[2]=true;
    }
    if(!is_use_slam[3]){
        get_latest_pose_[3]=true;
    }
    if(!is_use_slam[4]){
        get_latest_pose_[4]=true;
    }

    auto itr = std::find(get_latest_pose_.begin(), get_latest_pose_.end(), false);
    if (itr == get_latest_pose_.end()) {
        ROS_DEBUG_STREAM("Getting All Packages' Pose");
        fusion();
        get_latest_pose_ = std::vector<bool>(is_use_slam.size(), false);

        return true;
    } else {

//        std::cout<<"/------ Non exist -----/";
        if(is_use_slam[0]){
            std::cout<<"sptam: " << get_latest_pose_[0] <<std::endl;
        }
        if(is_use_slam[1]){
            std::cout<<"limo:  " << get_latest_pose_[1] <<std::endl;
        }
        if(is_use_slam[2]){
            std::cout<<"orb:   " << get_latest_pose_[2] <<std::endl;
        }
        if(is_use_slam[3]){
            std::cout<<"ndt:   " << get_latest_pose_[3] <<std::endl;
        }
        if(is_use_slam[4]){
            std::cout<<"loam:   " << get_latest_pose_[4] <<std::endl;
        }

        return false;
    }

}

void SlamFusion::fusion() {

//    ROS_INFO("Fuse Poses");
//
//    if(path_mean.poses.empty() || path_median.poses.empty()){
//        std::vector<double> x,y,z,ro,pi,ya;
//        for(int i=0;i<paths.size();i++){
//
//            if(is_use_slam[i]){
//                geometry_msgs::Pose po=paths[i].poses.back().pose;
//
//                std::vector<double> po_rpy;
//                quat2rpy(po.orientation,&po_rpy);
//
//                x.push_back(po.position.x);
//                y.push_back(po.position.y);
//                z.push_back(po.position.z);
//                ro.push_back(po_rpy[0]);
//                pi.push_back(po_rpy[1]);
//                ya.push_back(po_rpy[2]);
//            }
//        }
//
//
//        for(const auto &pa:paths){
//        }
//
//        std::vector<double> mean(6),median(6);
//        // 平均
//        mean[0]=getMean(x);
//        mean[1]=getMean(y);
//        mean[2]=getMean(z);
//        mean[3]=getMean(ro);
//        mean[4]=getMean(pi);
//        mean[5]=getMean(ya);
//        std::cout<<mean[0]<<mean[1]<<mean[2]<<mean[3]<<mean[4]<<mean[5]<<std::endl;
//        // 中央値
//        median[0]=getMedian(x);
//        median[1]=getMedian(y);
//        median[2]=getMedian(z);
//        median[3]=getMedian(ro);
//        median[4]=getMedian(pi);
//        median[5]=getMedian(ya);
//        std::cout<<median[0]<<median[1]<<median[2]<<median[3]<<median[4]<<median[5]<<std::endl;
//
//        geometry_msgs::PoseStamped mean_st,median_st;
//        poseRpy2poseQuat(mean,&mean_st.pose);
//        mean_st.header.stamp=current_time;
//        mean_st.header.frame_id="local_cs";
//        poseRpy2poseQuat(median,&median_st.pose);
//        median_st.header.stamp=current_time;
//        median_st.header.frame_id="local_cs";
//
//        path_mean.poses.push_back(mean_st);
//        path_mean.header=mean_st.header;
//        path_median.poses.push_back(median_st);
//        path_median.header=median_st.header;
//
//    }
//    else{
//
//        std::vector<double> x,y,z,ro,pi,ya;
//        for(int i=0;i<paths.size();i++) {
//            if (is_use_slam[i]) {
//                geometry_msgs::Pose po=paths[i].poses.back().pose;
//                geometry_msgs::Pose ppo=paths[i].poses[paths[i].poses.size()-1].pose;
//
//                std::vector<double> po_rpy,ppo_rpy;
//                quat2rpy(po.orientation,&po_rpy);
//                quat2rpy(ppo.orientation,&ppo_rpy);
//
//                x.push_back(po.position.x-ppo.position.x);
//                y.push_back(po.position.y-ppo.position.y);
//                z.push_back(po.position.z-ppo.position.z);
//                ro.push_back(po_rpy[0]-ppo_rpy[0]);
//                pi.push_back(po_rpy[1]-ppo_rpy[1]);
//                ya.push_back(po_rpy[2]-ppo_rpy[2]);
//
//            }
//        }
//
//
//        for(const auto &pa:paths){
//
//        }
//
//        std::vector<double> mean(6),median(6);
//
//        // 平均
//        mean[0]=getMean(x);
//        mean[1]=getMean(y);
//        mean[2]=getMean(z);
//        mean[3]=getMean(ro);
//        mean[4]=getMean(pi);
//        mean[5]=getMean(ya);
//        std::cout<<mean[0]<<mean[1]<<mean[2]<<mean[3]<<mean[4]<<mean[5]<<std::endl;
//        // 中央値
//        median[0]=getMedian(x);
//        median[1]=getMedian(y);
//        median[2]=getMedian(z);
//        median[3]=getMedian(ro);
//        median[4]=getMedian(pi);
//        median[5]=getMedian(ya);
//        std::cout<<median[0]<<median[1]<<median[2]<<median[3]<<median[4]<<median[5]<<std::endl;
//
//        std::vector<double> mean_pre(6),median_pre(6);
//        poseQuat2poseRpy(path_mean.poses.back().pose,&mean_pre);
//        poseQuat2poseRpy(path_median.poses.back().pose,&median_pre);
//
//        std::vector<double> mean_now(6),median_now(6);
//        for(int i=0;i<mean_now.size();i++){
//            mean_now[i]=mean_pre[i]+mean[i];
//            median_now[i]=median_pre[i]+median[i];
//        }
//
//        geometry_msgs::PoseStamped mean_st,median_st;
//        poseRpy2poseQuat(mean_now,&mean_st.pose);
//        mean_st.header.stamp=current_time;
//        mean_st.header.frame_id="local_cs";
//        poseRpy2poseQuat(median_now,&median_st.pose);
//        median_st.header.stamp=current_time;
//        median_st.header.frame_id="local_cs";
//
//        path_mean.poses.push_back(mean_st);
//        path_median.poses.push_back(median_st);
//
//
//    }
//
//    publish();

}

void SlamFusion::publish() {
//    pub_groundtruth_path.publish(groundtruth_path);
//    pub_mean_pose.publish(path_mean.poses.back());
//    pub_mean_path.publish(path_mean);
//    pub_median_pose.publish(path_median.poses.back());
//    pub_median_path.publish(path_median);
//    if(is_use_slam[0]){
//        pub_slam_pose[0].publish(paths[0].poses_with_covariance.back());
//        pub_slam_path[0].publish(paths[0]);
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    }
//    if(is_use_slam[1]){
//        pub_slam_pose[1].publish(paths[1].poses_with_covariance.back());
//        pub_slam_path[1].publish(paths[1]);
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    }
//    if(is_use_slam[2]){
//        pub_slam_pose[2].publish(paths[2].poses_with_covariance.back());
//        pub_slam_path[2].publish(paths[2]);
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    }
//    if(is_use_slam[3]){
//        pub_slam_pose[3].publish(paths[3].poses_with_covariance.back());
//        pub_slam_path[3].publish(paths[3]);
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    }


}

void SlamFusion::callback_finish(const std_msgs::EmptyConstPtr &msg) {

    std::cout<<"callback_finish"<<std::endl;
    rosbag::Bag bag;
    bag.open("test.bag",rosbag::bagmode::Write);

    std::vector<std::string> str={  "local_cs/sptam_pose",
                                    "local_cs/limo_pose",
                                    "local_cs/orb_pose",
                                    "local_cs/ndt_pose",
                                    "local_cs/loam_pose"};

    for(int pi=0;pi<is_use_slam.size();pi++){
        for(int i=0;i<paths[pi].poses_with_covariance.size();i++){
            if (is_use_slam[pi]) {
                ros::Time time=paths[pi].poses_with_covariance[i].header.stamp;
                if(time.toSec()<=0.0){
                    continue;
                }
                std::cout<<str[pi]<<std::endl;
                bag.write(str[pi],time,paths[pi].poses_with_covariance[i]);
//            pub_slam_pose[pi].publish(paths[pi].poses_with_covariance[i]);
//                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

    }
    for(int i=0;i<groundtruth_path.poses.size();i++){
        ros::Time time=groundtruth_path.poses[i].header.stamp;
        if(time.toSec()<=0.0){
            continue;
        }
        std::cout<<POSE_NAME_GROUNDTRUTH<<std::endl;
        bag.write(POSE_NAME_GROUNDTRUTH,time,groundtruth_path.poses[i]);
//            pub_slam_pose[pi].publish(paths[pi].poses_with_covariance[i]);
//        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    }



//    for (int i = 0; i < groundtruth_path.poses.size(); i++) {
//        if (is_use_slam[0]) {
//            pub_slam_pose[0].publish(paths[0].poses_with_covariance[i]);
//            std::this_thread::sleep_for(std::chrono::milliseconds(50));
//        }
//        if (is_use_slam[2]) {
//            pub_slam_pose[2].publish(paths[2].poses_with_covariance[i]);
//            std::this_thread::sleep_for(std::chrono::milliseconds(50));
//        }
//        if (is_use_slam[3]) {
//            pub_slam_pose[3].publish(paths[3].poses_with_covariance[i]);
//            std::this_thread::sleep_for(std::chrono::milliseconds(50));
//        }
//    }

    bag.close();
    std::cout << "Close Bag" <<std::endl;


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
