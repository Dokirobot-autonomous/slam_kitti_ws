#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>

//#define USE_SLAM_NAMES "SPTAM,LIMO,ORB,NDT"

bool USE_SPTAM=false;
bool USE_LIMO=true;
bool USE_ORB=true;
bool USE_NDT=true;

#define POSE_NAME_SPTAM "sptam/robot/pose"
#define POSE_NAME_LIMO "/estimate/complete_path"
#define POSE_NAME_ORB "/camera_pose"
#define POSE_NAME_NDT "/current_pose"
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

class SlamFusion {

private:

    ros::NodeHandle nh;

    std::vector<nav_msgs::Path> paths;  // groundtruth座標に変換してからpush_back
    nav_msgs::Path groundtruth_path;
    nav_msgs::Path path_mean,path_median;

    std::vector<bool> get_latest_pose_;

    bool existAllPose();

    ros::Publisher pub_groundtruth_path;
    ros::Publisher pub_mean_pose,pub_mean_path;
    ros::Publisher pub_median_pose,pub_median_path;
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
    void callback_limo_path(const nav_msgs::Path::ConstPtr &msg);
    void callback_orb_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callback_ndt_pose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void callback_groundtruth(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void fusion();

    void publish();


//    inline std::vector<std::string> use_slam_names() const {
//        return use_slam_names;
//    }

};

void SlamFusion::initialize(ros::NodeHandle _nh) {

    ROS_INFO("Initialize slam_fusion");

    nh = _nh;

    is_use_slam={USE_SPTAM,USE_LIMO,USE_ORB,USE_NDT};

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

    paths.resize(4);
    get_latest_pose_ = std::vector<bool>(4, false);
    pub_slam_pose.resize(4);
    pub_slam_path.resize(4);

    pub_groundtruth_path=nh.advertise<nav_msgs::Path>("/groundtruth_pose/path",10);
    pub_mean_pose=nh.advertise<geometry_msgs::PoseStamped>("/fusion/mean_pose",10);
    pub_mean_path=nh.advertise<nav_msgs::Path>("/fusion/mean_path",10);
    pub_median_pose=nh.advertise<geometry_msgs::PoseStamped>("/fusion/median_pose",10);
    pub_median_path=nh.advertise<nav_msgs::Path>("/fusion/median_path",10);
    if(is_use_slam[0]){
        pub_slam_pose[0]=nh.advertise<geometry_msgs::PoseStamped>("/local_cs/sptam_pose",10);
        pub_slam_path[0]=nh.advertise<nav_msgs::Path>("/local_cs/sptam_path",10);
    }
    if(is_use_slam[1]){
        pub_slam_pose[1]=nh.advertise<geometry_msgs::PoseStamped>("/local_cs/limo_pose",10);
        pub_slam_path[1]=nh.advertise<nav_msgs::Path>("/local_cs/limo_path",10);
    }
    if(is_use_slam[2]){
        pub_slam_pose[2]=nh.advertise<geometry_msgs::PoseStamped>("/local_cs/orb_pose",10);
        pub_slam_path[2]=nh.advertise<nav_msgs::Path>("/local_cs/orb_path",10);
    }
    if(is_use_slam[3]){
        pub_slam_pose[3]=nh.advertise<geometry_msgs::PoseStamped>("/local_cs/ndt_pose",10);
        pub_slam_path[3]=nh.advertise<nav_msgs::Path>("/local_cs/ndt_path",10);
    }


}

void SlamFusion::callback_sptam(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_SPTAM;
    ROS_DEBUG_STREAM("callback " << str);

    /* Poseの確認 */
    geometry_msgs::PoseStamped tmp;
    tmp.header = msg->header;
    tmp.pose = msg->pose.pose;
    geometry_msgs::PoseStamped pose_transformed;
    try {
        listener.transformPose("local_cs",tmp.header.stamp,tmp,msg->header.frame_id,pose_transformed);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    ROS_DEBUG_STREAM("sptam local_cs pose \n"<<pose_transformed);

    paths[0].poses.push_back(pose_transformed);
    paths[0].header=pose_transformed.header;

    get_latest_pose_[0] = true;


    if (existAllPose()) {
        fusion();
    }
}

void SlamFusion::callback_limo_path(const nav_msgs::Path::ConstPtr &msg) {

    std::string str = POSE_NAME_LIMO;
    ROS_DEBUG_STREAM("callback " << str);
    if(paths[1].poses.empty()){
        for(int i=0;i<msg->poses.size();i++){
            /* Poseの確認 */
            geometry_msgs::PoseStamped pose_transformed;
            try {
                listener.transformPose("local_cs",msg->poses[i].header.stamp,msg->poses[i],msg->poses[i].header.frame_id,pose_transformed);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            ROS_DEBUG_STREAM("limo local_cs pose \n"<<pose_transformed);

            paths[1].poses.push_back(pose_transformed);

        }
        paths[1].header=msg->header;
    }
    else{
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

        paths[1].poses.push_back(pose_transformed);
        paths[1].header=pose_transformed.header;
    }

    ROS_DEBUG_STREAM("size of pose in limo_path: " << msg->poses.size());


    get_latest_pose_[1] = true;
    if (existAllPose()) {
        fusion();
    }
}

void SlamFusion::callback_orb_pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_ORB;
    ROS_DEBUG_STREAM("callback " << str);

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

    paths[2].poses.push_back(pose_transformed);
    paths[2].header=pose_transformed.header;

    get_latest_pose_[2] = true;
    if (existAllPose()) {
        fusion();
    }

}

void SlamFusion::callback_ndt_pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_NDT;
    ROS_DEBUG_STREAM("callback " << str);

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

    paths[3].poses.push_back(pose_transformed);
    paths[3].header=pose_transformed.header;

    get_latest_pose_[3] = true;
    if (existAllPose()) {
        fusion();
    }

}

void SlamFusion::callback_groundtruth(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    std::string str = POSE_NAME_GROUNDTRUTH;
    ROS_DEBUG_STREAM("callback " << str);

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

    auto itr = std::find(get_latest_pose_.begin(), get_latest_pose_.end(), false);
    if (itr == get_latest_pose_.end()) {
        ROS_DEBUG_STREAM("Getting All Packages' Pose");
        fusion();
        get_latest_pose_ = std::vector<bool>(4, false);

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

        return false;
    }

}

void SlamFusion::fusion() {

    ROS_INFO("Fuse Poses");

    if(path_mean.poses.empty() || path_median.poses.empty()){
        std::vector<double> x,y,z,ro,pi,ya;
        for(int i=0;i<paths.size();i++){

            if(is_use_slam[i]){
                geometry_msgs::Pose po=paths[i].poses.back().pose;

                std::vector<double> po_rpy;
                quat2rpy(po.orientation,&po_rpy);

                x.push_back(po.position.x);
                y.push_back(po.position.y);
                z.push_back(po.position.z);
                ro.push_back(po_rpy[0]);
                pi.push_back(po_rpy[1]);
                ya.push_back(po_rpy[2]);
            }
        }


        for(const auto &pa:paths){
        }

        std::vector<double> mean(6),median(6);
        // 平均
        mean[0]=getMean(x);
        mean[1]=getMean(y);
        mean[2]=getMean(z);
        mean[3]=getMean(ro);
        mean[4]=getMean(pi);
        mean[5]=getMean(ya);
        std::cout<<mean[0]<<mean[1]<<mean[2]<<mean[3]<<mean[4]<<mean[5]<<std::endl;
        // 中央値
        median[0]=getMedian(x);
        median[1]=getMedian(y);
        median[2]=getMedian(z);
        median[3]=getMedian(ro);
        median[4]=getMedian(pi);
        median[5]=getMedian(ya);
        std::cout<<median[0]<<median[1]<<median[2]<<median[3]<<median[4]<<median[5]<<std::endl;

        geometry_msgs::PoseStamped mean_st,median_st;
        poseRpy2poseQuat(mean,&mean_st.pose);
        mean_st.header.stamp=current_time;
        mean_st.header.frame_id="local_cs";
        poseRpy2poseQuat(median,&median_st.pose);
        median_st.header.stamp=current_time;
        median_st.header.frame_id="local_cs";

        path_mean.poses.push_back(mean_st);
        path_mean.header=mean_st.header;
        path_median.poses.push_back(median_st);
        path_median.header=median_st.header;

    }
    else{

        std::vector<double> x,y,z,ro,pi,ya;
        for(int i=0;i<paths.size();i++) {
            if (is_use_slam[i]) {
                geometry_msgs::Pose po=paths[i].poses.back().pose;
                geometry_msgs::Pose ppo=paths[i].poses[paths[i].poses.size()-1].pose;

                std::vector<double> po_rpy,ppo_rpy;
                quat2rpy(po.orientation,&po_rpy);
                quat2rpy(ppo.orientation,&ppo_rpy);

                x.push_back(po.position.x-ppo.position.x);
                y.push_back(po.position.y-ppo.position.y);
                z.push_back(po.position.z-ppo.position.z);
                ro.push_back(po_rpy[0]-ppo_rpy[0]);
                pi.push_back(po_rpy[1]-ppo_rpy[1]);
                ya.push_back(po_rpy[2]-ppo_rpy[2]);

            }
        }


        for(const auto &pa:paths){

        }

        std::vector<double> mean(6),median(6);

        // 平均
        mean[0]=getMean(x);
        mean[1]=getMean(y);
        mean[2]=getMean(z);
        mean[3]=getMean(ro);
        mean[4]=getMean(pi);
        mean[5]=getMean(ya);
        std::cout<<mean[0]<<mean[1]<<mean[2]<<mean[3]<<mean[4]<<mean[5]<<std::endl;
        // 中央値
        median[0]=getMedian(x);
        median[1]=getMedian(y);
        median[2]=getMedian(z);
        median[3]=getMedian(ro);
        median[4]=getMedian(pi);
        median[5]=getMedian(ya);
        std::cout<<median[0]<<median[1]<<median[2]<<median[3]<<median[4]<<median[5]<<std::endl;

        std::vector<double> mean_pre(6),median_pre(6);
        poseQuat2poseRpy(path_mean.poses.back().pose,&mean_pre);
        poseQuat2poseRpy(path_median.poses.back().pose,&median_pre);

        std::vector<double> mean_now(6),median_now(6);
        for(int i=0;i<mean_now.size();i++){
            mean_now[i]=mean_pre[i]+mean[i];
            median_now[i]=median_pre[i]+median[i];
        }

        geometry_msgs::PoseStamped mean_st,median_st;
        poseRpy2poseQuat(mean_now,&mean_st.pose);
        mean_st.header.stamp=current_time;
        mean_st.header.frame_id="local_cs";
        poseRpy2poseQuat(median_now,&median_st.pose);
        median_st.header.stamp=current_time;
        median_st.header.frame_id="local_cs";

        path_mean.poses.push_back(mean_st);
        path_median.poses.push_back(median_st);


    }

    publish();

}

void SlamFusion::publish() {
    pub_groundtruth_path.publish(groundtruth_path);
    pub_mean_pose.publish(path_mean.poses.back());
    pub_mean_path.publish(path_mean);
    pub_median_pose.publish(path_median.poses.back());
    pub_median_path.publish(path_median);
    if(is_use_slam[0]){
        pub_slam_pose[0].publish(paths[0].poses.back());
        pub_slam_path[0].publish(paths[0]);
    }
    if(is_use_slam[1]){
        pub_slam_pose[1].publish(paths[1].poses.back());
        pub_slam_path[1].publish(paths[1]);
    }
    if(is_use_slam[2]){
        pub_slam_pose[2].publish(paths[2].poses.back());
        pub_slam_path[2].publish(paths[2]);
    }
    if(is_use_slam[3]){
        pub_slam_pose[3].publish(paths[3].poses.back());
        pub_slam_path[3].publish(paths[3]);
    }
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

    ros::Subscriber pub_sptam_pose,sub_limo_pose,sub_orb_pose,sub_ndt_pose;

    if(fusion.is_use_slam[0]){
        pub_sptam_pose = nh.subscribe(POSE_NAME_SPTAM, 1, &SlamFusion::callback_sptam, &fusion);
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
    ros::Subscriber sub_groundtruth = nh.subscribe(POSE_NAME_GROUNDTRUTH, 1, &SlamFusion::callback_groundtruth, &fusion);

    ros::spin();

    return (0);
}
