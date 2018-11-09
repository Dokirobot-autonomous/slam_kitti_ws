//
// Created by ohashi on 18/11/07.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>


/* for NDT Mapping */
//#define RESOLUTION {0.2,0.4,0.6,0.8,1.0}
//#define STEP_SIZE {0.05,0.1,0.15,0.2}
//#define TRANSFORMATION_EPSILON {0.01,0.02,0.03}
//#define MAXIMUM_ITERATIONS {30}
//#define LEAF_SIZE {0.2,0.4,0.6,0.8,1.0}
#define RESOLUTION {5.0,4.0,3.0,2.0,1.0}
#define STEP_SIZE {0.1}
#define TRANSFORMATION_EPSILON {0.01}
#define MAXIMUM_ITERATIONS {30}
#define LEAF_SIZE {1.0}
#define MINIMUM_SCAN_RANGE {5}
#define MAXIMUM_SCAN_RANGE {200}
#define MINIMUM_ADD_SCAN_SHIFT {1}

#define OUTPUT_FNAME "/media/ohashi/e5c3c17e-1d77-4109-ae86-cb7c80ff9c1c/share/rosbag/Kitti/odometry/dataset/sequences/"

class ErrorCalculator{

private:
    nav_msgs::Path ground,ndt_path,ndt_path_transformed;
    bool start_calculation_;

    tf::TransformListener listener;

public:
    void callback_ground(const nav_msgs::Path::ConstPtr &msg){
        ROS_INFO("callback_ground");
        ground=*msg;
    }
    void callback_ndt_path(const nav_msgs::Path::ConstPtr &msg) {
        ROS_INFO("callback_ndt");
        ndt_path = *msg;
    }
    void transformPath(){

        ndt_path_transformed.poses.clear();

        for(unsigned int i=1;i<ndt_path.poses.size();i++){

            geometry_msgs::PoseStamped pose=ndt_path.poses[i];
            geometry_msgs::PoseStamped pose_transformed;

            try {
                listener.transformPose("local_cs",pose.header.stamp,pose,pose.header.frame_id,pose_transformed);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
//            ROS_DEBUG_STREAM("ndt local_cs pose \n"<<pose_transformed);

            ndt_path_transformed.poses.push_back(pose_transformed);

        }

        ndt_path_transformed.header=ndt_path.header;

        ROS_INFO_STREAM("Done transformation: "<<ndt_path_transformed.poses.size());
    }
    float getTransitionError(){

        const nav_msgs::Path& path1=ground;
        const nav_msgs::Path& path2=ndt_path_transformed;

        float error=0.0;                // [%]
        float migration_distance=0.0;

        for(unsigned int i=1;i<path1.poses.size();i++){

            /* 移動距離 */
            float dx=path1.poses[i].pose.position.x-path1.poses[i-1].pose.position.x;
            float dy=path1.poses[i].pose.position.y-path1.poses[i-1].pose.position.y;
            float dz=path1.poses[i].pose.position.z-path1.poses[i-1].pose.position.z;
            migration_distance+=std::sqrt(dx*dx+dy*dy+dz*dz);

            float ex=path1.poses[i].pose.position.x-path2.poses[i].pose.position.x;
            float ey=path1.poses[i].pose.position.y-path2.poses[i].pose.position.y;
            float ez=path1.poses[i].pose.position.z-path2.poses[i].pose.position.z;
            error+=(std::sqrt(ex*ex+ey*ey+ez*ez)/migration_distance);

        }

        ROS_INFO_STREAM("Get transition error: " << error/path1.poses.size());

        return error/path1.poses.size();

    }
    float getRotationError(){

        const nav_msgs::Path& path1=ground;
        const nav_msgs::Path& path2=ndt_path_transformed;

        float error=0.0;                // [%]

        for(unsigned int i=1;i<path1.poses.size();i++){

            geometry_msgs::Quaternion gq1=path1.poses[i].pose.orientation;
            geometry_msgs::Quaternion gq2=path2.poses[i].pose.orientation;
            tf::Quaternion q1(gq1.x,gq1.y,gq1.z,gq1.w);
            tf::Quaternion q2(gq2.x,gq2.y,gq2.z,gq2.w);
            tf::Matrix3x3 m1(q1);
            tf::Matrix3x3 m2(q2);
            double rpy1[3],rpy2[3];
            m1.getRPY(rpy1[0],rpy1[1],rpy1[2]);
            m2.getRPY(rpy2[0],rpy2[1],rpy2[2]);

            float dr=std::abs(rpy1[0]-rpy2[0]);
            float dp=std::abs(rpy1[1]-rpy2[1]);
            float dy=std::abs(rpy1[2]-rpy2[2]);
            while(true){
                if(dr>M_PI) dr-=2.0*M_PI;
                else break;
//                ROS_INFO_STREAM(dr);
            }
            while(true){
                if(dp>M_PI) dp-=2.0*M_PI;
                else break;
//                ROS_INFO_STREAM(dp);
            }
            while(true){
                if(dy>M_PI) dy-=2.0*M_PI;
                else break;
//                ROS_INFO_STREAM(dy);
            }

//            ROS_INFO_STREAM(std::abs(dr)+std::abs(dp)+std::abs(dy));
            error+=(std::abs(dr)+std::abs(dp)+std::abs(dy));

        }

        ROS_INFO_STREAM("Get rotatioin error: "<<error/path1.poses.size());

        return error/path1.poses.size();

    }

};

// -----------------------------------
//	Main Function
// -----------------------------------
int main(int argc, char **argv) {

    ROS_INFO("mypkg: param_optimization node");

    // debug mode
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Initialize ROS
    ros::init(argc, argv, "param_optimization");
    ros::NodeHandle nh;

    nh.setParam("exist_param_optimization",1);

    std::string fname_param = std::string(OUTPUT_FNAME) + std::string("params.csv");
    std::ofstream ofs_params(fname_param.c_str(), std::ios_base::app);
    if (ofs_params.fail()) {
        ROS_ERROR_STREAM("No exist output directory");
        std::cout << fname_param << std::endl;
        return -1;
    }
//        ofs_params<<"res,step,trans,max_itr,leaf,min_scan,max_scan,min_add"<<std::endl;

    std::string fname_tr = std::string(OUTPUT_FNAME) + std::string("error_trans.csv");
    std::ofstream ofs_tr(fname_tr.c_str(), std::ios_base::app);
    if (ofs_tr.fail()) {
        ROS_ERROR_STREAM("No exist output directory");
        std::cout << fname_param << std::endl;
        return -1;
    }
//        ofs_tr<<"Seq00,Sec01,Seq02,Sec03,Seq04,Sec05,Seq06,Sec07,Seq08,Sec09,Sec10"<<std::endl;

    std::string fname_ro = std::string(OUTPUT_FNAME) + std::string("error_rotate.csv");
    std::ofstream ofs_ro(fname_ro.c_str(), std::ios_base::app);
    if (ofs_ro.fail()) {
        ROS_ERROR_STREAM("No exist output directory");
        std::cout << fname_param << std::endl;
        return -1;
    }
    //        ofs_ro<<"Seq00,Sec01,Seq02,Sec03,Seq04,Sec05,Seq06,Sec07,Seq08,Sec09,Sec10"<<std::endl;

    int first_node_num,last_node_num;
    nh.getParam("first_node_num",first_node_num);
    nh.getParam("last_node_num",last_node_num);

    // get parameters
    std::vector<float> res,step,trans,max_ite,leaf,min_scan,max_scan,min_shift;
    res.resize(last_node_num-first_node_num+1);
    step.resize(last_node_num-first_node_num+1);
    trans.resize(last_node_num-first_node_num+1);
    max_ite.resize(last_node_num-first_node_num+1);
    leaf.resize(last_node_num-first_node_num+1);
    min_scan.resize(last_node_num-first_node_num+1);
    max_scan.resize(last_node_num-first_node_num+1);
    min_shift.resize(last_node_num-first_node_num+1);

    for(int i=first_node_num;i<=last_node_num;i++){
        std::string str="/ndt_mapping"+std::to_string(i);
        nh.getParam(str+"resolution",res[i]);
        nh.getParam(str+"step_size",step[i]);
        nh.getParam(str+"transformation_epsilon",trans[i]);
        nh.getParam(str+"maximu_iterations",max_ite[i]);
        nh.getParam(str+"leaf_size",leaf[i]);
        nh.getParam(str+"minimum_scan_range",min_scan[i]);
        nh.getParam(str+"maximum_scan_range",max_scan[i]);
        nh.getParam(str+"minimum_add_scan_shift",min_shift[i]);
    }





    std::vector<std::vector<float>> trans_errors(last_node_num-first_node_num+1);
    std::vector<std::vector<float>> rotate_errors(last_node_num-first_node_num+1);

    while(true){
        std::vector<ErrorCalculator> ec(last_node_num-first_node_num+1);
        std::vector<ros::Subscriber> sub_ground(last_node_num-first_node_num+1);
        std::vector<ros::Subscriber> sub_ndt(last_node_num-first_node_num+1);
        for(int i=first_node_num;i<=last_node_num;i++){
            sub_ground[i]=nh.subscribe("/groundtruth_pose/path", 1, &ErrorCalculator::callback_ground, &ec[i]);
            sub_ndt[i]=nh.subscribe("/current_path"+std::to_string(i), 1, &ErrorCalculator::callback_ndt_path, &ec[i]);
        }

        int exist_ndt_mapping;
        /* ndt_mapping が起動していなければloop */
        for(int i=first_node_num;i<=last_node_num;i++) {
            while (true) {
                std::string str="/ndt_mapping"+std::to_string(i)+"/exist_ndt_mapping";
                nh.getParam(str, exist_ndt_mapping);
                if (nh.hasParam(str) && exist_ndt_mapping == 1) break;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        for(int i=first_node_num;i<=last_node_num;i++) {
            while (true) {
                ros::spinOnce();
                std::string str="/ndt_mapping"+std::to_string(i)+"/exist_ndt_mapping";
                nh.getParam(str, exist_ndt_mapping);
                if (exist_ndt_mapping == 0) break;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        for(int i=first_node_num;i<=last_node_num;i++) {
            ec[i].transformPath();
            float te = ec[i].getTransitionError();
            float re = ec[i].getRotationError();
            trans_errors[i].push_back(te);
            rotate_errors[i].push_back(re);
            nh.setParam("done_error_calculation"+std::to_string(i),1);
        }

        int fin_all_sequence;
        if(!nh.getParam("fin_all_sequence",fin_all_sequence)){
            fin_all_sequence=0;
        }
        if(fin_all_sequence==1){
            nh.setParam("fin_all_sequence",0);
            break;
        }

    }

    for(int i=first_node_num;i<=last_node_num;i++) {
        ofs_params << "," << res[i] << "," << step[i] << "," << trans[i] << "," << max_ite[i] << "," << leaf[i] << "," << min_scan[i] << "," << max_scan[i] << "," << min_shift[i] << "," << std::endl;
        for(int j=0;j<trans_errors.size()-1;j++){
            ofs_tr<<trans_errors[i][j]<<",";
            ofs_ro<<rotate_errors[i][j]<<",";
        }
        ofs_tr<<std::endl;
        ofs_ro<<std::endl;
    }


    return (0);
}