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

#define OUTPUT_FNAME "/home/ohashi/slam_kitti_ws/output/errors.csv"

class ErrorCalculator {

private:
    nav_msgs::Path ground, ndt_path, ndt_path_transformed;
    bool start_calculation_;

    tf::TransformListener listener;

public:
    void callback_ground(const nav_msgs::Path::ConstPtr &msg) {
        ROS_INFO("callback_ground");
        ground = *msg;
    }

    void callback_ndt_path(const nav_msgs::Path::ConstPtr &msg) {
        ROS_INFO("callback_ndt");
        ndt_path = *msg;
    }

    void transformPath() {

        ndt_path_transformed.poses.clear();

        for (unsigned int i = 1; i < ndt_path.poses.size(); i++) {

            geometry_msgs::PoseStamped pose = ndt_path.poses[i];
            geometry_msgs::PoseStamped pose_transformed;

            try {
                listener.transformPose("local_cs", pose.header.stamp, pose, pose.header.frame_id, pose_transformed);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
//            ROS_DEBUG_STREAM("ndt local_cs pose \n"<<pose_transformed);

            ndt_path_transformed.poses.push_back(pose_transformed);

        }

        ndt_path_transformed.header = ndt_path.header;

        ROS_INFO_STREAM("Done transformation: " << ndt_path_transformed.poses.size());
    }

    double getTransitionError() {

        const nav_msgs::Path &path1 = ground;
        const nav_msgs::Path &path2 = ndt_path_transformed;

        double error = 0.0;                // [%]
        double migration_distance = 0.0;

        for (unsigned int i = 1; i < path1.poses.size(); i++) {

//            if (path1.poses[i].header.stamp != path2.poses[i].header.stamp)
//                continue;

            /* 移動距離 */
            double dx = path1.poses[i].pose.position.x - path1.poses[0].pose.position.x;
            double dy = path1.poses[i].pose.position.y - path1.poses[0].pose.position.y;
            double dz = path1.poses[i].pose.position.z - path1.poses[0].pose.position.z;
            migration_distance += std::sqrt(dx * dx + dy * dy + dz * dz);

            double ex = path1.poses[i].pose.position.x - path2.poses[0].pose.position.x;
            double ey = path1.poses[i].pose.position.y - path2.poses[0].pose.position.y;
            double ez = path1.poses[i].pose.position.z - path2.poses[0].pose.position.z;
            ROS_INFO_STREAM(std::sqrt(ex * ex + ey * ey + ez * ez) / migration_distance);
            error += (std::sqrt(ex * ex + ey * ey + ez * ez) / migration_distance);

        }

        ROS_INFO_STREAM("Get transition error: " << error / path1.poses.size());

        return error / path1.poses.size();

    }

    double getRotationError() {

        const nav_msgs::Path &path1 = ground;
        const nav_msgs::Path &path2 = ndt_path_transformed;

        double error = 0.0;                // [%]

        for (unsigned int i = 1; i < path1.poses.size(); i++) {

//            if (path1.poses[i].header.stamp != path2.poses[i].header.stamp)
//                continue;

            geometry_msgs::Quaternion gq1 = path1.poses[i].pose.orientation;
            geometry_msgs::Quaternion gq2 = path2.poses[i].pose.orientation;
            tf::Quaternion q1(gq1.x, gq1.y, gq1.z, gq1.w);
            tf::Quaternion q2(gq2.x, gq2.y, gq2.z, gq2.w);
            tf::Matrix3x3 m1(q1);
            tf::Matrix3x3 m2(q2);
            double rpy1[3], rpy2[3];
            m1.getRPY(rpy1[0], rpy1[1], rpy1[2]);
            m2.getRPY(rpy2[0], rpy2[1], rpy2[2]);

            double dr = std::abs(rpy1[0] - rpy2[0]);
            double dp = std::abs(rpy1[1] - rpy2[1]);
            double dy = std::abs(rpy1[2] - rpy2[2]);
            while (true) {
                if (dr > M_PI) dr -= 2.0 * M_PI;
                else break;
//                ROS_INFO_STREAM(dr);
            }
            while (true) {
                if (dp > M_PI) dp -= 2.0 * M_PI;
                else break;
//                ROS_INFO_STREAM(dp);
            }
            while (true) {
                if (dy > M_PI) dy -= 2.0 * M_PI;
                else break;
//                ROS_INFO_STREAM(dy);
            }

//            ROS_INFO_STREAM(std::abs(dr)+std::abs(dp)+std::abs(dy));
            error += (std::abs(dr) + std::abs(dp) + std::abs(dy));

        }

        ROS_INFO_STREAM("Get rotatioin error: " << error / path1.poses.size());

        return error / path1.poses.size();

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

    nh.setParam("exist_param_optimization", 1);

//        ofs<<"res,step,trans,max_itr,leaf,min_scan,max_scan,min_add"<<std::endl;
//    ofs << "res,leaf" << std::endl;

//    std::string fname_tr = std::string(OUTPUT_FNAME) + std::string("error_trans.csv");
//    std::ofstream ofs_tr(fname_tr.c_str(), std::ios_base::app);
//    if (ofs_tr.fail()) {
//        ROS_ERROR_STREAM("No exist output directory");
//        std::cout << fname << std::endl;
//        return -1;
//    }
////    ofs_tr << "Seq00,Sec01,Seq02,Sec03,Seq04,Sec05,Seq06,Sec07,Seq08,Sec09,Sec10" << std::endl;
//
//    std::string fname_ro = std::string(OUTPUT_FNAME) + std::string("error_rotate.csv");
//    std::ofstream ofs_ro(fname_ro.c_str(), std::ios_base::app);
//    if (ofs_ro.fail()) {
//        ROS_ERROR_STREAM("No exist output directory");
//        std::cout << fname << std::endl;
//        return -1;
//    }
////    ofs_ro << "Seq00,Sec01,Seq02,Sec03,Seq04,Sec05,Seq06,Sec07,Seq08,Sec09,Sec10" << std::endl;

//    int first_sequence, last_sequence;
//    nh.getParam("first_sequence", first_sequence);
//    nh.getParam("last_sequence", last_sequence);

//     std::string str = "/ndt_mapping" + std::to_string(i);
//            nh.getParam(str + "resolution", res[i]);
//            nh.getParam(str + "step_size", step[i]);
//            nh.getParam(str + "transformation_epsilon", trans[i]);
//            nh.getParam(str + "maximu_iterations", max_ite[i]);
//            nh.getParam(str + "leaf_size", leaf[i]);
//            nh.getParam(str + "minimum_scan_range", min_scan[i]);
//            nh.getParam(str + "maximum_scan_range", max_scan[i]);
//            nh.getParam(str + "minimum_add_scan_shift", min_shift[i]);
//        }


    ErrorCalculator ec;
    ros::Subscriber sub_ground;
    ros::Subscriber sub_ndt;
    sub_ground = nh.subscribe("/groundtruth_pose/path", 1, &ErrorCalculator::callback_ground, &ec);
    sub_ndt = nh.subscribe("/current_path", 1, &ErrorCalculator::callback_ndt_path, &ec);

//    int exist_ndt_mapping;
    /* ndt_mapping が起動していなければloop */
//    while (true) {
//        std::string str = "/ndt_mapping_tku/exist_ndt_mapping";
//        nh.getParam(str, exist_ndt_mapping);
//        if (nh.hasParam(str) && exist_ndt_mapping == 1) break;
//        ROS_INFO("waiting for ndt_mapping nodes");
//        std::this_thread::sleep_for(std::chrono::seconds(1));
//    }


    /* exist_kitti_player が起動していなければloop */
    int exist_kitti_player;
    while (true) {
        std::string str = "/exist_kitti_player";
        nh.getParam(str, exist_kitti_player);
        if (nh.hasParam(str) && exist_kitti_player== 1) break;
        ROS_INFO("waiting for kitti_player nodes");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
    ROS_INFO_STREAM(__FILE__<<","<<__LINE__);


    while (true) {
        ros::spinOnce();
        std::string str_exist_kitti_player="/exist_kitti_player";
        int exist_kitti_player;
        nh.getParam(str_exist_kitti_player, exist_kitti_player);
        if (exist_kitti_player == 0){
          std::string str_entries_played="/entries_played";
          std::string str_ndt_mapping_tku_callbacks="/ndt_mapping_tku/callback_num";
          int entries_played,ndt_mapping_tku_callbacks;
          nh.getParam(str_entries_played,entries_played);
          nh.getParam(str_ndt_mapping_tku_callbacks,ndt_mapping_tku_callbacks);
          if (entries_played==ndt_mapping_tku_callbacks){
            break;
          }

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ROS_INFO("Start error calculation");

    ec.transformPath();
    double te = ec.getTransitionError();
    double re = ec.getRotationError();

    ROS_INFO("Done error calculation");


    std::string fname = OUTPUT_FNAME;
    std::ofstream ofs(fname.c_str(), std::ios_base::app);
    if (ofs.fail()) {
        ROS_ERROR_STREAM("No exist output directory");
        std::cout << fname << std::endl;
        return -1;
    }

    ofs << "," << te << "," << re << std::endl;

    ofs.close();

    nh.setParam("done_error_calculation", 1);
    nh.shutdown();

    return (0);
}
