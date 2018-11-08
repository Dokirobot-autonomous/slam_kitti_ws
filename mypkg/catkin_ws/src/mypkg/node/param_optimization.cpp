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
#define RESOLUTION {0.2,0.4,0.6,0.8,1.0}
#define STEP_SIZE {0.05,0.1,0.15,0.2}
#define TRANSFORMATION_EPSILON {0.01,0.02,0.03}
#define MAXIMUM_ITERATIONS {30}
#define LEAF_SIZE {0.2,0.4,0.6,0.8,1.0}
//#define RESOLUTION {1.0}
//#define STEP_SIZE {0.1}
//#define TRANSFORMATION_EPSILON {0.01}
//#define MAXIMUM_ITERATIONS {30}
//#define LEAF_SIZE {1.0}
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


        std::string fname_param=std::string(OUTPUT_FNAME)+std::string("params.csv");
        std::ofstream ofs_params(fname_param.c_str());
        if(ofs_params.fail()){
            ROS_ERROR_STREAM("No exist output directory");
            std::cout<<fname_param<<std::endl;
            return -1;
        }
        ofs_params<<"res,step,trans,max_itr,leaf,min_scan,max_scan,min_add"<<std::endl;

        std::string fname_tr=std::string(OUTPUT_FNAME)+std::string("error_trans.csv");
        std::ofstream ofs_tr(fname_tr.c_str());
        if(ofs_tr.fail()){
            ROS_ERROR_STREAM("No exist output directory");
            std::cout<<fname_param<<std::endl;
            return -1;
        }
        ofs_tr<<"Seq00,Sec01,Seq02,Sec03,Seq04,Sec05,Seq06,Sec07,Seq08,Sec09,Sec10"<<std::endl;

        std::string fname_ro=std::string(OUTPUT_FNAME)+std::string("error_rotate.csv");
        std::ofstream ofs_ro(fname_ro.c_str());
        if(ofs_ro.fail()){
            ROS_ERROR_STREAM("No exist output directory");
            std::cout<<fname_param<<std::endl;
            return -1;
        }
        ofs_ro<<"Seq00,Sec01,Seq02,Sec03,Seq04,Sec05,Seq06,Sec07,Seq08,Sec09,Sec10"<<std::endl;


    float resolution[]=RESOLUTION;
    float step_size[]=STEP_SIZE;
    float transformation_epsilon[]=TRANSFORMATION_EPSILON;
    float maximum_iterations[]=MAXIMUM_ITERATIONS;
    float leaf_size[]=LEAF_SIZE;
    float minimum_scan_range[]=MINIMUM_SCAN_RANGE;
    float maximum_scan_range[]=MAXIMUM_SCAN_RANGE;
    float minimum_add_shift[]=MINIMUM_ADD_SCAN_SHIFT;

    int resolution_size=sizeof(resolution)/sizeof(float);
    int step_size_size=sizeof(step_size)/sizeof(float);
    int transformation_epsilon_size=sizeof(transformation_epsilon)/sizeof(float);
    int maximum_iterations_size=sizeof(maximum_iterations)/sizeof(float);
    int leaf_size_size=sizeof(leaf_size)/sizeof(float);
    int minimum_scan_range_size=sizeof(minimum_scan_range)/sizeof(float);
    int maximum_scan_range_size=sizeof(maximum_scan_range)/sizeof(float);
    int minimum_add_shift_size=sizeof(minimum_add_shift)/sizeof(float);

    for(int ir=0;ir<resolution_size;ir++){
        for(int is=0;is<step_size_size;is++){
            for(int it=0;it<transformation_epsilon_size;it++){
                for(int imi=0;imi<maximum_iterations_size;imi++){
                    for(int il=0;il<leaf_size_size;il++){
                        for(int imins=0;imins<minimum_scan_range_size;imins++){
                            for(int imaxs=0;imaxs<maximum_scan_range_size;imaxs++){
                                for(int ima=0;ima<minimum_add_shift_size;ima++){

                                    float params[]={resolution[ir],step_size[is],transformation_epsilon[it],
                                                    maximum_iterations[imi],leaf_size[il],minimum_scan_range[imins],
                                                    maximum_scan_range[imaxs],minimum_add_shift[ima]};

                                    ROS_INFO_STREAM("Paramters: res,step,trans,max_itr,leaf,min_scan,max_scan,min_add");
                                    for(int i=0;i<7;i++){
                                        ROS_INFO_STREAM(params[i]);
                                    }

                                    nh.setParam("my_ndt_param/resolution",params[0]);
                                    nh.setParam("my_ndt_param/step_size",params[1]);
                                    nh.setParam("my_ndt_param/transformation_epsilon",params[2]);
                                    nh.setParam("my_ndt_param/maximum_iterations",params[3]);
                                    nh.setParam("my_ndt_param/leaf_size",params[4]);
                                    nh.setParam("my_ndt_param/minimum_scan_range",params[5]);
                                    nh.setParam("my_ndt_param/maximum_scan_range",params[6]);
                                    nh.setParam("my_ndt_param/minimum_add_shift",params[7]);

                                    std::vector<float> trans_errors,rotate_errors;

                                    for(unsigned int seq=0;seq<11;seq++){
                                        ErrorCalculator ec;

                                        /* Subscribe */
                                        ros::Subscriber sub_ground = nh.subscribe("/groundtruth_pose/path", 1, &ErrorCalculator::callback_ground, &ec);
                                        ros::Subscriber sub_ndt = nh.subscribe("/current_path", 1, &ErrorCalculator::callback_ndt_path, &ec);

                                        nh.setParam("current_sequence",(int)seq);
                                        nh.setParam("start_mapping",1);

                                        int exist_ndt_mapping;
                                        /* ndt_mapping が起動していなければloop */
                                        while(true){
                                            nh.getParam("exist_ndt_mapping",exist_ndt_mapping);
                                            if(nh.hasParam("exist_ndt_mapping") && exist_ndt_mapping==1) break;
                                            std::this_thread::sleep_for(std::chrono::seconds(1));
                                        }

                                        while(true){
                                            nh.getParam("exist_ndt_mapping",exist_ndt_mapping);
                                            ros::spinOnce();
                                        if(exist_ndt_mapping==0) break;
                                            std::this_thread::sleep_for(std::chrono::seconds(1));
                                        }

                                        ec.transformPath();
                                        float te=ec.getTransitionError();
                                        float re=ec.getRotationError();
                                        trans_errors.push_back(te);
                                        rotate_errors.push_back(re);
                                    }

                                    for(unsigned int i=0;i<8;i++){
                                        if(i!=7){
                                            ofs_params<<params[i]<<",";
                                        }
                                        else{
                                            ofs_params<<params[i]<<std::endl;
                                        }
                                    }
                                    for(unsigned int i=0;i<trans_errors.size();i++){
                                        ROS_INFO("a");
                                        if(i!=(trans_errors.size()-1)){
                                            ROS_INFO("b");
                                            ofs_tr<<trans_errors[i]<<",";
                                        }
                                        else{
                                            ROS_INFO("c");
                                            ofs_tr<<trans_errors[i]<<std::endl;
                                        }
                                    }
                                    for(unsigned int i=0;i<rotate_errors.size();i++){
                                        if(i!=rotate_errors.size()-1){
                                            ofs_ro<<rotate_errors[i]<<",";
                                        }
                                        else{
                                            ofs_ro<<rotate_errors[i]<<std::endl;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    nh.setParam("exist_param_optimization",0);


    return (0);
}