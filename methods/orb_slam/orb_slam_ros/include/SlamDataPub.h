/**
* Description: This file is part of parkingEnvSensing.
* Date: 2017-06-20
* Final Edit: 2017-06-20
*/

#ifndef DATAPUB_H
#define DATAPUB_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>  


#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class SlamDataPub
{
public:
    SlamDataPub(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Map *pMap);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetCurrentCameraPoseCovariance(const cv::Mat &Cov);

private:

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    Map* mpMap;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
    
    std::mutex mMutexCamera;
    cv::Mat mCameraPose;
    cv::Mat mCameraPoseCovariance;

    //ros::Publisher path_pub_;
    ros::Publisher CamPose_pub_;
    ros::Publisher CamPoseWithCov_pub_;
    ros::Publisher VehiclePose_pub_;
    ros::Publisher VehiclePoseWithCov_pub_;
    ros::Publisher CamPath_pub_;
    ros::Publisher VehiclePath_pub_;
    ros::Publisher AllPointCloud_pub_;
    ros::Publisher RefPointCloud_pub_;  
    
    image_transport::Publisher DrawFrame_pub_;
    tf::TransformBroadcaster Vehicle2Ground_broadcaster_;
    tf::TransformBroadcaster FusionMap2OrbMap_broadcaster_;

    //tf::TransformBroadcaster broadcaster;
    ros::NodeHandle nh;
    
    bool mbGetNewCamPose;
    bool mbGetNewCamPoseWithCov;

    void TrackingDataPub();   
    void GetCurrentROSCameraMatrix(geometry_msgs::PoseStamped &cam_pose);
    void GetCurrentROSCameraWithCovarianceMatrix(geometry_msgs::PoseWithCovarianceStamped &cam_pose_with_cov);
    void GetCurrentROSVehicleMatrix(geometry_msgs::PoseStamped &vehicle_pose);
    void GetCurrentROSTrajectories(nav_msgs::Path &cam_path, nav_msgs::Path &vehicle_path);    
    
    void PointCloudPub();
    void GetCurrentROSAllPointCloud( sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud);
    
    void DrawFramePub();
    
    Eigen::Matrix3f mInitCam2Ground_R;
    Eigen::Vector3f mInitCam2Ground_t;
    Eigen::Matrix4f mTrans_cam2ground;   //calibration
    
    Eigen::Matrix3f mCam2Vehicle_R;  // camera is stationary to vehicle
    Eigen::Vector3f mCam2Vehicle_t;
    Eigen::Matrix4f mTrans_cam2vehicle;    //

    Eigen::Matrix4f mCam2GroundNow_T;   
    Eigen::Matrix4f mVehicle2GroundNow_T;   
    
};

}


#endif // DATAPUB_H
	

