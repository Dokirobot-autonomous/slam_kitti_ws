/**
* Description: This file is part of parkingEnvSensing.
* Date: 2017-06-20
* Final Edit: 2017-06-20
*/

/* カメラのPoseデータはTracking.ccの "mpSlamDataPub->SetCurrentCameraPose" から取得 */

#include "SlamDataPub.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


#include <opencv2/core/eigen.hpp>

#include <mutex>

namespace ORB_SLAM2
{

SlamDataPub::SlamDataPub(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Map *pMap):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking), mpMap(pMap),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false),mbGetNewCamPose(false),mbGetNewCamPoseWithCov(false),mTimeStamp(0.0)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }
    
    // camera under ground      
    mInitCam2Ground_R << 1,0,0,0,0,1,0,-1,0;  // camera coordinate represented in ground coordinate system
    mInitCam2Ground_t.setZero();     
    mTrans_cam2ground.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    mTrans_cam2ground.block<3,3>(0,0) = mInitCam2Ground_R;
    mTrans_cam2ground.block<3,1>(0,3) = mInitCam2Ground_t;  //< block_rows, block_cols >(pos_row, pos_col)

    // camera under vehicle, 
    mCam2Vehicle_R << 0,0,1,-1,0,0,0,-1,0;  // camera coordinate represented in vehicle coordinate system
    mCam2Vehicle_t.setZero();
    mTrans_cam2vehicle.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    mTrans_cam2vehicle.block<3,3>(0,0) = mCam2Vehicle_R;
    mTrans_cam2vehicle.block<3,1>(0,3) = mCam2Vehicle_t;  //< block_rows, block_cols >(pos_row, pos_col)

    
}

void SlamDataPub::TrackingDataPub()
{
    geometry_msgs::PoseStamped camPose2Ground;
    geometry_msgs::PoseWithCovarianceStamped camPoseWithCov2Ground;
    geometry_msgs::PoseStamped vehiclePose2Ground;
    geometry_msgs::PoseWithCovarianceStamped vehiclePoseWithCov2Ground;

    nav_msgs::Path cameraPath, vehiclePath;
    while(1)
    {
        if(mbGetNewCamPose && mbGetNewCamPoseWithCov){

	      GetCurrentROSCameraMatrix(camPose2Ground);
		  GetCurrentROSCameraWithCovarianceMatrix(camPoseWithCov2Ground);
	      GetCurrentROSVehicleMatrix(vehiclePose2Ground);
	      GetCurrentROSVehicleWithCovarianceMatrix(vehiclePoseWithCov2Ground);
	      GetCurrentROSTrajectories(cameraPath, vehiclePath);
	      CamPose_pub_.publish(camPose2Ground);
		  CamPoseWithCov_pub_.publish(camPoseWithCov2Ground);
		  VehiclePose_pub_.publish(vehiclePose2Ground);
		  VehiclePoseWithCov_pub_.publish(vehiclePoseWithCov2Ground);
	      CamPath_pub_.publish(cameraPath);   // KeyFrames
	      VehiclePath_pub_.publish(vehiclePath);
	      
	      float tf_q_x = vehiclePose2Ground.pose.orientation.x;
	      float tf_q_y = vehiclePose2Ground.pose.orientation.y;
	      float tf_q_z = vehiclePose2Ground.pose.orientation.z;
	      float tf_q_w = vehiclePose2Ground.pose.orientation.w;
	      float tf_x = vehiclePose2Ground.pose.position.x;
	      float tf_y = vehiclePose2Ground.pose.position.y;
	      float tf_z = vehiclePose2Ground.pose.position.z;

	      std::cout<<mTimeStamp <<std::endl;
	      ros::Time t;
	      t.fromSec(mTimeStamp);
	      Vehicle2Ground_broadcaster_.sendTransform(
		  tf::StampedTransform(
		  tf::Transform(tf::Quaternion(tf_q_x,tf_q_y,tf_q_z,tf_q_w), tf::Vector3(tf_x, tf_y, tf_z)),
		  t,"orb_map", "vehicle"));
	    
	  }
	  if(CheckFinish())
	      break;  
	   usleep(1*1000); 
    }
   
}

void SlamDataPub::PointCloudPub()
{
    sensor_msgs::PointCloud2 allMapPoints;
    sensor_msgs::PointCloud2 referenceMapPoints;
    while(1)
    {
// 	  if(mbGetNewCamPose)
// 	  {
	      GetCurrentROSAllPointCloud(allMapPoints, referenceMapPoints);
	      AllPointCloud_pub_.publish(allMapPoints);
	      RefPointCloud_pub_.publish(referenceMapPoints);
// 	  }
	  if(CheckFinish())
	      break;  
	  usleep(mT*1000/2); 
    }
    
}

void SlamDataPub::DrawFramePub()
{
    cv_bridge::CvImage cvi;
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.header.stamp.sec = mTimeStamp;
    while(1)
    {
// 	if(mbGetNewCamPose)
// 	{  
	cv::Mat img = mpFrameDrawer->DrawFrame();
	cv::imshow("Current Frame",img);
	cv::waitKey(mT/2);
	cvi.image = img;
	sensor_msgs::Image im;
	cvi.toImageMsg(im);
	DrawFrame_pub_.publish(im);
//	}
	if(CheckFinish())
	    break;  
	//usleep(1*1000); 
    }
 
}

void SlamDataPub::Run()
{
    mbFinished = false;
    mbStopped = false;

    CamPose_pub_ = nh.advertise<geometry_msgs::PoseStamped >("camera_pose",1);
    CamPoseWithCov_pub_=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("camera_pose_with_covariance",1);
    VehiclePose_pub_ = nh.advertise<geometry_msgs::PoseStamped >("vehicle_pose",1);
    VehiclePoseWithCov_pub_=nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vehicle_pose_with_covariance",1);
    CamPath_pub_ = nh.advertise<nav_msgs::Path>("camera_path",1);
    VehiclePath_pub_ = nh.advertise<nav_msgs::Path>("vehicle_path",1);
    AllPointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_all",1);
    RefPointCloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_ref",1);
    
    image_transport::ImageTransport it_(nh);
    DrawFrame_pub_ = it_.advertise("/frame_now", 1);
       
    thread threadCamPosePub(&SlamDataPub::TrackingDataPub,this);   
    thread threadPointCloudPub(&SlamDataPub::PointCloudPub,this);  
    thread threadDrawFramePub(&SlamDataPub::DrawFramePub,this); 
    
    threadCamPosePub.join(); 
    threadPointCloudPub.join();

    SetFinish();
}

void SlamDataPub::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool SlamDataPub::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void SlamDataPub::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool SlamDataPub::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void SlamDataPub::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool SlamDataPub::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool SlamDataPub::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void SlamDataPub::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void SlamDataPub::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbGetNewCamPose = true;
}

void SlamDataPub::SetCurrentCameraPose(const cv::Mat &Tcw,double timestamp)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
    mbGetNewCamPose = true;
    mTimeStamp=timestamp;
}

    void SlamDataPub::SetCurrentCameraPoseCovariance(const cv::Mat &Cov)
{
	unique_lock<mutex> lock(mMutexCamera);
	mCameraPoseCovariance = Cov.clone();
	mbGetNewCamPoseWithCov = true;
}


void SlamDataPub::GetCurrentROSCameraMatrix(geometry_msgs::PoseStamped &cam_pose)
{
      if(!mCameraPose.empty())
      {
	  Eigen::Matrix4f cam_pose2firstcam;
 	  Eigen::Matrix4f cam_pose2ground;
 	  {
	      unique_lock<mutex> lock(mMutexCamera);
	      cv2eigen(mCameraPose.inv(),cam_pose2firstcam);
	      cam_pose2ground = mTrans_cam2ground * cam_pose2firstcam;
	      {
		  mCam2GroundNow_T = cam_pose2ground;
	      }
	  }

 	  cam_pose.pose.position.x = cam_pose2ground(0,3);
	  cam_pose.pose.position.y  = cam_pose2ground(1,3);
	  cam_pose.pose.position.z  = cam_pose2ground(2,3);
	   
	  Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
	  Eigen::Quaternionf q(Rwc);
	  cam_pose.pose.orientation.x = q.x();
	  cam_pose.pose.orientation.y = q.y();
	  cam_pose.pose.orientation.z = q.z();
	  cam_pose.pose.orientation.w = q.w();
	  
	  cam_pose.header.frame_id = "orb_map";
	  cam_pose.header.stamp.fromSec(mTimeStamp);
//	  cam_pose.header.stamp.sec=mTimeStamp;
//	  cam_pose.header.stamp = ros::Time::now();
	  
	  mbGetNewCamPose = false;
      }
      
}

void SlamDataPub::GetCurrentROSCameraWithCovarianceMatrix(geometry_msgs::PoseWithCovarianceStamped &cam_pose_with_cov)
{
	if(!mCameraPose.empty() && !mCameraPoseCovariance.empty())
	{
		Eigen::Matrix4f cam_pose2firstcam;
		Eigen::Matrix4f cam_pose2ground;
		{
			unique_lock<mutex> lock(mMutexCamera);
			cv2eigen(mCameraPose.inv(),cam_pose2firstcam);
			cam_pose2ground = mTrans_cam2ground * cam_pose2firstcam;
			{
				mCam2GroundNow_T = cam_pose2ground;
			}
		}

		cam_pose_with_cov.pose.pose.position.x = cam_pose2ground(0,3);
		cam_pose_with_cov.pose.pose.position.y  = cam_pose2ground(1,3);
		cam_pose_with_cov.pose.pose.position.z  = cam_pose2ground(2,3);

		Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
		Eigen::Quaternionf q(Rwc);
		cam_pose_with_cov.pose.pose.orientation.x = q.x();
		cam_pose_with_cov.pose.pose.orientation.y = q.y();
		cam_pose_with_cov.pose.pose.orientation.z = q.z();
		cam_pose_with_cov.pose.pose.orientation.w = q.w();


//		Eigen::Matrix<float,6,6> cam_pose_cov2firstcam;
//        Eigen::Matrix<float,6,6> cam_pose_cov2ground;
//		{
//			unique_lock<mutex> lock(mMutexCamera);
//			cv2eigen(mCameraPoseCovariance,cam_pose_cov2firstcam);
//			cam_pose_cov2ground = cam_pose_cov2firstcam;
////			cam_pose2ground = mTrans_cam2ground * cam_pose_cov2firstcam;
//			{
//				mCam2GroundNow_T = cam_pose_cov2ground;
//			}
//		}
//
//		for(int i=0;i<6;i++){
//			for(int j=0;j<6;j++){
//				cam_pose_with_cov.pose.covariance[i*6+j]=cam_pose_cov2ground(i,j);
//			}
//		}

        for(int i=0;i<6;i++){
            for(int j=0;j<6;j++){
                cam_pose_with_cov.pose.covariance[i*6+j]=mCameraPoseCovariance.at<float>(j,i);
            }
        }


		cam_pose_with_cov.header.frame_id = "orb_map";
		cam_pose_with_cov.header.stamp.fromSec(mTimeStamp);

		mbGetNewCamPoseWithCov = false;
	}




}

void SlamDataPub::GetCurrentROSVehicleMatrix(geometry_msgs::PoseStamped &vehicle_pose)
{
    if (!mCameraPose.empty()) {
        Eigen::Matrix4f vehicle_pose2ground;
        {
            vehicle_pose2ground = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();
        }
        {
            mVehicle2GroundNow_T = vehicle_pose2ground;
        }
        vehicle_pose.pose.position.x = vehicle_pose2ground(0, 3);
        vehicle_pose.pose.position.y = vehicle_pose2ground(1, 3);
        vehicle_pose.pose.position.z = vehicle_pose2ground(2, 3);

        Eigen::Matrix3f Rwc = vehicle_pose2ground.block<3, 3>(0, 0);
        Eigen::Quaternionf q(Rwc);
        vehicle_pose.pose.orientation.x = q.x();
        vehicle_pose.pose.orientation.y = q.y();
        vehicle_pose.pose.orientation.z = q.z();
        vehicle_pose.pose.orientation.w = q.w();

        vehicle_pose.header.frame_id = "orb_map";
        vehicle_pose.header.stamp.fromSec(mTimeStamp);
    }
}

void SlamDataPub::GetCurrentROSVehicleWithCovarianceMatrix(geometry_msgs::PoseWithCovarianceStamped &vehicle_pose_with_cov) {

    if(!mCameraPose.empty() && !mCameraPoseCovariance.empty()){
        Eigen::Matrix4f vehicle_pose2ground;
        {
            vehicle_pose2ground = mCam2GroundNow_T * mTrans_cam2vehicle.inverse();
        }
        {
            mVehicle2GroundNow_T = vehicle_pose2ground;
        }
        vehicle_pose_with_cov.pose.pose.position.x = vehicle_pose2ground(0, 3);
        vehicle_pose_with_cov.pose.pose.position.y = vehicle_pose2ground(1, 3);
        vehicle_pose_with_cov.pose.pose.position.z = vehicle_pose2ground(2, 3);

        Eigen::Matrix3f Rwc = vehicle_pose2ground.block<3, 3>(0, 0);
        Eigen::Quaternionf q(Rwc);
        vehicle_pose_with_cov.pose.pose.orientation.x = q.x();
        vehicle_pose_with_cov.pose.pose.orientation.y = q.y();
        vehicle_pose_with_cov.pose.pose.orientation.z = q.z();
        vehicle_pose_with_cov.pose.pose.orientation.w = q.w();

//        /* covariance */
//        Eigen::MatrixXf trans(6,6);
//        trans.block<3,3>(0,0)=mTrans_cam2vehicle.block<3,3>(0,0);
//        trans.block<3,3>(3,0)=trans.block<3,3>(0,3)=Eigen::MatrixXf::Zero(3,3);
//        trans.block<3,3>(3,3)=Eigen::MatrixXf::Identity(3,3);
//        Eigen::MatrixXf cam_cov(6,6);
//        for(int i=0;i<6;i++){
//            for(int j=0;j<6;j++){
//                cam_cov(j,i)=mCameraPoseCovariance.at<float>(j,i);
//            }
//        }
//        Eigen::MatrixXf vehicle_cov(6,6);
//        vehicle_cov=trans*cam_cov*trans.transpose();

        for(int i=0;i<6;i++){
            for(int j=0;j<6;j++){
                vehicle_pose_with_cov.pose.covariance[i*6+j]=mCameraPoseCovariance.at<float>(j,i);
            }
        }

        vehicle_pose_with_cov.header.frame_id = "orb_map";
        vehicle_pose_with_cov.header.stamp.fromSec(mTimeStamp);


    }


}

void SlamDataPub::GetCurrentROSTrajectories(nav_msgs::Path &cam_path, nav_msgs::Path &vehicle_path)
{
      if(!mCameraPose.empty())
      {  
	  nav_msgs::Path cam_path_temp;
	  nav_msgs::Path vehicle_path_temp;
	  
	  geometry_msgs::PoseStamped cam_pose;
	   geometry_msgs::PoseStamped vehicle_pose;
	  
	  vector<cv::Mat> currentTrajectory;	
	  mpSystem->GetCurrentTrajectory(currentTrajectory);
	  
	  Eigen::Matrix4f cam_pose_temp;
	  
	  for(auto mt:currentTrajectory) // no need to inverse
	  {
	      cv2eigen(mt,cam_pose_temp);
	      
	      Eigen::Matrix4f cam_pose2ground = mTrans_cam2ground * cam_pose_temp;
	      Eigen::Matrix4f vehicle_pose2ground = cam_pose2ground * mTrans_cam2vehicle.inverse();
	      
	      cam_pose.pose.position.x = cam_pose2ground(0,3);
	      cam_pose.pose.position.y = cam_pose2ground(1,3);
	      cam_pose.pose.position.z = cam_pose2ground(2,3);
	      Eigen::Matrix3f Rwc = cam_pose2ground.block<3,3>(0,0);
	      Eigen::Quaternionf q(Rwc);	      
	      cam_pose.pose.orientation.x = q.x();
	      cam_pose.pose.orientation.y = q.y();
	      cam_pose.pose.orientation.z = q.z();
	      cam_pose.pose.orientation.w = q.w();
	      
	      vehicle_pose.pose.position.x = vehicle_pose2ground(0,3);
	      vehicle_pose.pose.position.y = vehicle_pose2ground(1,3);
	      vehicle_pose.pose.position.z = vehicle_pose2ground(2,3);
	      Eigen::Matrix3f Rwc2 = vehicle_pose2ground.block<3,3>(0,0);
	      Eigen::Quaternionf q2(Rwc2);	      
	      vehicle_pose.pose.orientation.x = q2.x();
	      vehicle_pose.pose.orientation.y = q2.y();
	      vehicle_pose.pose.orientation.z = q2.z();
	      vehicle_pose.pose.orientation.w = q2.w();
	      
	      vehicle_path_temp.poses.push_back(vehicle_pose);
	      cam_path_temp.poses.push_back(cam_pose);	     
	  }
 	  cam_path_temp.header.frame_id = "orb_map";
 	  cam_path_temp.header.stamp.fromSec(mTimeStamp);
	  vehicle_path_temp.header.frame_id = "orb_map";
	  vehicle_path_temp.header.stamp.fromSec(mTimeStamp);
	  
	  cam_path = cam_path_temp;
	  vehicle_path = vehicle_path_temp;
      }
}

void SlamDataPub::GetCurrentROSAllPointCloud( sensor_msgs::PointCloud2 &all_point_cloud, sensor_msgs::PointCloud2 &ref_point_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_all( new pcl::PointCloud<pcl::PointXYZRGBA> );  
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ref( new pcl::PointCloud<pcl::PointXYZRGBA> );     
    
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;
	
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        pcl::PointXYZRGBA p1;
	Eigen::Vector4f p1_temp, p1_temp_t;
	p1_temp(0) = pos.at<float>(0);
	p1_temp(1) = pos.at<float>(1);
	p1_temp(2) = pos.at<float>(2);
	p1_temp(3) = 1; 
	p1_temp_t = mTrans_cam2ground * p1_temp;	
	p1.x = p1_temp_t(0);
	p1.y = p1_temp_t(1);
	p1.z = p1_temp_t(2);
	p1.b = 255;
	p1.g = 255;
	p1.r = 255;
	p1.a = 255;
	cloud_all->points.push_back( p1 );
    }
    pcl::PCLPointCloud2 pcl_pc1;
    pcl::toPCLPointCloud2(*cloud_all, pcl_pc1);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc1, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header.frame_id = "orb_map";
    all_point_cloud.header.stamp.fromSec(mTimeStamp);
  
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        pcl::PointXYZRGBA p2;
	Eigen::Vector4f p2_temp, p2_temp_t;
	p2_temp(0) = pos.at<float>(0);
	p2_temp(1) = pos.at<float>(1);
	p2_temp(2) = pos.at<float>(2);
	p2_temp(3) = 1;
	p2_temp_t = mTrans_cam2ground * p2_temp;	
	p2.x = p2_temp_t(0);
	p2.y = p2_temp_t(1);
	p2.z = p2_temp_t(2);
	p2.b = 0;
	p2.g = 0;
	p2.r = 255;
	p2.a = 255;
	cloud_ref->points.push_back( p2 );
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud_ref, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header.frame_id = "orb_map";
    ref_point_cloud.header.stamp.fromSec(mTimeStamp);

}

}


