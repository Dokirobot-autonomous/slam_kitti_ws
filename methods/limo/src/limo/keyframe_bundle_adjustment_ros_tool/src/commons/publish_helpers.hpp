// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <Eigen/Eigen>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>

#include "keyframe_bundle_adjustment_ros_tool/PathWithCovariance.h"

// for publishing landmarks
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tf2_eigen/tf2_eigen.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>



namespace keyframe_bundle_adjustment_ros_tool {

namespace helpers {

/**
 * @brief toGeometryMsg, convert to geometry_msg style  pose
 * @param p, pose eigen isometry
 * @return geometry_msg style pose
 */
template <int TransformType>
void toGeometryMsg(geometry_msgs::Pose& out, const Eigen::Transform<double, 3, TransformType>& pose) {
    // convert accumulated_pose_ to transformStamped
    Eigen::Quaterniond rot_quat(pose.rotation());

    out.position.x = pose.translation().x();
    out.position.y = pose.translation().y();
    out.position.z = pose.translation().z();

    out.orientation.x = rot_quat.x();
    out.orientation.y = rot_quat.y();
    out.orientation.z = rot_quat.z();
    out.orientation.w = rot_quat.w();
}

///@brief overload for transform
template <int TransformType>
void toGeometryMsg(geometry_msgs::Transform& out, const Eigen::Transform<double, 3, TransformType>& pose) {
    // convert accumulated_pose_ to transformStamped
    Eigen::Quaterniond rot_quat(pose.rotation());

    out.translation.x = pose.translation().x();
    out.translation.y = pose.translation().y();
    out.translation.z = pose.translation().z();

    out.rotation.x = rot_quat.x();
    out.rotation.y = rot_quat.y();
    out.rotation.z = rot_quat.z();
    out.rotation.w = rot_quat.w();
}

namespace {
using Category = keyframe_bundle_adjustment::LandmarkCategorizatonInterface::Category;
float getColorVal(const keyframe_bundle_adjustment::LandmarkId& lm_id,
                  const std::map<keyframe_bundle_adjustment::LandmarkId, Category>& lm_categories) {
    // Choose value of color according to category.
    // Outliers are not present in category -> val=20
    float val = 80.;
    auto it = lm_categories.find(lm_id);
    if (it != lm_categories.cend()) {
        switch (it->second) {
        case Category::FarField:
            val = 250.;
            break;
        case Category::MiddleField:
            val = 180.;
            break;
        case Category::NearField:
            val = 120.;
            break;
        default:
            break;
        }
    }
    return val;
}
}

/**
 * @brief publishLandmarks, publish the landmarks as sensor_msgs::PointCloud2 cloud; frame_id is
 * global coordinate system; timestamp is the one of last keyframe
 * @param landmarks_publisher
 * @param bundle_adjuster
 * @param tf_parent_frame_id
 */
void publishLandmarks(ros::Publisher& landmarks_publisher,
                      keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                      std::string tf_parent_frame_id) {
    using PointType = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointType>;
    using namespace keyframe_bundle_adjustment;

    ros::Time stamp;
    stamp.fromNSec(bundle_adjuster->getKeyframe().timestamp_);

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "/" + tf_parent_frame_id;
    msg->header.stamp = stamp.toNSec();
    msg->height = 1;


    // plot selected landmarks in green
    const auto& selected_lms = bundle_adjuster->getSelectedLandmarkConstPtrs();

    // get difference to active landmarks -> outlier
    const auto& active_lms = bundle_adjuster->getActiveLandmarkConstPtrs();
    // size of pointcloud should be equal to atctive_lms
    msg->width = active_lms.size();

    // Get categories to color landmarks wether if the are in near, middle or far field
    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> lm_categories =
        bundle_adjuster->landmark_selector_->getLandmarkCategories();

    // Plot inliers(selected lms) green.
    for (const auto& lm_id_pos : selected_lms) {
        const auto& lm = *(lm_id_pos.second);

        float val = getColorVal(lm_id_pos.first, lm_categories);

        std::array<float, 3> color_rgb{0., val, 0.};

        if (lm.is_ground_plane) {
            color_rgb = std::array<float, 3>{0., val, val};
        }
        if (lm.has_measured_depth) {
            color_rgb = std::array<float, 3>{val, val, 0.};
        }

        PointType p;
        p.x = lm.pos[0];
        p.y = lm.pos[1];
        p.z = lm.pos[2];
        p.r = color_rgb[0];
        p.g = color_rgb[1];
        p.b = color_rgb[2];

        msg->points.push_back(p);
    }

    // see https://www.fluentcpp.com/2017/01/09/know-your-algorithms-algos-on-sets/
    std::map<LandmarkId, Landmark::ConstPtr> diff;
    std::set_difference(active_lms.cbegin(),
                        active_lms.cend(),
                        selected_lms.cbegin(),
                        selected_lms.cend(),
                        std::inserter(diff, diff.begin()),
                        [](const auto& a, const auto& b) { return a.first < b.first; });

    // plot outliers red
    for (const auto& lm_id : diff) {
        const auto& lm = *lm_id.second;

        std::array<float, 3> color_rgb{0., 0., 0.};

        if (lm.is_ground_plane) {
            color_rgb = std::array<float, 3>{100., 0., 0.};
        }
        if (lm.has_measured_depth) {
            color_rgb = std::array<float, 3>{100., 0., 100.};
        }

        PointType p;
        p.x = lm.pos[0];
        p.y = lm.pos[1];
        p.z = lm.pos[2];
        p.r = color_rgb[0];
        p.g = color_rgb[1];
        p.b = color_rgb[2];

        msg->points.push_back(p);
    }

    landmarks_publisher.publish(msg);
}
/**
 * @brief convertToOutmsg, convert from keyframe poses to output
 * @param cur_stamp, current timestamp of measurement
* @param bundle_adjuster
 * @return msg to be published as arra of pose deltas
 */

// mapping_msgs_ros::PoseDeltaConstraintArray convertToOutmsg(
//     ros::Time cur_stamp,
//     keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
//     std::string calib_source_frame_id) {

//     using OutputMsg = mapping_msgs_ros::PoseDeltaConstraintArray;
//     OutputMsg out_msg;
//     out_msg.header.stamp = cur_stamp;
//     out_msg.header.frame_id = calib_source_frame_id;

//     // first kf is coordinate system get timestamp
//     auto it =
//         std::min_element(bundle_adjuster->keyframes_.cbegin(),
//                          bundle_adjuster->keyframes_.cend(),
//                          [](const auto& a, const auto& b) { return a.second->timestamp_ < b.second->timestamp_; });
//     ros::Time timestamp_origin;
//     timestamp_origin.fromNSec(it->second->timestamp_);
//     // convert all keyframe poses to delta constraints
//     for (const auto& kf : bundle_adjuster->keyframes_) {
//         mapping_msgs_ros::PoseDeltaConstraint pose_delta_constraint;
//         ros::Time ts;
//         ts.fromNSec(kf.second->timestamp_);
//         pose_delta_constraint.header.stamp = ts;
//         pose_delta_constraint.header.frame_id = calib_source_frame_id;

//         // convert the eigen stuff to geometry_msgs
//         Eigen::Affine3d current_motion{kf.second->getEigenPose()};
//         pose_delta_constraint.pose_delta.pose = tf2::toMsg(current_motion);
//         // poses are defined from current from to origin
//         pose_delta_constraint.id_from = ts.toNSec();
//         pose_delta_constraint.id_to = timestamp_origin.toNSec();

//         pose_delta_constraint.stamp_from = ts;
//         pose_delta_constraint.stamp_to = timestamp_origin;

//         out_msg.constraints.push_back(pose_delta_constraint);
//     }

//     return out_msg;
// }

void publishPlanes(ros::Publisher& plane_publisher,
                   keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                   std::string tf_parent_frame_id) {
    visualization_msgs::MarkerArray out;
    double plane_size = 6.;
    //    std::cout << "Plane positions:" << std::endl;
    int count = 0;
    for (const auto& id__kf_ptr : bundle_adjuster->keyframes_) {
        if (id__kf_ptr.second->local_ground_plane_.distance > -0.1) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = tf_parent_frame_id;
            ros::Time cur_ts;
            cur_ts.fromNSec(id__kf_ptr.second->timestamp_);
            marker.header.stamp = cur_ts;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = std::to_string(count);
            marker.scale.x = plane_size;
            marker.scale.y = plane_size;
            marker.scale.z = 0.1;
            marker.color.a = 0.3; // Don't forget to set the alpha!
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.0;

            Eigen::Map<Eigen::Vector3d> plane_dir(id__kf_ptr.second->local_ground_plane_.direction.data());
            Eigen::Quaterniond ori;
            ori.setFromTwoVectors(Eigen::Vector3d(0., 0., 1.), plane_dir);
            marker.pose.orientation.x = ori.x();
            marker.pose.orientation.y = ori.y();
            marker.pose.orientation.z = ori.z();
            marker.pose.orientation.w = ori.w();

            Eigen::Vector3d pos = id__kf_ptr.second->getEigenPose().inverse().translation() -
                                  plane_dir * id__kf_ptr.second->local_ground_plane_.distance;
            marker.pose.position.x = pos.x();
            marker.pose.position.y = pos.y();
            marker.pose.position.z = pos.z();

            //            std::cout << pos.transpose() << "---" << plane_dir.transpose() << std::endl;
            out.markers.push_back(marker);
            count++;

            {
                visualization_msgs::Marker marker_arr;
                marker_arr.header.frame_id = tf_parent_frame_id;
                marker_arr.header.stamp = cur_ts;
                marker_arr.type = visualization_msgs::Marker::ARROW;
                marker_arr.action = visualization_msgs::Marker::ADD;
                marker_arr.ns = std::to_string(count);
                marker_arr.scale.x = 3.;
                marker_arr.scale.y = 0.3;
                marker_arr.scale.z = 0.3;
                marker_arr.color.a = 0.7; // Don't forget to set the alpha!
                marker_arr.color.r = 0.5;
                marker_arr.color.g = 0.5;
                marker_arr.color.b = 0.0;

                Eigen::Quaterniond ori_arr;
                ori_arr.setFromTwoVectors(Eigen::Vector3d(1., 0., 0.), plane_dir);
                marker_arr.pose.orientation.x = ori_arr.x();
                marker_arr.pose.orientation.y = ori_arr.y();
                marker_arr.pose.orientation.z = ori_arr.z();
                marker_arr.pose.orientation.w = ori_arr.w();

                marker_arr.pose.position = marker.pose.position;

                out.markers.push_back(marker_arr);
                count++;
            }
        }
    }

    //    std::cout << "Markers size=" << out.markers.size() << std::endl;

    plane_publisher.publish(out);
}

/**
 * @brief publishPath, publish
 * two nav_msgs::path that can be
 * shown in rviz, one for frames that are optimized at the moemnt and one for all frames;frame id is
 * same as tf_parent, timestamp of msg is timestamp of the most
 * current keyframe;
 * @param path_publisher
 * @param active_path_publisher
 * @param bundle_adjuster
 */
void publishPaths(ros::Publisher& path_publisher,
                  ros::Publisher& active_path_publisher,
                  keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                  std::string tf_parent_frame_id) {
    nav_msgs::Path path_msg;

    // timestamp of msg is same as last keyframe
    ros::Time cur_ts;
    cur_ts.fromNSec(bundle_adjuster->getKeyframe().timestamp_);
    path_msg.header.stamp = cur_ts;
    // frame id of msg is same as tf_parent without tf2 convention
    path_msg.header.frame_id = "/" + tf_parent_frame_id;

    nav_msgs::Path active_path_msg = path_msg;

    for (const auto& kf : bundle_adjuster->keyframes_) {
        geometry_msgs::PoseStamped cur_pose;
        ros::Time p_ts;
        p_ts.fromNSec(kf.second->timestamp_);
        cur_pose.header.stamp = p_ts;
        cur_pose.header.frame_id = path_msg.header.frame_id;
        //            ROS_DEBUG_STREAM("origin in cur_pose=\n"
        //                             << kf.second.getEigenPose().translation().transpose());

        toGeometryMsg(cur_pose.pose, kf.second->getEigenPose().inverse()); // second：std::pairの2番目

        path_msg.poses.push_back(cur_pose);

        if (kf.second->is_active_) {
            active_path_msg.poses.push_back(cur_pose);
        }
    }

    path_publisher.publish(path_msg);
    active_path_publisher.publish(active_path_msg);
}

void quatCov2rpyCov(double *pose_quat,double *quat_cov,double *rpy_cov){

//  for(int i=0;i<7;i++){
//    for(int j=0;j<7;j++){
//      std::cout<<*(quat_cov+j*7+i)<<"\t";
//    }
//    std::cout<<std::endl;
//  }

  double dd=10e-6; // 推定結果が0.1~1.0の間なので，影響を及ぼさない範囲で変更
  tf::Quaternion quat_origin(*(pose_quat+3),*(pose_quat+4),*(pose_quat+5),*(pose_quat+6));
  tf::Quaternion quat_qxdd(*(pose_quat+3)+dd,*(pose_quat+4),*(pose_quat+5),*(pose_quat+6));
  tf::Quaternion quat_qydd(*(pose_quat+3),*(pose_quat+4)+dd,*(pose_quat+5),*(pose_quat+6));
  tf::Quaternion quat_qzdd(*(pose_quat+3),*(pose_quat+4),*(pose_quat+5)+dd,*(pose_quat+6));
  tf::Quaternion quat_qwdd(*(pose_quat+3),*(pose_quat+4),*(pose_quat+5),*(pose_quat+6)+dd);

  double rpy_origin[3],rpy_qxdd[3],rpy_qydd[3],rpy_qzdd[3],rpy_qwdd[3];
  tf::Matrix3x3(quat_origin).getRPY(rpy_origin[0], rpy_origin[1], rpy_origin[2]);//quaternion -> roll,pitch,yaw
  tf::Matrix3x3(quat_qxdd).getRPY(rpy_qxdd[0], rpy_qxdd[1], rpy_qxdd[2]);//quaternion -> roll,pitch,yaw
  tf::Matrix3x3(quat_qydd).getRPY(rpy_qydd[0], rpy_qydd[1], rpy_qydd[2]);//quaternion -> roll,pitch,yaw
  tf::Matrix3x3(quat_qzdd).getRPY(rpy_qzdd[0], rpy_qzdd[1], rpy_qzdd[2]);//quaternion -> roll,pitch,yaw
  tf::Matrix3x3(quat_qwdd).getRPY(rpy_qwdd[0], rpy_qwdd[1], rpy_qwdd[2]);//quaternion -> roll,pitch,yaw

  Eigen::MatrixXd jacobian(6,7);
  jacobian << 1,0,0,0,0,0,0,
              0,1,0,0,0,0,0,
              0,0,1,0,0,0,0,
              0,0,0,(rpy_qxdd[0]-rpy_origin[0])/dd,(rpy_qydd[0]-rpy_origin[0])/dd,(rpy_qzdd[0]-rpy_origin[0])/dd,(rpy_qwdd[0]-rpy_origin[0])/dd,
              0,0,0,(rpy_qxdd[1]-rpy_origin[1])/dd,(rpy_qydd[1]-rpy_origin[1])/dd,(rpy_qzdd[1]-rpy_origin[1])/dd,(rpy_qwdd[1]-rpy_origin[1])/dd,
              0,0,0,(rpy_qxdd[2]-rpy_origin[2])/dd,(rpy_qydd[2]-rpy_origin[2])/dd,(rpy_qzdd[2]-rpy_origin[2])/dd,(rpy_qwdd[2]-rpy_origin[2])/dd;


  Eigen::MatrixXd eigen_quat_cov(7,7);
  for(int i=0;i<7;i++){
    for(int j=0;j<7;j++){
      eigen_quat_cov(j,i)=*(quat_cov+j*7+i);
    }
  }


  Eigen::MatrixXd eigen_rpy_cov(6,6);
  eigen_rpy_cov=jacobian*eigen_quat_cov*jacobian.transpose();

  for(int i=0;i<6;i++){
    for(int j=0;j<6;j++){
      *(rpy_cov+j*6+i)=eigen_rpy_cov(j,i);
    }
  }

}


/**
 * @brief publishPath, publish
 * two nav_msgs::path that can be
 * shown in rviz, one for frames that are optimized at the moemnt and one for all frames;frame id is
 * same as tf_parent, timestamp of msg is timestamp of the most
 * current keyframe;
 * @param path_publisher
 * @param active_path_publisher
 * @param bundle_adjuster
 */
void publishPathWithCovariance(ros::Publisher& path_with_cov_publisher,
                  ros::Publisher& active_path_with_cov_publisher,
                  keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                  std::string tf_parent_frame_id) {

   keyframe_bundle_adjustment_ros_tool::PathWithCovariance path_with_cov_msg;

   // timestamp of msg is same as last keyframe
   ros::Time cur_ts;
   cur_ts.fromNSec(bundle_adjuster->getKeyframe().timestamp_);
   path_with_cov_msg.header.stamp = cur_ts;
   // frame id of msg is same as tf_parent without tf2 convention
   path_with_cov_msg.header.frame_id = "/" + tf_parent_frame_id;

   keyframe_bundle_adjustment_ros_tool::PathWithCovariance active_path_with_cov_msg = path_with_cov_msg;

   for (const auto& kf : bundle_adjuster->keyframes_) {
       geometry_msgs::PoseWithCovarianceStamped cur_pose_with_cov;
       ros::Time p_ts;
       p_ts.fromNSec(kf.second->timestamp_);
       cur_pose_with_cov.header.stamp = p_ts;
       cur_pose_with_cov.header.frame_id = path_with_cov_msg.header.frame_id;
       //            ROS_DEBUG_STREAM("origin in cur_pose=\n"
       //                             << kf.second.getEigenPose().translation().transpose());

       toGeometryMsg(cur_pose_with_cov.pose.pose, kf.second->getEigenPose().inverse()); // second：std::pairの2番目

       // covariance
       double covariance[36];
       quatCov2rpyCov(kf.second->pose_.data(),kf.second->pose_covariance_.data(),covariance);
       for(int i=0;i<36;i++){
         cur_pose_with_cov.pose.covariance[i]=*(covariance+i);
       }

       path_with_cov_msg.poses_with_covariance.push_back(cur_pose_with_cov);

       if (kf.second->is_active_) {
           active_path_with_cov_msg.poses_with_covariance.push_back(cur_pose_with_cov);
       }
   }

   path_with_cov_publisher.publish(path_with_cov_msg);
   active_path_with_cov_publisher.publish(active_path_with_cov_msg);

}


}
}
