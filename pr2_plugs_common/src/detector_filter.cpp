/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Wim Meeussen
*********************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdl/frames.hpp>
#include "pr2_plugs_common/detector_filter.h"

namespace detector{

using namespace MatrixWrapper;
using namespace BFL;


DetectorFilter::DetectorFilter()
  : initialized_(false),
    prior_(NULL),
    filter_(NULL)
{
  // create system model
  ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
  SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
  Matrix A(6,6);  A = 0;
  for (unsigned int i=1; i<=6; i++){
    sysNoise_Cov(i,i) = pow(0.000000001,2);
    A(i,i) = 1.0;
  }
  Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
  sys_pdf_   = new LinearAnalyticConditionalGaussian(A, system_Uncertainty);
  sys_model_ = new LinearAnalyticSystemModelGaussianUncertainty(sys_pdf_);

  // create measurement model
  ColumnVector measNoise_Mu(6);  measNoise_Mu = 0;
  SymmetricMatrix measNoise_Cov(6);  measNoise_Cov = 0;
  Matrix H(6,6);  H = 0;
  for (unsigned int i=1; i<=6; i++){
    measNoise_Cov(i,i) = 1;
    H(i,i) = 1.0;
  }
  Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);
  meas_pdf_   = new LinearAnalyticConditionalGaussian(H, measurement_Uncertainty);
  meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);

  ros::NodeHandle node_private("~");
  node_private.param("fixed_frame", fixed_frame_, std::string("fixed_frame_not_specified"));
  reset_srv_ = node_private.advertiseService("reset_state", &DetectorFilter::resetState, this);

  ros::NodeHandle node;
  pose_sub_ = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_in", 10, &DetectorFilter::poseCallback, this);

}

DetectorFilter::~DetectorFilter()
{
  if (filter_) delete filter_;
  if (prior_) delete prior_;
  if (meas_model_) delete meas_model_;
  if (meas_pdf_) delete meas_pdf_;
  if (sys_model_) delete sys_model_;
  if (sys_pdf_) delete sys_pdf_;
}



void DetectorFilter::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  ROS_INFO("Measurement in frame %s: %f, %f, %f,     %f, %f, %f, %f",
            pose->header.frame_id.c_str(), 
            pose->pose.pose.position.x, 
            pose->pose.pose.position.y,
            pose->pose.pose.position.z,
            pose->pose.pose.orientation.x,
            pose->pose.pose.orientation.y,
            pose->pose.pose.orientation.z,
            pose->pose.pose.orientation.w);

  // convert posewithcovariancestamped to posestamped
  tf::Stamped<tf::Pose> tf_stamped_pose;
  tf::poseMsgToTF(pose->pose.pose, tf_stamped_pose);
  tf_stamped_pose.stamp_ = pose->header.stamp;
  tf_stamped_pose.frame_id_ = pose->header.frame_id;

  // transform posestamped to fixed frame
  if (!tf_.waitForTransform(fixed_frame_, pose->header.frame_id, pose->header.stamp, ros::Duration(0.5))){
    ROS_ERROR("Could not transform from %s to %s at time %f", fixed_frame_.c_str(), pose->header.frame_id.c_str(), pose->header.stamp.toSec());
    return;
  }
  tf_.transformPose(fixed_frame_, tf_stamped_pose, tf_stamped_pose);

  // convert posestamped back to posewithcovariancestamped
  geometry_msgs::PoseWithCovarianceStamped tf_covariance_pose;
  tf_covariance_pose = *pose;
  tf::poseTFToMsg(tf_stamped_pose, tf_covariance_pose.pose.pose);
  tf_covariance_pose.header.frame_id = fixed_frame_;

  ROS_INFO("Measurement in frame %s: %f, %f, %f,     %f, %f, %f, %f",
            fixed_frame_.c_str(), 
            tf_covariance_pose.pose.pose.position.x, 
            tf_covariance_pose.pose.pose.position.y,
            tf_covariance_pose.pose.pose.position.z,
            tf_covariance_pose.pose.pose.orientation.x,
            tf_covariance_pose.pose.pose.orientation.y,
            tf_covariance_pose.pose.pose.orientation.z,
            tf_covariance_pose.pose.pose.orientation.w);

  // initialize filter first time
  if (!initialized_){
    ROS_INFO("Initializing detector filter");
    initialize(tf_covariance_pose);
  }
  // update filter
  else{
    ROS_DEBUG("update filter");
    ColumnVector measurement(6);
    decomposeTransform(tf_covariance_pose, measurement);
    filter_->Update(sys_model_);
    filter_->Update(meas_model_, measurement);
  }
}

void DetectorFilter::initialize(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  // set prior of filter
  ColumnVector prior_Mu(6);
  decomposeTransform(pose, prior_Mu);
  SymmetricMatrix prior_Cov(6);
  for (unsigned int i=0; i<6; i++) 
    for (unsigned int j=0; j<6; j++)
      prior_Cov(i+1,j+1) = pose.pose.covariance[6*i+j];

  // make sure we don't leak
  if (filter_) delete filter_;
  if (prior_) delete prior_;
  prior_  = new Gaussian(prior_Mu,prior_Cov);
  filter_ = new ExtendedKalmanFilter(prior_);
  filter_time_ = pose.header.stamp;
  initialized_ = true;
}



void DetectorFilter::decomposeTransform(const geometry_msgs::PoseWithCovarianceStamped& pose,
                                        MatrixWrapper::ColumnVector& vector)
{
  assert(vector.rows() == 6);

  // construct kdl rotation from quaternion, and extract RPY
  KDL::Rotation rot = KDL::Rotation::Quaternion(pose.pose.pose.orientation.x,
                                                pose.pose.pose.orientation.y,
                                                pose.pose.pose.orientation.z,
                                                pose.pose.pose.orientation.w);
  rot.GetRPY(vector(4), vector(5), vector(6));

  vector(1) = pose.pose.pose.position.x;
  vector(2) = pose.pose.pose.position.y;
  vector(3) = pose.pose.pose.position.z;
};

void DetectorFilter::composeTransform(const MatrixWrapper::ColumnVector& vector,
                                      geometry_msgs::PoseWithCovarianceStamped& pose)
{
  assert(vector.rows() == 6);

  // construct kdl rotation from vector (x, y, z, Rx, Ry, Rz), and extract quaternion
  KDL::Rotation rot = KDL::Rotation::RPY(vector(4), vector(5), vector(6));
  rot.GetQuaternion(pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
                    pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);

  pose.pose.pose.position.x = vector(1);
  pose.pose.pose.position.y = vector(2);
  pose.pose.pose.position.z = vector(3);
};


bool DetectorFilter::getPose(geometry_msgs::PoseWithCovarianceStamped& pose)
{
  // only get pose when we received a pose at the input side
  if (!initialized_) return false;
  
  // mean
  ColumnVector mean = filter_->PostGet()->ExpectedValueGet();
  composeTransform(mean, pose);

  // header
  pose.header.stamp = filter_time_;
  pose.header.frame_id = fixed_frame_;

  // covariance
  SymmetricMatrix covar =  filter_->PostGet()->CovarianceGet();
  for (unsigned int i=0; i<6; i++)
    for (unsigned int j=0; j<6; j++)
      pose.pose.covariance[6*i+j] = covar(i+1,j+1);

  return true;
}

bool DetectorFilter::resetState(std_srvs::Empty::Request  &req,
				std_srvs::Empty::Response &res )
{
  initialized_ = false;
  return true;
}



}// namespace


int main(int argc, char** argv)
{
  ros::init(argc, argv, "detector_filter");

  ros::NodeHandle node;
  ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 10);
  geometry_msgs::PoseWithCovarianceStamped pose;
  detector::DetectorFilter detector_filter;

  ros::Rate rate(100.0);
  while (ros::ok()){
    if (detector_filter.getPose(pose)){
      pose.header.stamp = ros::Time::now();
      pose_pub.publish(pose);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}



