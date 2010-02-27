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
#ifndef DETECTOR_FILTER_H_
#define DETECTOR_FILTER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/thread/mutex.hpp>
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include <std_srvs/Empty.h>


namespace detector
{

class DetectorFilter
{
public:
  DetectorFilter();
  ~DetectorFilter();

  bool getPose(geometry_msgs::PoseWithCovarianceStamped& pose);

private:
  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
  void initialize(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void decomposeTransform(const geometry_msgs::PoseWithCovarianceStamped& pose,
                          MatrixWrapper::ColumnVector& vector);
  void composeTransform(const MatrixWrapper::ColumnVector& vector,
                        geometry_msgs::PoseWithCovarianceStamped& pose);

  bool resetState(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res );

  ros::Subscriber pose_sub_;
  ros::ServiceServer reset_srv_;
  std::string fixed_frame_;
  bool initialized_;
  tf::TransformListener tf_;

  // filter stuff
  BFL::LinearAnalyticConditionalGaussian*                 sys_pdf_;
  BFL::LinearAnalyticSystemModelGaussianUncertainty*      sys_model_;
  BFL::LinearAnalyticConditionalGaussian*                 meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model_;
  BFL::Gaussian*                                          prior_;
  BFL::ExtendedKalmanFilter*                              filter_;
  ros::Time                                               filter_time_;


};// class
}// namespace
#endif
