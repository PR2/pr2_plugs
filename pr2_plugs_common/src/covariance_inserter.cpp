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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


static std::string fixed_frame_;
static std::vector<double> cov_fixed_;



void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  geometry_msgs::PoseWithCovarianceStamped pose_fixed;
  ros::NodeHandle node_top;
  static tf::TransformListener tf;
  static  ros::Publisher  pub = node_top.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_out", 10);

  // transfrom pose to fixed frame
  ROS_DEBUG("Measurement in frame %s: %f, %f, %f,     %f, %f, %f, %f",
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
  if (!tf.waitForTransform(fixed_frame_, pose->header.frame_id, pose->header.stamp, ros::Duration(0.5))){
    ROS_ERROR("Could not transform from %s to %s at time %f", fixed_frame_.c_str(), pose->header.frame_id.c_str(), pose->header.stamp.toSec());
    return;
  }
  tf.transformPose(fixed_frame_, tf_stamped_pose, tf_stamped_pose);

  // convert posestamped back to posewithcovariancestamped
  pose_fixed = *pose;
  tf::poseTFToMsg(tf_stamped_pose, pose_fixed.pose.pose);
  pose_fixed.header.frame_id = fixed_frame_;

  ROS_DEBUG("Measurement in frame %s: %f, %f, %f,     %f, %f, %f, %f",
            fixed_frame_.c_str(), 
            pose_fixed.pose.pose.position.x, 
            pose_fixed.pose.pose.position.y,
            pose_fixed.pose.pose.position.z,
            pose_fixed.pose.pose.orientation.x,
            pose_fixed.pose.pose.orientation.y,
            pose_fixed.pose.pose.orientation.z,
            pose_fixed.pose.pose.orientation.w);

  // set fixed frame covariance
  for (unsigned int i=0; i<6; i++)
    pose_fixed.pose.covariance[6*i+i] = cov_fixed_[i];
  pub.publish(pose_fixed);
}



int main(int argc, char** argv){
  ros::init(argc, argv, "covariance_inserter");
  ros::NodeHandle node("~");
  cov_fixed_.resize(6);

  node.param("fixed_frame", fixed_frame_, std::string("fixed_frame_not_specified"));
  node.param("cov_fixed1", cov_fixed_[0], 0.0);
  node.param("cov_fixed2", cov_fixed_[1], 0.0);
  node.param("cov_fixed3", cov_fixed_[2], 0.0);
  node.param("cov_fixed4", cov_fixed_[3], 0.0);
  node.param("cov_fixed5", cov_fixed_[4], 0.0);
  node.param("cov_fixed6", cov_fixed_[5], 0.0);

  ros::NodeHandle node_top;
  ros::Subscriber sub = node_top.subscribe<geometry_msgs::PoseWithCovarianceStamped>("pose_in", 10, &poseCallback);

  ros::spin();
  return 0;
}
