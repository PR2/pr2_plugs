/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/

/* Author: Wim Meeussen */

#include "pr2_plugs_actions/move_base_omnidirectional.h"

using namespace ros;
using namespace std;

static const string fixed_frame = "odom_combined";


namespace pr2_plugs_actions{

MoveBaseOmnidirectionalAction::MoveBaseOmnidirectionalAction() :
  costmap_ros_("costmap_move_base", tf_),
  costmap_model_(costmap_),
  action_server_(ros::NodeHandle(), 
		 "move_base_omnidirectional", 
		 boost::bind(&MoveBaseOmnidirectionalAction::execute, this, _1))
{
  costmap_ros_.stop();

  ros::NodeHandle node;
  base_pub_ = node.advertise<geometry_msgs::Twist>("base_controller/command", 10);

  footprint_ = costmap_ros_.getRobotFootprint();
};



MoveBaseOmnidirectionalAction::~MoveBaseOmnidirectionalAction()
{};



void MoveBaseOmnidirectionalAction::execute(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{ 
  ROS_INFO("MoveBaseOmnidirectionalAction: execute");
  costmap_ros_.start();

  // get current robot pose
  tf::Stamped<tf::Pose> robot_pose;
  costmap_ros_.getRobotPose(robot_pose);
  ROS_INFO("MoveBaseOmnidirectionalAction: current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));

  // get desired robot pose
  tf::Stamped<tf::Pose> desired_pose;
  tf::poseStampedMsgToTF(goal->target_pose, desired_pose);
  if (!tf_.waitForTransform(fixed_frame, desired_pose.frame_id_, desired_pose.stamp_, ros::Duration(2.0))){
    ROS_ERROR("MoveBaseOmnidirectionalAction: could not transform from %s to %s", fixed_frame.c_str(), desired_pose.frame_id_.c_str());
    action_server_.setAborted();
    return;
  }
  tf_.transformPose(fixed_frame, desired_pose, desired_pose);
  ROS_INFO("MoveBaseOmnidirectionalAction: desired robot pose %f %f ==> %f", desired_pose.getOrigin().x(), desired_pose.getOrigin().y(), tf::getYaw(desired_pose.getRotation()));

  // command base to desired pose
  geometry_msgs::Twist diff = diff2D(desired_pose, robot_pose);
  ROS_INFO("MoveBaseOmnidirectionalAction: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);
  diff = limitTwist(diff);
  ROS_INFO("MoveBaseOmnidirectionalAction: diff limit %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);
  while (fabs(diff.linear.x) > 0.02 || abs(diff.linear.y) > 0.02 || abs(diff.angular.z) > 0.02){
    if (action_server_.isPreemptRequested()){
      geometry_msgs::Twist zero;
      base_pub_.publish(zero);
      action_server_.setPreempted();
      return;
    }
    base_pub_.publish(diff);
    costmap_ros_.getRobotPose(robot_pose);
    diff = limitTwist(diff2D(desired_pose, robot_pose));
    ros::Duration(0.01).sleep();
  }

  costmap_ros_.stop();
  action_server_.setSucceeded();
}



geometry_msgs::Twist MoveBaseOmnidirectionalAction::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
{
  geometry_msgs::Twist res;
  tf::Pose diff = pose2.inverse() * pose1;
  res.linear.x = diff.getOrigin().x();
  res.linear.y = diff.getOrigin().y();
  res.angular.z = tf::getYaw(diff.getRotation());
  return res;
}


geometry_msgs::Twist MoveBaseOmnidirectionalAction::limitTwist(const geometry_msgs::Twist& twist)
{
  geometry_msgs::Twist res;
  if (fabs(twist.linear.x) > 0.01) res.linear.x = 0.1 * twist.linear.x / fabs(twist.linear.x);
  if (fabs(twist.linear.y) > 0.01) res.linear.y = 0.1 * twist.linear.y / fabs(twist.linear.y);
  if (fabs(twist.angular.z) > 0.01) res.angular.z = 0.2 * twist.angular.z / fabs(twist.angular.z);
  return res;
}



std::vector<geometry_msgs::Point> MoveBaseOmnidirectionalAction::getOrientedFootprint(const tf::Vector3 pos, double theta_cost)
{
  double cos_th = cos(theta_cost);
  double sin_th = sin(theta_cost);
  std::vector<geometry_msgs::Point> oriented_footprint;
  for(unsigned int i = 0; i < footprint_.size(); ++i){
    geometry_msgs::Point new_pt;
    new_pt.x = pos.x() + (footprint_[i].x * cos_th - footprint_[i].y * sin_th);
    new_pt.y = pos.y() + (footprint_[i].x * sin_th + footprint_[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
  return oriented_footprint;
}


}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_omnidirectional");

  pr2_plugs_actions::MoveBaseOmnidirectionalAction action_server;

  ros::spin();
  return 0;
}
