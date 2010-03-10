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

#include "pr2_plugs_actions/align_base.h"

using namespace ros;
using namespace std;

static const string fixed_frame = "odom_combined";
static const double motion_step = 0.01; // 1 cm steps
static const double update_rate = 10.0; // 10 hz
static const double desired_distance = 0.81;

namespace pr2_plugs_actions{

AlignBaseAction::AlignBaseAction() :
  wall_detector_("detect_wall_norm", true),
  costmap_ros_("costmap_align_base", tf_),
  costmap_model_(costmap_),
  action_server_(ros::NodeHandle(), 
		 "align_base", 
		 boost::bind(&AlignBaseAction::execute, this, _1))
{
  costmap_ros_.stop();

  ros::NodeHandle node;
  base_pub_ = node.advertise<geometry_msgs::Twist>("base_controller/command", 10);

  footprint_ = costmap_ros_.getRobotFootprint();
};



AlignBaseAction::~AlignBaseAction()
{};



void AlignBaseAction::execute(const pr2_plugs_msgs::AlignBaseGoalConstPtr& goal)
{ 
  ROS_INFO("AlignBaseAction: execute");

  costmap_ros_.start();

  // get wall normal
  pr2_plugs_msgs::DetectWallNormGoal wall_norm_goal;
  wall_norm_goal.look_point = goal->look_point;
  while (ros::ok() && wall_detector_.sendGoalAndWait(wall_norm_goal, ros::Duration(100.0), ros::Duration(5.0)) != actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("AlignBaseAction: try again to get wall norm");

  // convert wall norm to fixed frame
  geometry_msgs::PointStamped wall_point_msg = wall_detector_.getResult()->wall_point;
  geometry_msgs::Vector3Stamped wall_norm_msg = wall_detector_.getResult()->wall_norm;
  if (!tf_.waitForTransform(fixed_frame, wall_norm_msg.header.frame_id, wall_norm_msg.header.stamp, ros::Duration(2.0))){
    ROS_ERROR("AlignBaseAction: failed to transform from frame %s to %s", fixed_frame.c_str(), wall_norm_msg.header.frame_id.c_str());
    action_server_.setAborted();
    return;
  }
  tf_.transformPoint(fixed_frame, wall_point_msg, wall_point_msg);
  tf_.transformVector(fixed_frame, wall_norm_msg, wall_norm_msg);
  tf::Vector3 wall_norm = fromVector(wall_norm_msg.vector);
  tf::Vector3 wall_point = fromPoint(wall_point_msg.point);
  ROS_INFO("AlignBaseAction: wall norm  %f %f %f", wall_norm.x(), wall_norm.y(), wall_norm.z());
  ROS_INFO("AlignBaseAction: wall point %f %f %f", wall_point.x(), wall_point.y(), wall_point.z());

  // get current robot pose
  tf::Stamped<tf::Pose> robot_pose;
  costmap_ros_.getRobotPose(robot_pose);
  ROS_INFO("AlignBaseAction: current robot pose %f %f ==> %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), tf::getYaw(robot_pose.getRotation()));

  // get desired robot pose
  tf::Pose desired_pose;
  desired_pose.setOrigin(robot_pose.getOrigin() + (wall_norm * (wall_norm.dot(wall_point-robot_pose.getOrigin()) - desired_distance)));
  desired_pose.setRotation(tf::createQuaternionFromYaw(getVectorAngle(tf::Vector3(0,1,0), wall_norm*-1)));
  ROS_INFO("AlignBaseAction: desired robot pose %f %f ==> %f", desired_pose.getOrigin().x(), desired_pose.getOrigin().y(), tf::getYaw(desired_pose.getRotation()));

  // command base to desired pose
  geometry_msgs::Twist diff = diff2D(desired_pose, robot_pose);
  ROS_INFO("AlignBaseAction: diff %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);
  diff = limitTwist(diff);
  ROS_INFO("AlignBaseAction: diff limit %f %f ==> %f", diff.linear.x, diff.linear.y, diff.angular.z);
  while (fabs(diff.linear.x) > 0.02 || abs(diff.linear.y) > 0.02 || abs(diff.angular.z) > 0.02){
    base_pub_.publish(diff);
    costmap_ros_.getRobotPose(robot_pose);
    diff = limitTwist(diff2D(desired_pose, robot_pose));
    ros::Duration(0.01).sleep();
  }

  costmap_ros_.stop();
  action_server_.setSucceeded();
}



geometry_msgs::Twist AlignBaseAction::diff2D(const tf::Pose& pose1, const tf::Pose& pose2)
{
  geometry_msgs::Twist res;
  tf::Pose diff = pose2.inverse() * pose1;
  res.linear.x = diff.getOrigin().x();
  res.linear.y = diff.getOrigin().y();
  res.angular.z = tf::getYaw(diff.getRotation());
  return res;
}


geometry_msgs::Twist AlignBaseAction::limitTwist(const geometry_msgs::Twist& twist)
{
  geometry_msgs::Twist res;
  if (fabs(twist.linear.x) > 0.01) res.linear.x = 0.1 * twist.linear.x / fabs(twist.linear.x);
  if (fabs(twist.linear.y) > 0.01) res.linear.y = 0.1 * twist.linear.y / fabs(twist.linear.y);
  if (fabs(twist.angular.z) > 0.01) res.angular.z = 0.2 * twist.angular.z / fabs(twist.angular.z);
  return res;
}

geometry_msgs::Point AlignBaseAction::toPoint(const tf::Vector3& pnt)
{
  geometry_msgs::Point res;
  res.x = pnt.x();
  res.y = pnt.y();
  res.z = pnt.z();
  return res;
}

geometry_msgs::Vector3 AlignBaseAction::toVector(const tf::Vector3& pnt)
{
  geometry_msgs::Vector3 res;
  res.x = pnt.x();
  res.y = pnt.y();
  res.z = pnt.z();
  return res;
}

tf::Vector3 AlignBaseAction::fromVector(const geometry_msgs::Vector3& pnt)
{
  return tf::Vector3(pnt.x, pnt.y, pnt.z);
}

tf::Vector3 AlignBaseAction::fromPoint(const geometry_msgs::Point& pnt)
{
  return tf::Vector3(pnt.x, pnt.y, pnt.z);
}


std::vector<geometry_msgs::Point> AlignBaseAction::getOrientedFootprint(const tf::Vector3 pos, double theta_cost)
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

double AlignBaseAction::getVectorAngle(const tf::Vector3& v1, const tf::Vector3& v2)
{
  tf::Vector3 vec1 = v1; vec1 = vec1.normalize();
  tf::Vector3 vec2 = v2; vec2 = vec2.normalize();
  double dot      = vec2.x() * vec1.x() + vec2.y() * vec1.y();
  double perp_dot = vec2.y() * vec1.x() - vec2.x() * vec1.y();
  return atan2(perp_dot, dot);
}

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_base");

  pr2_plugs_actions::AlignBaseAction action_server;

  ros::spin();
  return 0;
}
