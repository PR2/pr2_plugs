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


#include <string.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

namespace pr2_plugs_common{

static bool gotoJointPosition(double q1, double q2,double q3,double q4,double q5,double q6,double q7, bool right_arm)
{
  ROS_INFO("Waiting for joint trajectory action");
  std::string arm = "r";
  if (!right_arm) arm = "l";

  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> trajectory_action(arm+"_arm_plugs_controller/joint_trajectory_action", true);
  trajectory_action.waitForServer();
  ROS_INFO("Moving %s arm to detection pose", arm.c_str());

  pr2_controllers_msgs::JointTrajectoryGoal trajectory_goal ;
  trajectory_goal.trajectory.joint_names.resize(7);
  trajectory_goal.trajectory.joint_names[0] = arm+"_upper_arm_roll_joint";
  trajectory_goal.trajectory.joint_names[1] = arm+"_shoulder_pan_joint";
  trajectory_goal.trajectory.joint_names[2] = arm+"_shoulder_lift_joint";
  trajectory_goal.trajectory.joint_names[3] = arm+"_forearm_roll_joint";
  trajectory_goal.trajectory.joint_names[4] = arm+"_elbow_flex_joint";
  trajectory_goal.trajectory.joint_names[5] = arm+"_wrist_flex_joint";
  trajectory_goal.trajectory.joint_names[6] = arm+"_wrist_roll_joint";

  
  // moves arm to given joint space position
  trajectory_goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2);
  trajectory_goal.trajectory.points.resize(1);
  trajectory_goal.trajectory.points[0].positions.resize(7);
  trajectory_goal.trajectory.points[0].positions[0] = q1;
  trajectory_goal.trajectory.points[0].positions[1] = q2;
  trajectory_goal.trajectory.points[0].positions[2] = q3;
  trajectory_goal.trajectory.points[0].positions[3] = q4;
  trajectory_goal.trajectory.points[0].positions[4] = q5;
  trajectory_goal.trajectory.points[0].positions[5] = q6;
  trajectory_goal.trajectory.points[0].positions[6] = q7;
  trajectory_goal.trajectory.points[0].time_from_start = ros::Duration(4.0);
  if (trajectory_action.sendGoalAndWait(trajectory_goal,ros::Duration(20.0), ros::Duration(5.0)) != actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_ERROR("Failed to reach joint space detection position");
    return false;
  }
  return true;
}

}
