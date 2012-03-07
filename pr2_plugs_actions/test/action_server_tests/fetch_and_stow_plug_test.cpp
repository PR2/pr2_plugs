/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Melonee Wise
 */


#include <ros/ros.h>
#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <actionlib/client/simple_action_client.h>
#include <pr2_plugs_msgs/FetchPlugAction.h>
#include <pr2_plugs_msgs/StowPlugAction.h>
#include <pr2_plugs_msgs/DetectPlugOnBaseAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_common_action_msgs/ArmMoveIKAction.h>
#include <actionlib/server/simple_action_server.h>


void spinThread()
{
  ros::spin();
}

void arm_execute(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>* as)
{
  as->setSucceeded();
}

void gripper_execute(const pr2_controllers_msgs::Pr2GripperCommandGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_controllers_msgs::Pr2GripperCommandAction>* as)
{
  as->setSucceeded();
}

void ik_execute(const pr2_common_action_msgs::ArmMoveIKGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_common_action_msgs::ArmMoveIKAction>* as)
{
  as->setSucceeded();
}

void plug_execute(const pr2_plugs_msgs::DetectPlugOnBaseGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_plugs_msgs::DetectPlugOnBaseAction>* as)
{
  as->setSucceeded();
}

void spine_execute(const pr2_controllers_msgs::SingleJointPositionGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_controllers_msgs::SingleJointPositionAction>* as)
{
  as->setSucceeded();
}


TEST(ActionServerTest, fetch_plug)
{
  ros::NodeHandle n;
  boost::thread spin_thread(&spinThread);
  actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> arm_server(n, "r_arm_controller/joint_trajectory_generator", boost::bind(&arm_execute, _1, &arm_server), false);
  actionlib::SimpleActionServer<pr2_common_action_msgs::ArmMoveIKAction> ik_server(n, "r_arm_ik", boost::bind(&ik_execute, _1, &ik_server), false);
  actionlib::SimpleActionServer<pr2_plugs_msgs::DetectPlugOnBaseAction> plug_server(n, "detect_plug_on_base", boost::bind(&plug_execute, _1, &plug_server), false);
  actionlib::SimpleActionServer<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_server(n, "r_gripper_controller/gripper_action", boost::bind(&gripper_execute, _1, &gripper_server), false);
  actionlib::SimpleActionServer<pr2_controllers_msgs::SingleJointPositionAction> spine_server(n, "torso_controller/position_joint_action", boost::bind(&spine_execute, _1, &spine_server), false);
  arm_server.start();
  ik_server.start();
  plug_server.start();
  gripper_server.start();
  spine_server.start();


  actionlib::SimpleActionClient<pr2_plugs_msgs::FetchPlugAction> ac("fetch_plug");


  EXPECT_TRUE(ac.waitForServer(ros::Duration(20.0)));
  ros::shutdown();
  spin_thread.join();

}

TEST(ActionServerTest, stow_plug)
{
  ros::NodeHandle n;
  boost::thread spin_thread(&spinThread);
  actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> arm_server(n, "r_arm_controller/joint_trajectory_generator", boost::bind(&arm_execute, _1, &arm_server), false);
  actionlib::SimpleActionServer<pr2_common_action_msgs::ArmMoveIKAction> ik_server(n, "r_arm_ik", boost::bind(&ik_execute, _1, &ik_server), false);
  actionlib::SimpleActionServer<pr2_plugs_msgs::DetectPlugOnBaseAction> plug_server(n, "detect_plug_on_base", boost::bind(&plug_execute, _1, &plug_server), false);
  actionlib::SimpleActionServer<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_server(n, "r_gripper_controller/gripper_action", boost::bind(&gripper_execute, _1, &gripper_server), false);
  actionlib::SimpleActionServer<pr2_controllers_msgs::SingleJointPositionAction> spine_server(n, "torso_controller/position_joint_action", boost::bind(&spine_execute, _1, &spine_server), false);
  arm_server.start();
  ik_server.start();
  plug_server.start();
  gripper_server.start();
  spine_server.start();

  actionlib::SimpleActionClient<pr2_plugs_msgs::StowPlugAction> ac("stow_plug");


  EXPECT_TRUE(ac.waitForServer(ros::Duration(20.0)));
  ros::shutdown();
  spin_thread.join();

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "fetch_and_stow_plug_tests");
  return RUN_ALL_TESTS();

}
