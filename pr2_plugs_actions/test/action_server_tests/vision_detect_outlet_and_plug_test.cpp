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
#include <pr2_plugs_msgs/VisionOutletDetectionAction.h>
#include <pr2_plugs_msgs/VisionPlugDetectionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

void spinThread()
{
  ros::spin();
}

TEST(ActionServerTest, vision_plug_detection)
{
  ros::NodeHandle n;
  boost::thread spin_thread(&spinThread);

  actionlib::SimpleActionClient<pr2_plugs_msgs::VisionPlugDetectionAction> ac("vision_plug_detection");
  pr2_plugs_msgs::VisionPlugDetectionGoal goal;
  goal.prior.header.stamp = ros::Time::now();
  goal.prior.header.frame_id = "base_link";
  goal.origin_on_right = false;
  goal.camera_name = "forearm_camera_l";

  ASSERT_TRUE(ac.waitForServer(ros::Duration(100.0)));

  ac.sendGoal(goal);
  ac.waitForResult();
  EXPECT_TRUE(ac.getState() == actionlib::SimpleClientGoalState::ABORTED);
  ros::shutdown();
  spin_thread.join();

}

TEST(ActionServerTest, vision_outlet_detection)
{
  ros::NodeHandle n;
  boost::thread spin_thread(&spinThread);

  actionlib::SimpleActionClient<pr2_plugs_msgs::VisionOutletDetectionAction> ac("vision_outlet_detection");
  pr2_plugs_msgs::VisionOutletDetectionGoal goal;
  goal.prior.header.stamp = ros::Time::now();
  goal.prior.header.frame_id = "base_link";
  goal.camera_name = "forearm_camera_l";

  ASSERT_TRUE(ac.waitForServer(ros::Duration(100.0)));

  ac.sendGoal(goal);
  ac.waitForResult();
  EXPECT_TRUE(ac.getState() == actionlib::SimpleClientGoalState::ABORTED);
  ros::shutdown();
  spin_thread.join();

}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "vision_detect_outlet_and_plug_test");
  return RUN_ALL_TESTS();

}

