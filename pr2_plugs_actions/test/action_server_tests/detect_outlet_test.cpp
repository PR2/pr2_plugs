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
#include <pr2_plugs_msgs/DetectOutletAction.h>
#include <pr2_plugs_msgs/VisionOutletDetectionAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_plugs_msgs/DetectWallNormAction.h>
#include <actionlib/server/simple_action_server.h>


void spinThread()
{
  ros::spin();
}

void arm_execute(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>* as)
{
  as->setSucceeded();
}

void norm_execute(const pr2_plugs_msgs::DetectWallNormGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_plugs_msgs::DetectWallNormAction>* as)
{
  as->setSucceeded();
}

void outlet_execute(const pr2_plugs_msgs::VisionOutletDetectionGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_plugs_msgs::VisionOutletDetectionAction>* as)
{
  as->setSucceeded();
}


TEST(ActionServerTest, detect_outlet)
{
  ros::NodeHandle n;
  boost::thread spin_thread(&spinThread);
  actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> arm_server(n, "r_arm_plugs_controller/joint_trajectory_action", boost::bind(&arm_execute, _1, &arm_server));
  actionlib::SimpleActionServer<pr2_plugs_msgs::DetectWallNormAction> norm_server(n, "detect_wall_norm", boost::bind(&norm_execute, _1, &norm_server));
  actionlib::SimpleActionServer<pr2_plugs_msgs::VisionOutletDetectionAction> outlet_server(n, "vision_outlet_detection", boost::bind(&outlet_execute, _1, &outlet_server));

  actionlib::SimpleActionClient<pr2_plugs_msgs::DetectOutletAction> ac("detect_outlet"); 


  EXPECT_TRUE(ac.waitForServer(ros::Duration(15.0)));
  ros::shutdown();
  spin_thread.join();
  
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "detect_outlet_tests");
  return RUN_ALL_TESTS();

}
