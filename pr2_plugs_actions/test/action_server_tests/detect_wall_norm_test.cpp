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
#include <pr2_plugs_msgs/DetectWallNormAction.h>
#include <actionlib/server/simple_action_server.h>
#include <stereo_wall_detection/DetectWall.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <pr2_controllers_msgs/PointHeadAction.h>


void spinThread()
{
  ros::spin();
}

bool set_param (dynamic_reconfigure::Reconfigure::Request &req, dynamic_reconfigure::Reconfigure::Response &resp)
{
  return true;
}

void head_execute(const pr2_controllers_msgs::PointHeadGoalConstPtr& goal, actionlib::SimpleActionServer<pr2_controllers_msgs::PointHeadAction>* as)
{
  as->setSucceeded();
}

bool detect_wall (stereo_wall_detection::DetectWall::Request &req, stereo_wall_detection::DetectWall::Response &resp)
{
  return true;
}

TEST(ActionServerTest, detect_wall_norm)
{
  ros::NodeHandle n;
  boost::thread spin_thread(&spinThread);
  
  ros::ServiceServer wall_serv = n.advertiseService ("stereo_wall_detection/detect_wall", detect_wall);
  ros::ServiceServer param_serv = n.advertiseService ("camera_synchronizer_node/set_parameters", set_param);
  actionlib::SimpleActionServer<pr2_controllers_msgs::PointHeadAction> head_server(n, "head_traj_controller/point_head_action", boost::bind(&head_execute, _1, &head_server), false);
  head_server.start();

  actionlib::SimpleActionClient<pr2_plugs_msgs::DetectWallNormAction> ac("detect_wall_norm"); 


  EXPECT_TRUE(ac.waitForServer(ros::Duration(15.0)));
  ros::shutdown();
  spin_thread.join();
  
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "detect_wall_norm_tests");
  return RUN_ALL_TESTS();

}
