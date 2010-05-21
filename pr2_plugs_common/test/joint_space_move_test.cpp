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
#include <pr2_plugs_common/joint_space_move.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>

typedef actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> Server;

void execute(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr& goal, Server* as)
{
  double joint_pos[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  std::vector<double> pos (joint_pos, joint_pos + sizeof(joint_pos) / sizeof(double) );
  if(goal->trajectory.points[0].positions == pos)
    as->setSucceeded();
  else
    as->setAborted();
}

void spinThread()
{
  ros::spin();
}


TEST(JointSpaceMove, test_failure)
{

  EXPECT_TRUE(pr2_plugs_common::gotoJointPosition(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, true));

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "trajectory_unwrap_test");
  ros::NodeHandle n;
  Server server(n, "r_arm_plugs_controller/joint_trajectory_action", boost::bind(&execute, _1, &server));
  boost::thread spin_thread(&spinThread);
  int log = RUN_ALL_TESTS();
  ros::shutdown();
  spin_thread.join();
  return log;
  
}
