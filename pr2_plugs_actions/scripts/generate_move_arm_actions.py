#!/usr/bin/env python                                                                        
# Software License Agreement (BSD License)                                                   
#                                                                                            
# Copyright (c) 2009, Willow Garage, Inc.                                                    
# All rights reserved.                                                                       
#                                                                                            
# Redistribution and use in source and binary forms, with or without                         
# modification, are permitted provided that the following conditions                         
# are met:                                                                                   
#                                                                                            
#  * Redistributions of source code must retain the above copyright                          
#    notice, this list of conditions and the following disclaimer.                           
#  * Redistributions in binary form must reproduce the above                                 
#    copyright notice, this list of conditions and the following                             
#    disclaimer in the documentation and/or other materials provided                         
#    with the distribution.                                                                  
#  * Neither the name of the Willow Garage nor the names of its                              
#    contributors may be used to endorse or promote products derived                         
#    from this software without specific prior written permission.                           
#                                                                                            
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS                        
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT                          
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS                          
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE                             
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,                        
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,                       
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                           
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER                           
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT                         
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN                          
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE                            
# POSSIBILITY OF SUCH DAMAGE.                                                                
#                                                                                            
# Author Melonee Wise                                                    


import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib
import sys
from pr2_controllers_msgs.msg import *
from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint



class MoveArmServer:
  def __init__(self, node_name, action_name):
    self.name = action_name
    self.ns = node_name + "/" + action_name
    self.arm = rospy.get_param(self.ns + "/arm")
    self.arm_controller = rospy.get_param(self.ns + "/arm_controller")

    self.joint_space_client = actionlib.SimpleActionClient(self.arm_controller + '/joint_trajectory_action', JointTrajectoryAction)
    
    if(not self.joint_space_client.wait_for_server(rospy.Duration(20.0))):
      rospy.logerr("%s: Joint Trajectory Action server is not up", self.name)
      sys.exit(1)  
      
    self.joint_space_goal = JointTrajectoryGoal()
    self.joint_space_goal.trajectory.joint_names = rospy.get_param(node_name + "/" + self.arm + "_arm_joints")
    self.waypoints = rospy.get_param(self.ns +"/waypoints")

    self.server = actionlib.simple_action_server.SimpleActionServer(self.name, EmptyAction, self.execute_cb)


  def execute_cb(self, goal):
    rospy.loginfo("%s: Received goal", self.name)
    for waypoint in self.waypoints:
      self.joint_space_goal.trajectory.header.stamp = rospy.Time.now()
      self.joint_space_goal.trajectory.points = [JointTrajectoryPoint(waypoint, [], [], rospy.Duration(4.0))]
      joint_space_client.send_goal(joint_space_goal)
      joint_space_client.wait_for_result(rospy.Duration(20.0))
      if joint_space_client.get_state() != GoalStatus.SUCCEEDED:
        rospy.logerr('%s: Failed to move arm to joint configuration', self.name)
        server.set_aborted()
        return

    result = EmptyResult() 
    self.server.set_succeeded(result)


if __name__ == '__main__':
    
  name ='generate_move_arm_actions'
  rospy.init_node(name)
  actions = rospy.get_param(name +"/actions")
  for action in actions:
    server = MoveArmServer(name, action)
    rospy.loginfo('%s: Action server running', name)
  rospy.spin()
