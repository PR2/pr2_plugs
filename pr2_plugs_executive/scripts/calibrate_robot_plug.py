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


import roslib
roslib.load_manifest('pr2_plugs_executive')

import rospy

import os
import sys
import time
import tf

from pr2_plugs_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from geometry_msgs.msg import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from std_srvs.srv import *

import actionlib



def main():
  rospy.init_node("plugs_executive")

  executive = Executive()
  rospy.spin()


class Executive:
  def __init__(self):

    # Declare list of actions for easy construction                                                 
    actions = [
      ('move_base',MoveBaseAction),
      ('tuck_arms',TuckArmsAction),
      ('detect_outlet',DetectOutletAction),
      ('fetch_plug',FetchPlugAction),
      ('detect_plug',DetectPlugInGripperAction)]

   # publisher for state                                                                           
    self.recharge_state = RechargeState()
    self.recharge_state_pub = rospy.Publisher('recharge_state', RechargeState, None, False, True)
    self.recharge_state.state = RechargeState.WAITING_FOR_STATE
    self.recharge_state_pub.publish(self.recharge_state)

    # subscriber for commands                                                                       
    rospy.Subscriber("recharge_command", RechargeCommand, self.recharge_cb)

    # construct tf listener                                                                         
    self.transformer = tf.TransformListener(True, rospy.Duration(60.0))
    self.preempt_timeout = rospy.Duration(5.0)

    # Construct action ac                                                                           
    rospy.loginfo("Starting actions.")
    self.ac = dict()
    for (name,action) in actions:
      self.ac[name] = actionlib.SimpleActionClient(name,action)

    # Wait for all the ac to start                                                                  
    for (name,action) in actions:
      print "Wait for server "+name
      self.ac[name].wait_for_server()
    rospy.loginfo("All actions started.")

    # ready to receive commands                                                                     
    self.recharge_state.state = RechargeState.UNPLUGGED
    self.recharge_state_pub.publish(self.recharge_state)

    # vars we need between plug and unplug                                                          
    self.gripper_to_plug = PoseStamped()
    self.base_to_plug = PoseStamped()

  def recharge_cb(self, msg):
    if self.recharge_state.state == RechargeState.WAITING_FOR_STATE:
      rospy.logerr("Plugs Executive: can't execute commands while still waiting for state")
      return

    elif self.recharge_state.state == RechargeState.UNPLUGGED and msg.command == RechargeCommand.CALIBRATE:
      rospy.logerr("Plugs Executive: Starting to plug in")
      self.recharge_state.state = RechargeState.WAITING_FOR_STATE
      self.recharge_state_pub.publish(self.recharge_state)

      # Untuck the arms                                                                             
      untuck_goal = TuckArmsGoal()
      untuck_goal.left=True
      untuck_goal.right=True
      untuck_goal.untuck=True
      untucked = False
      while not untucked:
        rospy.loginfo("Untucking right arm...")
        untucked = (self.ac['tuck_arms'].send_goal_and_wait(untuck_goal, rospy.Duration(30.0), self.preempt_timeout) == GoalStatus.SUCCEEDED)

      # Detect the outlet                                                                           
      detect_outlet_goal = DetectOutletGoal()
      rospy.loginfo("Detecting the outlet ...")
      if self.ac['detect_outlet'].send_goal_and_wait(detect_outlet_goal, rospy.Duration(90.0), self.preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr("Failed to detect the outlet!")
        return
      outlet_pose = self.ac['detect_outlet'].get_result().outlet_pose
      try:
        self.transformer.waitForTransform("base_link", outlet_pose.header.frame_id, outlet_pose.header.stamp, rospy.Duration(2.0))
      except rospy.ServiceException, e:
        rospy.logerr('Could not transform between base_link and %s' %outlet_pose.header.frame_id)
        return
      base_to_outlet = self.transformer.transformPose('base_link', outlet_pose)

      rospy.loginfo('Fetching plug...')
      if self.ac['fetch_plug'].send_goal_and_wait(FetchPlugGoal(), rospy.Duration(60.0), self.preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr("Failed to fetch plug!")
        return
      plug_pose = self.ac['fetch_plug'].get_result().plug_pose
      try:
        self.transformer.waitForTransform("base_link", plug_pose.header.frame_id, plug_pose.header.stamp, rospy.Duration(2.0))
      except rospy.ServiceException, e:
        rospy.logerr('Could not transform between base_link and %s' %plug_pose.header.frame_id)
        return
      self.base_to_plug = self.transformer.transformPose('base_link', plug_pose)

      # Detect the plug in gripper                                                                  
      detect_plug_goal = DetectPlugInGripperGoal()
      rospy.loginfo('Detecting plug in gripper...')
      if self.ac['detect_plug'].send_goal_and_wait(detect_plug_goal, rospy.Duration(30.0), self.preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr("Failed to detect plug in gripper!")
        return
      plug_pose = self.ac['detect_plug'].get_result().plug_pose
      try:
        self.transformer.waitForTransform("r_gripper_tool_frame", plug_pose.header.frame_id, plug_pose.header.stamp, rospy.Duration(2.0))
      except rospy.ServiceException, e:
        rospy.logerr('Could not transform between r_gripper_tool_frame and %s' %plug_pose.header.frame_id)
        return
      self.gripper_to_plug = self.transformer.transformPose('r_gripper_tool_frame', plug_pose)

      print "rosrun pr2_controller_manager pr2_controller_manager stop r_arm_plugs_controller"
      print "insert plug into socket within 15 seconds"
      rospy.sleep(15.0)
     
      time = rospy.Time.now()
      try:
        self.transformer.waitForTransform("base_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
      except rospy.ServiceException, e:
        rospy.logerr('Could not transform between gripper and base at time %f' %time.to_sec())
        return
      pose_base_gripper= PoseStampedMath().fromTf(self.transformer.lookupTransform("base_link", "r_gripper_tool_frame", time))
      pose_outlet_base = PoseStampedMath(base_to_outlet).inverse()
      pose_gripper_plug = PoseStampedMath(self.gripper_to_plug)

         
      outlet_plug_pose = pose_outlet_base*pose_base_gripper*pose_gripper_plug
      plug_outlet_pose = (outlet_plug_pose.inverse()).msg
      outlet_plug_pose = (outlet_plug_pose).msg

      plug_position_x = rospy.get_param("vision_plug_detection/plug_position_x")
      plug_position_z = rospy.get_param("vision_plug_detection/plug_position_z")
   
      plug_position_x = plug_position_x + outlet_plug_pose.pose.position.z
      plug_position_z = plug_position_z + outlet_plug_pose.pose.position.y


      rospy.loginfo("The new calibration values for pr2_plug_description.xml are")
      rospy.loginfo("plug_position_x: %f plug_position_z: %f", plug_position_x, plug_position_z)  


if __name__ == '__main__':
  main()
