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

import os,sys,time
import threading
import tf

from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from std_srvs.srv import *
from executive_python import *

import actionlib

def main():
  rospy.init_node("plugs_executive")

  # Construct list of actions
  actions = {
      'tuck_arms':(TuckArmsAction, 30.0),
      'detect_outlet':(DetectOutletAction, 90.0),
      'fetch_plug':(FetchPlugAction, 60.0),
      'detect_plug':(DetectPlugInGripperAction, 40.0),
      'wiggle_plug':(WigglePlugAction, 30.0),
      'stow_plug':(StowPlugAction, 45.0),
      'plugin':(PluginAction, 60.0)
      }

  # Construct executive
  exc = ActionExecutive(actions)

  # Construct tf listener
  transformer = tf.TransformListener(True, rospy.Duration(60.0))  

  def wait_and_transform(frame_id,pose):
    try:
      transformer.waitForTransform(frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return transformer.transformPose(frame_id, pose)

  untuck_goal = TuckArmsGoal(untuck=True,left=False,right=True)
  detect_outlet_goal = DetectOutletGoal()
  detect_plug_goal = DetectPlugInGripperGoal()
  plugin_goal = PluginGoal()
  stow_plug_goal = StowPlugGoal()
  tuck_goal = TuckArmsGoal(untuck=False,left=False,right=True)

  wiggle_in_goal = WigglePlugGoal()
  wiggle_in_goal.travel.x = 0.05
  wiggle_in_goal.offset.y = 0.01
  wiggle_in_goal.period = rospy.Duration(2.0)
  wiggle_in_goal.move_duration = rospy.Duration(10.0)
  wiggle_in_goal.abort_threshold = 0.02

  wiggle_out_goal = WigglePlugGoal()
  wiggle_out_goal.travel.x = -0.08
  wiggle_out_goal.offset.y = 0.01
  wiggle_out_goal.period = rospy.Duration(2.0)
  wiggle_out_goal.move_duration = rospy.Duration(10.0)
  wiggle_out_goal.abort_threshold = 0.02

  # Untuck the arms
  exc.tuck_arms_and_wait(untuck_goal)

  # Detect the outlet 
  outlet_pose = exc.detect_outlet_and_wait(detect_outlet_goal).outlet_pose

  # Plug in
  # Fetch alug
  plug_pose = exc.fetch_plug_and_wait(FetchPlugGoal()).plug_pose
  base_to_plug = wait_and_transform("base_link",plug_pose)

  # Detect the plug in gripper
  plug_pose = exc.detect_plug_and_wait(detect_plug_goal).plug_pose
  gripper_to_plug = wait_and_transform('r_gripper_tool_frame', plug_pose)

  # Plug in
  plugin_goal.gripper_to_plug = gripper_to_plug
  plugin_goal.base_to_outlet = base_to_outlet
  exc.plugin_and_wait(plugin_goal)

  # Stow plug
  stow_plug_goal.gripper_to_plug = gripper_to_plug
  stow_plug_goal.base_to_plug = base_to_plug
  exc.stow_plug_and_wait(stow_plug_goal)

  # Wiggle in
  exc.wiggle_plug_and_wait(wiggle_in_goal) 

  # Wiggle out
  exc.wiggle_plug_and_wait(wiggle_out_goal)

  # Tuck the arms
  exc.tuck_arms_and_wait(tuck_goal)

  rospy.loginfo("Plugged in and uplugged!")



if __name__ == "__main__":
  main()

