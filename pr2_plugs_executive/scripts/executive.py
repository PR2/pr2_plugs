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
from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from std_srvs.srv import *

import actionlib

# Declare list of actions for easy construction
actions = [
    ('tuck_arms',TuckArmsAction),
    ('detect_outlet',DetectOutletAction),
    ('fetch_plug',FetchPlugAction),
    ('detect_plug',DetectPlugInGripperAction),
    ('wiggle_plug',WigglePlugAction),
    ('stow_plug',StowPlugAction),
    ('plugin',PluginAction)]

def main():
  rospy.init_node("plugs_executive")

  # construct tf listener
  transformer = tf.TransformListener(True, rospy.Duration(60.0))  
  preempt_timeout = rospy.Duration(5.0)

  # Construct action ac
  rospy.loginfo("Starting actions.")
  ac = dict()
  for (name,action) in actions:
    ac[name] = actionlib.SimpleActionClient(name,action)

  # Wait for all the ac to start
  for (name,action) in actions:
    print "Wait for server "+name
    ac[name].wait_for_server()
  rospy.loginfo("All actions started.")

  # Untuck the arms
  untuck_goal = TuckArmsGoal()
  untuck_goal.untuck=True
  untuck_goal.left=False
  untuck_goal.right=True
  untucked = False
  while not untucked:
    rospy.loginfo("Untucking right arm...")
    untucked = (ac['tuck_arms'].send_goal_and_wait(untuck_goal, rospy.Duration(30.0), preempt_timeout) == GoalStatus.SUCCEEDED)


  # Detect the outlet 
  detect_outlet_goal = DetectOutletGoal()
  rospy.loginfo("Detecting the outlet ...")
  if ac['detect_outlet'].send_goal_and_wait(detect_outlet_goal, rospy.Duration(90.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect the outlet!")
    return
  outlet_pose = ac['detect_outlet'].get_result().outlet_pose
  try:
    transformer.waitForTransform("base_link", outlet_pose.header.frame_id, outlet_pose.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between base_link and %s' %outlet_pose.header.frame_id)
    return
  base_to_outlet = transformer.transformPose('base_link', outlet_pose)


  # Fetch plug
  rospy.loginfo('Fetching plug...')
  if ac['fetch_plug'].send_goal_and_wait(FetchPlugGoal(), rospy.Duration(60.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to fetch plug!")
    return
  plug_pose = ac['fetch_plug'].get_result().plug_pose
  try:
    transformer.waitForTransform("base_link", plug_pose.header.frame_id, plug_pose.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between base_link and %s' %plug_pose.header.frame_id)
    return
  base_to_plug = transformer.transformPose('base_link', plug_pose)


  # Detect the plug in gripper
  detect_plug_goal = DetectPlugInGripperGoal()
  rospy.loginfo('Detecting plug in gripper...')
  if ac['detect_plug'].send_goal_and_wait(detect_plug_goal, rospy.Duration(30.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect plug in gripper!")
    return
  plug_pose = ac['detect_plug'].get_result().plug_pose
  try:
    transformer.waitForTransform("r_gripper_tool_frame", plug_pose.header.frame_id, plug_pose.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between r_gripper_tool_frame and %s' %plug_pose.header.frame_id)
    return
  gripper_to_plug = transformer.transformPose('r_gripper_tool_frame', plug_pose)

  # Plug in
  plugin_goal = PluginGoal()
  plugin_goal.gripper_to_plug = gripper_to_plug
  plugin_goal.base_to_outlet = base_to_outlet
  rospy.loginfo('Plugging in...')
  if ac['plugin'].send_goal_and_wait(plugin_goal, rospy.Duration(60.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to plug in!")
    return

  # Stow plug
  rospy.loginfo('Stowing plug...')
  stow_plug_goal = StowPlugGoal()
  stow_plug_goal.gripper_to_plug = gripper_to_plug
  stow_plug_goal.base_to_plug = base_to_plug
  if ac['stow_plug'].send_goal_and_wait(stow_plug_goal, rospy.Duration(45.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to stow plug!")
    return

  # Tuck the arms
  tuck_goal = TuckArmsGoal()
  tuck_goal.untuck=False
  tuck_goal.left=False
  tuck_goal.right=True

  rospy.loginfo("Tucking arms...")
  ac['tuck_arms'].send_goal_and_wait(tuck_goal, rospy.Duration(30.0), preempt_timeout)

  return

  # Wiggle in
  rospy.loginfo('Wiggling in...')
  wiggle_goal = WigglePlugGoal()
  wiggle_goal.travel.x = 0.05
  wiggle_goal.offset.y = 0.01
  wiggle_goal.period = rospy.Duration(2.0)
  wiggle_goal.move_duration = rospy.Duration(10.0)
  wiggle_goal.abort_threshold = 0.02
  if ac['wiggle_plug'].send_goal_and_wait(wiggle_goal, rospy.Duration(30.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to wiggle plug in!")
    return

  # Wiggle out
  rospy.loginfo('Wiggling out...')
  wiggle_goal.travel.x = -0.08
  wiggle_goal.offset.y = 0.01
  wiggle_goal.period = rospy.Duration(2.0)
  wiggle_goal.move_duration = rospy.Duration(10.0)
  if ac['wiggle_plug'].send_goal_and_wait(wiggle_goal, rospy.Duration(30.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to wiggle plug out!")
    return

  rospy.loginfo("Plugged in!")


if __name__ == "__main__":
  main()

