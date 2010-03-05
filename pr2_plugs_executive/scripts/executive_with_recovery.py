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
import threading
import tf

from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from std_srvs.srv import *

import actionlib

def ExecutionException(Exception):
  def __init__(self):
    pass

class Executive():
  def __init__(self,actions):
    # construct tf listener
    self.transformer = tf.TransformListener(True, rospy.Duration(60.0))  

    self.preempt_timeout = rospy.Duration(5.0)

    # Declare list of actions for easy construction
    self.actions = actions
    self.action_clients = dict()

    # Construct action clients
    rospy.loginfo("Starting action clients.")
    for name, action_spec in self.actions.iteritems():
      # Create an action client
      ac = actionlib.SimpleActionClient(name,action_spec)
      # Store the action client in the actionclient dictionary for iteration
      self.action_clients[name] = ac

      # Set this action client as a member of this class for convenience
      assert not hasattr(self, name)
      setattr(self, name+"_client", ac)

      # Construct functions that call the actions with default timeouts.
      # This is convienent for actions with an invariant timeout, since it's a
      # property of the action and not of the goal
      assert not hasattr(self, name+"_and_wait")
      setattr(self, name+"_and_wait",
          lambda goal,timeout=self.actions[name][1]:
            rospy.loginfo("Sending blocking goal to "+name+" action...")
            ac.send_goal_and_wait(goal,rospy.Duration(timeout),self.preempt_timeout)
          )

    # Wait for all the action clients to start (If we do this in parallel it happens a lot faster)
    action_wait_threads = []
    for name, ac in self.action_clients.iteritems():
      rospy.loginfo("Waiting for \""+name+"\" action server...")
      action_wait_threads.append(threading.Thread(target=ac.wait_for_server))
      action_wait_threads[-1].start()
    [thread.join() for thread in action_wait_threads]
    rospy.loginfo("All action servers ready.")


  def wait_and_transform(self,frame_id,pose):
    try:
      self.transformer.waitForTransform(frame, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return self.transformer.transformPose(frame_id, pose)

def main():
  rospy.init_node("plugs_executive")

  actions = {
      'tuck_arms':(TuckArmsAction, 30.0),
      'detect_wall_norm':(DetectWallNormAction, 30.0),
      'detect_outlet':(DetectOutletAction, 90.0),
      'fetch_plug':(FetchPlugAction, 60.0),
      'detect_plug':(DetectPlugInGripperAction, 30.0),
      'wiggle_plug':(WigglePlugAction, 30.0),
      'stow_plug':(StowPlugAction, 45.0),
      'plugin':(PluginAction, 60.0)
      }

  # Construct executive
  exc = Executive(actions)

  # Untuck the arms
  untuck_goal = TuckArmsGoal(untuck=True,left=False,right=True)
  untucked = False
  while not untucked:
    untucked = (exc.tuck_arms_and_wait(untuck_goal) == GoalStatus.SUCCEEDED)

  # Detect the outlet 
  detect_outlet_goal = DetectOutletGoal()
  if exc.detect_outlet_and_wait(detect_outlet_goal) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect the outlet!")
    return
  outlet_pose = exc.detect_outlet.get_result().outlet_pose
  base_to_outlet = exc.wait_and_transform("base_link",outlet_pose)

  # Fetch plug
  if exc.fetch_plug_and_wait(FetchPlugGoal()) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to fetch plug!")
    return
  plug_pose = exc.fetch_plug.get_result().plug_pose
  base_to_plug = wait_and_transform("base_link",plug_pose)

  # Detect the plug in gripper
  detect_plug_goal = DetectPlugInGripperGoal()
  if exc.detect_plug_and_wait(detect_plug_goal) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to detect plug in gripper!")
    return
  plug_pose = exc.detect_plug.get_result().plug_pose
  gripper_to_plug = wait_and_transform('r_gripper_tool_frame', plug_pose)

  # Plug in
  plugin_goal = PluginGoal()
  plugin_goal.gripper_to_plug = gripper_to_plug
  plugin_goal.base_to_outlet = base_to_outlet
  if exc.plugin_and_wait(plugin_goal) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to plug in!")
    return

  # Stow plug
  stow_plug_goal = StowPlugGoal()
  stow_plug_goal.gripper_to_plug = gripper_to_plug
  stow_plug_goal.base_to_plug = base_to_plug
  if exc.stow_plug_and_wait(stow_plug_goal) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to stow plug!")
    return

  # Tuck the arms
  tuck_goal = TuckArmsGoal(unutck=False,left=False,right=True)
  exc.tuck_arms_and_wait(tuck_goal)

  return

  # Wiggle in
  wiggle_goal = WigglePlugGoal()
  wiggle_goal.travel.x = 0.05
  wiggle_goal.offset.y = 0.01
  wiggle_goal.period = rospy.Duration(2.0)
  wiggle_goal.move_duration = rospy.Duration(10.0)
  wiggle_goal.abort_threshold = 0.02
  if exc.wiggle_plug_and_wait(wiggle_goal) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to wiggle plug in!")
    return

  # Wiggle out
  wiggle_goal.travel.x = -0.08
  wiggle_goal.offset.y = 0.01
  wiggle_goal.period = rospy.Duration(2.0)
  wiggle_goal.move_duration = rospy.Duration(10.0)
  if exc.wiggle_plug_and_wait(wiggle_goal) != GoalStatus.SUCCEEDED:
    rospy.logerr("Failed to wiggle plug out!")
    return

  rospy.loginfo("Plugged in and uplugged!")


if __name__ == "__main__":
  main()

