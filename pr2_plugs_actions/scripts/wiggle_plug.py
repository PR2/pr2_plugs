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


import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import PyKDL
import actionlib
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_arm_move_ik.tools import *
from tf_conversions.posemath import fromMsg, toMsg
from actionlib_msgs.msg import *
import geometry_msgs.msg
from math import pi
import math
from pr2_plugs_actions.tf_util import TFUtil

def drange(start, stop, step):
  r = start
  if start < stop:
    while r < stop:
      yield r
      r += step
  if start > stop:
    while r > stop:
      yield r
      r -= step


def outlet_to_plug_error(goal):
  time = rospy.Time.now()
  try:
    pose_base_gripper = fromMsg(TFUtil.wait_and_lookup("base_link","r_gripper_tool_frame", time).pose)
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
    server.set_aborted()
    return
  outlet_to_plug = fromMsg(goal.base_to_outlet).Inverse() * pose_base_gripper * fromMsg(goal.gripper_to_plug)
  return outlet_to_plug


def execute_cb(goal):
  rospy.loginfo("Action server received goal")
  preempt_timeout = rospy.Duration(10.0)
  cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
  try:
    pose_gripper_wrist= fromMsg(TFUtil.wait_and_lookup("r_gripper_tool_frame", "r_wrist_roll_link").pose)
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
    server.set_aborted()
    return

  pose_base_outlet = fromMsg(goal.base_to_outlet)
  pose_plug_gripper = fromMsg(goal.gripper_to_plug).Inverse()
  
  rate = rospy.Rate(100.0)
  current_error = outlet_to_plug_error(goal)
  forward_start = current_error.p[0]
  if goal.insert == 1:
    forward_stop = current_error.p[0] + 0.02
    forward_step = 0.0005
  else:
    forward_stop = current_error.p[0] - 0.04
    forward_step = 0.001

  wiggle = 1.0
  wiggle_count = 1
  for offset in drange(forward_start, forward_stop, forward_step):
    pose_outlet_plug = PyKDL.Frame(PyKDL.Rotation.RPY(0, math.pi/30*wiggle, 0), PyKDL.Vector(offset, 0, 0))
    cart_space_goal.pose.pose = toMsg(pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist)
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(0.0)

    result_state = cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout)

    if goal.insert != 1 and wiggle_count % 5 == 0 and result_state == GoalStatus.SUCCEEDED:
      server.set_succeeded(WigglePlugResult())
      return

    wiggle = wiggle * -1
    wiggle_count += 1


  pose_outlet_plug = PyKDL.Frame(PyKDL.Vector(offset, 0, 0))
  cart_space_goal.pose.pose = toMsg(pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist)
  cart_space_goal.pose.header.stamp = rospy.Time.now()
  cart_space_goal.pose.header.frame_id = 'base_link'
  cart_space_goal.move_duration = rospy.Duration(0.5)
  cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout)
  rate.sleep()

  result = WigglePlugResult()
  server.set_succeeded(result)


if __name__ == '__main__':
  #Initialize the node
  name = 'wiggle_plug'
  rospy.init_node(name)

  # transform listener
  TFUtil()

  # create action clients we use
  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', ArmMoveIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = ArmMoveIKGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, WigglePlugAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
