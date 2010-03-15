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
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
import threading
import tf

from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from std_srvs.srv import *
from executive_python import *

# State machine classes
from smach.state import *
from smach.state_machine import *
from smach.simple_action_state import *

import actionlib

class TFUtil():
  transformer = None
    
  @staticmethod
  def wait_and_transform(frame_id,pose):
    if not TFUtil.transformer:
      TFUtil.transformer = tf.TransformListener(True, rospy.Duration(60.0))  

    try:
      TFUtil.transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return TFUtil.transformer.transformPose(frame_id, pose)

def get_move_base_goal(state):
  # Get id from command
  plug_id = state.sm_userdata.sm_goal.command.plug_id

  # Get pose of desired id from the parameter server
  position = rospy.get_param("outlet_approach_poses/"+plug_id+"/position")
  orientation = rospy.get_param("outlet_approach_poses/"+plug_id+"/orientation")
  target_pose = PoseStamped(pose = Pose(
      position = Point(*position),
      orientation = Quaternion(*orientation)))

  # Create goal for move base
  move_base_goal = MoveBaseGoal()
  move_base_goal.target_pose = target_pose
  move_base_goal.target_pose.header.stamp = rospy.Time.now()
  move_base_goal.target_pose.header.frame_id = "map"

  return move_base_goal

def store_outlet_result(state, result_state, result):
  state.sm_userdata.outlet_pose = result.outlet_pose

def store_fetch_plug_result(state, result_state, result):
  state.sm_userdata.plug_on_base_pose = result.plug_on_base_pose

def get_plug_in_goal(state):
  # Update timestamps
  state.sm_userdata.outlet_pose.header.stamp = rospy.Time.now()
  # Return goal
  return PlugInSMGoal(base_to_outlet = state.sm_userdata.outlet_pose)

def store_plug_in_result(state, result_state, result):
  state.sm_userdata.plug_in_gripper_pose = result.gripper_to_plug
  if result_state is GoalStatus.SUCCEEDED:
    state.sm_userdata.sm_result.state.state = RechargeState.PLUGGED_IN


def get_wiggle_out_goal(state):
  wiggle_goal = WigglePlugGoal()
  wiggle_goal.gripper_to_plug = state.sm_userdata.plug_in_gripper_pose
  wiggle_goal.gripper_to_plug.header.stamp = rospy.Time.now()

  wiggle_goal.base_to_outlet = state.sm_userdata.outlet_pose
  wiggle_goal.base_to_outlet.header.stamp = rospy.Time.now()

  wiggle_goal.wiggle_period = rospy.Duration(0.5)
  wiggle_goal.insert = 0
  return wiggle_goal

def get_stow_plug_goal(state):
  stow_plug_goal = StowPlugGoal()
  stow_plug_goal.gripper_to_plug = state.sm_userdata.plug_in_gripper_pose
  stow_plug_goal.base_to_plug = state.sm_userdata.plug_on_base_pose
  return stow_plug_goal

def set_unplug_result(state, result_state, result):
  if result_state is GoalStatus.SUCCEEDED:
    state.sm_userdata.sm_result.state.state = RechargeState.UNPLUGGED

# Define state to process the recharge goal
class ProcessRechargeCommandState(State):
  def enter(self):
    # Process the command to determine if we should plug in or unplug
    command = self.sm_userdata.sm_goal.command.command
    if command is RechargeCommand.PLUG_IN:
      self.next_state_label = "navigate_and_plug_in"
    elif command is RechargeCommand.UNPLUG:
      self.next_state_label = "unplug"

def main():
  rospy.init_node("recharge_toplevel")

  # Close gripper goal
  close_gripper_goal = Pr2GripperCommandGoal()
  close_gripper_goal.command.position = 0.0
  close_gripper_goal.command.max_effort = 99999

  # Construct state machine
  sm = StateMachine('recharge',RechargeSMAction)

  # Default userdata fields
  sm.userdata.outlet_pose = PoseStamped()
  sm.userdata.plug_on_base_pose = PoseStamped()
  sm.userdata.plug_in_gripper_pose = PoseStamped()

  # Define entry states
  sm.add(ProcessRechargeCommandState('process_recharge_command'))

  # Define nominal plug-in sequence
  sm.add_sequence(
      # Meta-state for sending commands
      EmptyState('navigate_and_plug_in'),
      # Tuck the arms
      SimpleActionState('tuck','tuck_arms',TuckArmsAction,
        goal = TuckArmsGoal(False,True,True)),
      # Navigate to the requested outlet
      SimpleActionState('navigate_to_outlet','move_base',MoveBaseAction,
        exec_timeout = rospy.Duration(20*60.0),
        goal_cb = get_move_base_goal, aborted='navigate_to_outlet'),
      # Untuck the arms
      SimpleActionState('untuck','tuck_arms',TuckArmsAction,
        goal = TuckArmsGoal(True,True,True),
        aborted='untuck'),
      # Perform outlet detection
      SimpleActionState('detect_outlet','detect_outlet_sm',DetectOutletSMAction,
        exec_timeout = rospy.Duration(300.0),
        goal = DetectOutletSMGoal(),
        result_cb = store_outlet_result,
        aborted="tuck"),
      # Once we have the outlet pose, we will fetch the plug and plug in
      SimpleActionState('fetch_plug','fetch_plug_sm',FetchPlugSMAction,
        exec_timeout=rospy.Duration(300.0),
        goal = FetchPlugSMGoal(),
        result_cb = store_fetch_plug_result,
        aborted='fetch_plug' ),
      # Re-detect the plug in the gripper, plug, and wiggle in
      SimpleActionState('plug_in','plug_in_sm',PlugInSMAction,
        exec_timeout = rospy.Duration(100.0),
        goal = PluginGoal(),
        goal_cb = get_plug_in_goal,
        result_cb = store_plug_in_result,
        aborted = 'recover_stow_plug')
      )

  # Add recovery states
  sm.add_sequence(
      # Move the arm back from the outlet so we don't scratch the wall
      JointTrajectoryState('recover_move_arm_remove_plug',
        'r_arm_plugs_controller','pr2_plugs_configuration/recover_remove_plug_from_outlet'),
      # Stow the plug
      SimpleActionState('recover_stow_plug','stow_plug',StowPlugAction,
        goal_cb = get_stow_plug_goal,succeeded = 'detect_outlet')
      )

  # Define nominal unplug sequence
  sm.add_sequence(
      # Meta-state for sending commands
      EmptyState('unplug'),
      # Make sure the gripper is held tightly
      SimpleActionState('close_gripper',
        'r_gripper_controller/gripper_action', Pr2GripperCommandAction,
        goal = close_gripper_goal,
        succeeded='ABORTED',
        aborted='wiggle_out'),
      )
  sm.add_sequence(
      # Wiggle plug out
      SimpleActionState('wiggle_out','wiggle_plug',WigglePlugAction,
        goal_cb = get_wiggle_out_goal),
      # Stow plug
      SimpleActionState('stow_plug','stow_plug',StowPlugAction,
        goal_cb = get_stow_plug_goal,
        result_cb = set_unplug_result)
      )

  # Populate the sm database with some stubbed out results
  # Run state machine action server with no default state
  sm.run_server('process_recharge_command')
  rospy.spin()


if __name__ == "__main__":
  main()

