#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
from math import *
import tf

from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *

from executive_python import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from joint_trajectory_action_tools.tools import *

# State machine classes
from smach.state import *
from smach.state_machine import *
from smach.simple_action_state import *
from smach.joint_trajectory_state import *

import actionlib


class TFUtil():
  transformer = None
  def __init__(self):
    if not TFUtil.transformer:
      TFUtil.transformer = tf.TransformListener(True, rospy.Duration(60.0))  
    
  @staticmethod
  def wait_and_transform(frame_id,pose):
    try:
      TFUtil.transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return TFUtil.transformer.transformPose(frame_id, pose)

# Callback to store the plug detection result
def store_detect_plug_result(state, result_state, result):
  state.sm_userdata.gripper_to_plug = TFUtil.wait_and_transform('r_gripper_tool_frame',result.plug_pose)

def get_plugin_goal(state):
  plugin_goal = PluginGoal()
  plugin_goal.gripper_to_plug = state.sm_userdata.gripper_to_plug
  plugin_goal.base_to_outlet = state.sm_userdata.sm_goal.base_to_outlet
  return plugin_goal

def main():
  rospy.init_node("plug_in_sm",log_level=rospy.DEBUG)
  TFUtil()

  # Construct state machine
  sm = StateMachine('plug_in_sm',PlugInSMAction)

  # Default userdata
  sm.userdata.gripper_to_plug = PoseStamped()

  # Define nominal sequence
  sm.add_sequence(
      # Raise spine
      SimpleActionState('detect_plug_in_gripper',
        'detect_plug', DetectPlugInGripperAction,
        goal = DetectPlugInGripperGoal(),
        result_cb = store_detect_plug_result),

      # Perform rough base alignment
      SimpleActionState('plugin',
        'plugin', PluginAction,
        goal_cb = get_plugin_goal)
      )

  # Define recovery states

  # Populate the sm database with some stubbed out results
  # Run state machine action server with default state
  sm.run_server('detect_plug_in_gripper')
  rospy.spin()


if __name__ == "__main__":
  main()

