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

from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from joint_trajectory_action_tools.tools import *

# State machine classes
from smach import *

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
def store_detect_plug_result(ud, result_state, result):
  ud.sm_result.gripper_to_plug = TFUtil.wait_and_transform('r_gripper_tool_frame',result.plug_pose)

def get_plugin_goal(ud,goal):
  goal = PluginGoal()
  goal.gripper_to_plug = ud.sm_result.gripper_to_plug
  goal.base_to_outlet = ud.sm_goal.base_to_outlet
  return goal

def get_wiggle_goal(ud,goal):
  goal = WigglePlugGoal()
  goal.gripper_to_plug = ud.sm_result.gripper_to_plug
  goal.gripper_to_plug.header.stamp = rospy.Time.now()

  goal.base_to_outlet = ud.sm_goal.base_to_outlet
  goal.base_to_outlet.header.stamp = rospy.Time.now()

  goal.wiggle_period = rospy.Duration(0.5)
  goal.insert = 1
  return goal

def main():
  rospy.init_node("plug_in")#,log_level=rospy.DEBUG)
  TFUtil()

  # Construct state machine
  sm = StateMachine(['succeeded','aborted','preempted'])

  # Define nominal sequence
  sm.add(
      state_machine.sequence('succeeded',
        ('DETECT_PLUG_IN_GRIPPER',
          SimpleActionState('detect_plug',
            DetectPlugInGripperAction,
            goal = DetectPlugInGripperGoal(),
            result_cb = store_detect_plug_result),
          {}),

        ('INSERT_PLUG',
          SimpleActionState('plugin', PluginAction,
            goal_cb = get_plugin_goal),
          {}),

        ('WIGGLE_IN',
          SimpleActionState('wiggle_plug',
            WigglePlugAction,
            goal_cb = get_wiggle_goal),
          {'succeeded':'succeeded'} ) ) )

  # Set the initial state
  sm.set_initial_state(['DETECT_PLUG_IN_GRIPPER'])

  # Run state machine introspection server
  intro_server = smach.IntrospectionServer('plug_in',sm,'/RECHARGE/PLUG_IN')
  intro_server.start()

  # Run state machine action server 
  sms = ActionServerStateMachine(
      'plug_in', PlugInAction, sm,
      succeeded_outcomes = ['succeeded'],
      aborted_outcomes = ['aborted'],
      preempted_outcomes = ['preempted']
      )
  sms.run_server()

  rospy.spin()

  intro_server.stop()

if __name__ == "__main__":
  main()

