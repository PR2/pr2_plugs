#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
from math import *
import copy
import math
import tf

from std_srvs.srv import *

from actionlib_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

from executive_python import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from geometry_msgs.msg import *
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

# Code block state for grasping the plug
class GraspPlugState(State):
  def enter(self):
    cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
    cart_space_client.wait_for_server()
    cart_space_goal = PR2ArmIKGoal()

    preempt_timeout = rospy.Duration(5.0)
    # Grab relevant user data
    pose_tf_plug = self.sm_userdata.sm_result.plug_on_base_pose

    # Get grasp plug IK seed
    cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/grasp_plug_seed')

    # Define the desired grasp on te plug
    pose_plug_gripper = PoseStampedMath()
    pose_plug_gripper.fromEuler(-.03, 0, .01, pi/2, 0, -pi/9)

    pose_base_plug= PoseStampedMath(TFUtil.transformer.transformPose("base_link", pose_tf_plug))
    pose_gripper_wrist= PoseStampedMath().fromTf(TFUtil.transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", rospy.Time(0)))
    pose_plug_approach = PoseStampedMath().fromEuler(0, 0.05, 0, 0, 0, 0)

    cart_space_goal.pose = (pose_base_plug * pose_plug_approach * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(3.0)
    if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Failed to approach plug')
      self.set_aborted()
      return

    cart_space_goal.pose = (pose_base_plug * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(3.0)
    if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Failed to grasp plug')
      self.set_aborted()
      return

    self.set_succeeded()

# Callback to store the plug detection result
def store_detect_plug_result(state, result_state, result):
  if result_state == GoalStatus.SUCCEEDED:
    state.sm_userdata.sm_result.plug_on_base_pose = TFUtil.wait_and_transform('base_link',result.plug_pose) 

def main():
  rospy.init_node("fetch_plug_sm")#,log_level=rospy.DEBUG)

  TFUtil()
  # Define fixed goals

  # Open gripper goal
  open_gripper_goal = Pr2GripperCommandGoal()
  open_gripper_goal.command.position = 0.07
  open_gripper_goal.command.max_effort = 99999

  # Close gripper goal
  close_gripper_goal = Pr2GripperCommandGoal()
  close_gripper_goal.command.position = 0.0
  close_gripper_goal.command.max_effort = 99999

  # Construct state machine
  sm = StateMachine('fetch_plug_sm',FetchPlugSMAction)

  # Default userdata
  sm.userdata.plug_pose = PoseStamped()

  # Define nominal sequence
  sm.add_sequence(
      # Raise spine
      SimpleActionState('raise_spine',
        'torso_controller/position_joint_action', SingleJointPositionAction,
        goal = SingleJointPositionGoal(position=0.16)),

      # Move arm to detect the plug on the base
      JointTrajectoryState('move_arm_base_detect_pose',
        'r_arm_plugs_controller','pr2_plugs_configuration/detect_plug_on_base'),

      # Detect the plug
      SimpleActionState('detect_plug_on_base',
        'detect_plug_on_base',DetectPlugOnBaseAction,
        exec_timeout = rospy.Duration(120.0),
        aborted = 'move_arm_base_detect_pose',
        preempted = 'move_arm_base_detect_pose',
        goal = DetectPlugOnBaseGoal(),
        result_cb = store_detect_plug_result),

      # Move arm to the grasp pose
      JointTrajectoryState('move_arm_base_grasp_pose',
        'r_arm_plugs_controller','pr2_plugs_configuration/grasp_plug',
        aborted = 'recover_grasp_to_detect_pose'),

      # Open the gripper
      SimpleActionState('open_gripper',
        'r_gripper_controller/gripper_action', Pr2GripperCommandAction,
        goal = open_gripper_goal),

      # Grasp the plug
      GraspPlugState('grasp_plug',aborted = 'recover_grasp_to_detect_pose'),

      # Close the gripper
      SimpleActionState('close_gripper',
        'r_gripper_controller/gripper_action', Pr2GripperCommandAction,
        goal = close_gripper_goal,
        succeeded='recover_grasp_to_detect_pose',
        aborted='move_arm_remove_plug')
      )
  sm.add_sequence(
      # Remove the plug form the base
      JointTrajectoryState('move_arm_remove_plug',
        'r_arm_plugs_controller','pr2_plugs_configuration/remove_plug'),
      
      # Lower the spine
      SimpleActionState('lower_spine',
        'torso_controller/position_joint_action', SingleJointPositionAction,
        goal = SingleJointPositionGoal(position=0.01))
      )

  # Define recovery states
  sm.add(JointTrajectoryState('recover_grasp_to_detect_pose',
    'r_arm_plugs_controller','pr2_plugs_configuration/recover_grasp_to_detect',
    succeeded = 'detect_plug_on_base',
    aborted = 'recover_grasp_to_detect_pose'))

  # Populate the sm database with some stubbed out results
  # Run state machine action server with default state
  sm.run_server('raise_spine')
  rospy.spin()


if __name__ == "__main__":
  main()

