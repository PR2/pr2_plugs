#!/usr/bin/env python
# stub for outlet detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from actionlib_msgs.msg import *
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
import copy
import math
import tf

def drange(start, stop, step):
  r = start
  while r < stop:
    yield r
    r += step

def outlet_to_plug_error(goal):
  time = rospy.Time.now()
  try:
    transformer.waitForTransform("base_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
    server.set_aborted()
    return
  pose_base_gripper= PoseStampedMath().fromTf(transformer.lookupTransform("base_link","r_gripper_tool_frame", time))
  pose_outlet_base = PoseStampedMath(goal.base_to_outlet).inverse()
  pose_gripper_plug = PoseStampedMath(goal.gripper_to_plug)
  outlet_to_plug = (pose_outlet_base*pose_base_gripper*pose_gripper_plug).msg
  return outlet_to_plug


def execute_cb(goal):
  rospy.loginfo("Action server received goal")
  preempt_timeout = rospy.Duration(5.0)

  # make sure spine is down
  rospy.loginfo("Make sure spine is down...")
  spine_goal.position = 0.01
  if spine_client.send_goal_and_wait(spine_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Moving down spine failed')
    server.set_aborted()
    return

  # approach outlet
  cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')

  time = rospy.Time.now()
  try:
    transformer.waitForTransform("r_wrist_roll_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
    server.set_aborted()
    return
  pose_gripper_wrist= PoseStampedMath().fromTf(transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", time))
  pose_base_outlet = PoseStampedMath(goal.base_to_outlet)
  pose_plug_gripper = PoseStampedMath(goal.gripper_to_plug).inverse()

  step = 0.005
  # plug in
  for offset in drange(-0.07, 0.09, step):
    pose_outlet_plug = PoseStampedMath().fromEuler(offset, 0, 0, 0, 0, 0)
    cart_space_goal.pose = (pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(1.5)
    if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.loginfo("hit the wall or inserted into outlet")
      break

  #get current error 
  start_error = outlet_to_plug_error(goal)  
  outlet_test = [0.01, -0.01, 0.0]
  for motion in outlet_test:
    pose_outlet_plug = PoseStampedMath().fromEuler(offset, 0, motion, 0, 0, 0)
    cart_space_goal.pose = (pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(1.5)
    cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout)
    plug_error = outlet_to_plug_error(goal)
    print math.fabs(plug_error.pose.position.z - start_error.pose.position.z)
    if (math.fabs(plug_error.pose.position.z -start_error.pose.position.z) >= math.fabs(motion)/2) and (motion != 0.0 ):
      rospy.logerr("we're not in the outlet")
      pose_outlet_plug = PoseStampedMath().fromEuler(offset-0.05, 0, 0, 0, 0, 0)
      cart_space_goal.pose = (pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist).msg
      cart_space_goal.pose.header.stamp = rospy.Time.now()
      cart_space_goal.pose.header.frame_id = 'base_link'
      cart_space_goal.move_duration = rospy.Duration(1.5)
      cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout)
      server.set_aborted()
      return



  # return result
  result = PluginResult()
  server.set_succeeded(result)
  rospy.loginfo("Action server goal finished")



if __name__ == '__main__':
  #Initialize the node
  name = 'plugin'
  rospy.init_node(name)

  # transform listener
  transformer = tf.TransformListener()

  # create action clients we use
  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = PR2ArmIKGoal()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  rospy.loginfo('Connected to action clients')


  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, PluginAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
