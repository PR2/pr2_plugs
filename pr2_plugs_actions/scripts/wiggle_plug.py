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
from math import pi
import math
import tf

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
  preempt_timeout = rospy.Duration(10.0)

  
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
  
  rate = rospy.Rate(100.0)
  current_error = outlet_to_plug_error(goal)
  forward_start = current_error.pose.position.x
  if goal.insert == 1:
    forward_stop = current_error.pose.position.x + 0.02
    forward_step = 0.0005
  else:
    forward_stop = current_error.pose.position.x - 0.04
    forward_step = 0.001

  wiggle = 1.0
  wiggle_count = 1
  for offset in drange(forward_start, forward_stop, forward_step):
    pose_outlet_plug = PoseStampedMath().fromEuler(offset, 0, 0, 0, math.pi/30*wiggle, 0)
    cart_space_goal.pose = (pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(0.0)

    result_state = cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout)

    if goal.insert != 1 and wiggle_count % 5 == 0 and result_state == GoalStatus.SUCCEEDED:
      server.set_succeeded(WigglePlugResult())
      return

    wiggle = wiggle * -1
    wiggle_count += 1
    control_error = (PoseStampedMath(outlet_to_plug_error(goal)) * pose_outlet_plug.inverse()).msg
#    rospy.loginfo('Control error in x direction is %f' %control_error.pose.position.x)
#    rospy.loginfo('Current commanded depth is %f'%offset)
#    rospy.loginfo('Measured depth is %f' %outlet_to_plug_error(goal).pose.position.x)

  pose_outlet_plug = PoseStampedMath().fromEuler(offset, 0, 0, 0, 0, 0)
  cart_space_goal.pose = (pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist).msg
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
  transformer = tf.TransformListener()

  # create action clients we use
  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = PR2ArmIKGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, WigglePlugAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
