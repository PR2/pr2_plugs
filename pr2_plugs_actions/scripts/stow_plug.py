#!/usr/bin/env python
# stub for outlet detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from joint_trajectory_action_tools.tools import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from actionlib_msgs.msg import *
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
import copy
import math
import tf

#server actionlib.simple_action_server.SimpleActionServer

def execute_cb(goal):
  rospy.loginfo("Action server received goal")
  preempt_timeout = rospy.Duration(5.0)

  # move the spine up
  rospy.loginfo("Moving up spine...")
  spine_goal.position = 0.16
  if spine_client.send_goal_and_wait(spine_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Moving up spine failed')
    server.set_aborted()
    return

  # move to joint space position
  rospy.loginfo("Move in joint space...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/stow_plug'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move approach in joint space failed')
    server.set_aborted()
    return

  # return plug
  cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/return_plug_seed')

  pose_plug_gripper = PoseStampedMath(goal.gripper_to_plug).inverse()  
  pose_base_plug = PoseStampedMath(goal.base_to_plug)  
  pose_gripper_wrist = PoseStampedMath()
  time = rospy.Time.now()
  try:
    transformer.waitForTransform("r_wrist_roll_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
    server.set_aborted()
    return
  pose_gripper_wrist.fromTf(transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", time))

  cart_space_goal.pose = (pose_base_plug * pose_plug_gripper * pose_gripper_wrist).msg
  cart_space_goal.pose.header.stamp = rospy.Time.now()
  cart_space_goal.pose.header.frame_id = "base_link"
  cart_space_goal.move_duration = rospy.Duration(3.0)
  if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Failed to grasp plug')
    server.set_aborted()
    return

  # open gripper
  rospy.loginfo("Open gripper...")  
  gripper_goal.command.position = 0.07
  gripper_goal.command.max_effort = 99999
  if gripper_client.send_goal_and_wait(gripper_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Open gripper failed')
    server.set_aborted()
    return

  # retract arm
  rospy.loginfo("Releasing plug...")  
  pose_plug_approach = PoseStampedMath().fromEuler(0, 0.05, 0, 0, 0, 0)
  cart_space_goal.pose = (pose_base_plug * pose_plug_approach * pose_plug_gripper * pose_gripper_wrist).msg
  cart_space_goal.pose.header.stamp = rospy.Time.now()
  cart_space_goal.pose.header.frame_id = "base_link"
  cart_space_goal.move_duration = rospy.Duration(3.0)
  if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Failed to release plug')
    server.set_aborted()
    return

  # move to joint space position
  rospy.loginfo("Move in joint space...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/verify_plug_on_base'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move retract in joint space failed')
    server.set_aborted()
    return

  # call vision plug detection
  rospy.loginfo("Detecting plug...")
  detect_plug_goal.camera_name = "/forearm_camera_r"
  detect_plug_goal.prior = PoseStampedMath().fromEuler(0.075, 0.03, 0.24, pi/2, 0, pi/2).msg
  detect_plug_goal.prior.header.stamp = rospy.Time.now()
  detect_plug_goal.prior.header.frame_id = "base_link"
  detect_plug_goal.origin_on_right = True
  if detect_plug_client.send_goal_and_wait(detect_plug_goal, rospy.Duration(10.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Vision plug detection failed')
    server.set_aborted()
    return

  # move the spine down
  rospy.loginfo("Moving down spine...")
  spine_goal.position = 0.01
  spine_client.send_goal(spine_goal)

  # move to joint space position
  rospy.loginfo("Move in joint space...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/clear_arm'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move in joint space failed')
    server.set_aborted()
    return

  # return result
  server.set_succeeded(StowPlugResult())
  rospy.loginfo("Action server goal finished")  


if __name__ == '__main__':
  #Initialize the node
  name = 'stow_plug'
  rospy.init_node(name)

  # transform listener
  transformer = tf.TransformListener()

  # create action clients we use
  gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
  gripper_client.wait_for_server()
  gripper_goal = Pr2GripperCommandGoal()

  joint_space_client = actionlib.SimpleActionClient('r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction)
  joint_space_client.wait_for_server()
  joint_space_goal = JointTrajectoryGoal()

  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = PR2ArmIKGoal()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  detect_plug_client = actionlib.SimpleActionClient('vision_plug_detection', VisionPlugDetectionAction)
  detect_plug_client.wait_for_server()
  detect_plug_goal = VisionPlugDetectionGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, StowPlugAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
