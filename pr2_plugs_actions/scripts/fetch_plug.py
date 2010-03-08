#!/usr/bin/env python
# stub for outlet detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from joint_trajectory_action_tools.tools import *
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

  # move to joint space position
  rospy.loginfo("Move in joint space...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/detect_plug'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move 1 in joint space failed')
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

  # call vision plug detection
  rospy.loginfo("Detecting plug...")
  detect_plug_goal.camera_name = "/forearm_camera_r"
  detect_plug_goal.prior = PoseStampedMath().fromEuler(-0.32, -0.24, 0.56, 0, -pi/2, pi).msg
  detect_plug_goal.prior.header.stamp = rospy.Time.now()
  detect_plug_goal.prior.header.frame_id = "base_link"
  detect_plug_goal.origin_on_right = True
  if detect_plug_client.send_goal_and_wait(detect_plug_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Vision plug detection failed')
    server.set_aborted()
    return
  plug_pose = detect_plug_client.get_result().plug_pose
  try:
    transformer.waitForTransform("base_link", plug_pose.header.frame_id, plug_pose.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between base_link and %s' %plug_pose.header.frame_id)
    return
  plug_pose = transformer.transformPose('base_link', plug_pose)

  # grasp plug
  cart_space_goal.ik_seed.name = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
  cart_space_goal.ik_seed.position = [-2.1060293903819898, 0.26298790696529961, -3.6540052314061247, -2.0685583319566816, -0.57887736940987111, -1.4212910723690739, -3.2345250184874357]
  pose_tf_plug = PoseStampedMath(detect_plug_client.get_result().plug_pose)
  pose_plug_gripper = PoseStampedMath()
  pose_plug_gripper.fromEuler(-.03, 0, .01, pi/2, 0, -pi/6)
  pose_tf_plug = detect_plug_client.get_result().plug_pose
  try:
    transformer.waitForTransform("base_link", pose_tf_plug.header.frame_id, pose_tf_plug.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between base_link and %s' %pose_tf_plug.header.frame_id)
    server.set_aborted()
    return
  pose_base_plug= PoseStampedMath(transformer.transformPose("base_link", pose_tf_plug))
  pose_gripper_wrist= PoseStampedMath().fromTf(transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", rospy.Time(0)))
  pose_plug_approach = PoseStampedMath().fromEuler(0, 0.05, 0, 0, 0, 0)

  cart_space_goal.pose = (pose_base_plug * pose_plug_approach * pose_plug_gripper * pose_gripper_wrist).msg
  cart_space_goal.pose.header.stamp = rospy.Time.now()
  cart_space_goal.pose.header.frame_id = 'base_link'
  cart_space_goal.move_duration = rospy.Duration(3.0)
  if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Failed to approach plug')
    server.set_aborted()
    return

  cart_space_goal.pose = (pose_base_plug * pose_plug_gripper * pose_gripper_wrist).msg
  cart_space_goal.pose.header.stamp = rospy.Time.now()
  cart_space_goal.pose.header.frame_id = 'base_link'
  cart_space_goal.move_duration = rospy.Duration(3.0)
  if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Failed to grasp plug')
    server.set_aborted()
    return

  # close gripper
  rospy.loginfo("Close gripper...")  
  gripper_goal.command.position = 0.0
  gripper_goal.command.max_effort = 99999
  gripper_client.send_goal_and_wait(gripper_goal, rospy.Duration(20.0), preempt_timeout) 

  # Pull plug off base
  joint_space_goal.trajectory.header.stamp = rospy.Time.now()
  if joint_space_client.send_goal_and_wait(joint_space_goal, rospy.Duration(5.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move plug off base failed')
    server.set_aborted()
    return

  # return result
  result = FetchPlugResult()
  #result.plug_pose = detect_plug_client.get_result().plug_pose
  result.plug_pose = plug_pose
  server.set_succeeded(result)
  rospy.loginfo("Action server goal finished")  


if __name__ == '__main__':
  #Initialize the node
  name = 'fetch_plug'
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

  detect_plug_client = actionlib.SimpleActionClient('vision_plug_detection', VisionPlugDetectionAction)
  detect_plug_client.wait_for_server()
  detect_plug_goal = VisionPlugDetectionGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, FetchPlugAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
