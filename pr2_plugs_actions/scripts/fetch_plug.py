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
  preempt_timeout = rospy.Duration(20.0)

  # move the spine up
  rospy.loginfo("Moving up spine...")
  spine_goal.position = 0.16
  spine_client.send_goal(spine_goal)

  # detect plug on base
  rospy.loginfo("Detect plug on base...")
  if detect_plug_on_base_client.send_goal_and_wait(DetectPlugOnBaseGoal(), rospy.Duration(60.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Detect plug on base failed')
    server.set_aborted()
    return

  plug_pose = detect_plug_on_base_client.get_result().plug_pose
  try:
    transformer.waitForTransform("base_link", plug_pose.header.frame_id, plug_pose.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between base_link and %s' %plug_pose.header.frame_id)
    return
  plug_pose = transformer.transformPose('base_link', plug_pose)


  # move to grasp position in joint space
  rospy.loginfo("Move in joint space to grasping position...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/grasp_plug'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move to grasping position failed')
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

  # grasp plug
  cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/grasp_plug_seed')
  pose_tf_plug = PoseStampedMath(detect_plug_on_base_client.get_result().plug_pose)
  pose_plug_gripper = PoseStampedMath()
  pose_plug_gripper.fromEuler(-.03, 0, .01, pi/2, 0, -pi/9)
  pose_tf_plug = detect_plug_on_base_client.get_result().plug_pose
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
  if gripper_client.send_goal_and_wait(gripper_goal, rospy.Duration(40.0), preempt_timeout) !=GoalStatus.ABORTED:
    rospy.logerr('The gripper did not close on the plug')
    return

  # move the spine down
  rospy.loginfo("Moving down spine...")
  spine_goal.position = 0.01
  spine_client.send_goal(spine_goal)

  # move plug off base in joint space
  rospy.loginfo("Move in joint space to remove plug from base...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/remove_plug'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move to remove plug from base failed')
    server.set_aborted()
    return

  # return result
  result = FetchPlugResult()
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

  joint_space_client = actionlib.SimpleActionClient('r_arm_plugs_controller/joint_trajectory_generator', JointTrajectoryAction)
  joint_space_client.wait_for_server()

  detect_plug_on_base_client = actionlib.SimpleActionClient('detect_plug_on_base', DetectPlugOnBaseAction)
  detect_plug_on_base_client.wait_for_server()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = PR2ArmIKGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, FetchPlugAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
