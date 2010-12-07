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
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of the Willow Garage nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
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
import rospy;
import PyKDL
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from joint_trajectory_action_tools.tools import *
from pr2_arm_move_ik.tools import *
from tf_conversions.posemath import fromMsg, toMsg
from actionlib_msgs.msg import *
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
from pr2_plugs_actions.tf_util import TFUtil
from pr2_image_snapshot_recorder.msg import ImageSnapshotAction, ImageSnapshotGoal

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


  # return plug
  error = 1.0
  free_cord = False
  while error > 0.01 and not rospy.is_shutdown():
    # appraoch in joint space when re-trying
    if free_cord:
      rospy.loginfo("Free plug cord by doing approach...")
      if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/free_plug_cord'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Free plug cord failed')
        server.set_aborted()
        return
    free_cord = True

    # move to joint space position
    rospy.loginfo("Move in joint space...")
    if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/stow_plug'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Move approach in joint space failed')
      server.set_aborted()
      return

    # return plug to original location
    cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/return_plug_seed')
    pose_plug_gripper = fromMsg(goal.gripper_to_plug_grasp).Inverse()  
    pose_base_plug = fromMsg(goal.base_to_plug)
    pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link').pose)
    cart_space_goal.pose.pose = toMsg(pose_base_plug * pose_plug_gripper * pose_gripper_wrist)
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = "base_link"
    cart_space_goal.move_duration = rospy.Duration(3.0)
    if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Failed to return plug to location where the plug was found when fetching it.')
      server.set_aborted()
      return

    # check if plug in correct location
    pose_base_gripper_measured = fromMsg(TFUtil.wait_and_lookup('base_link', 'r_gripper_tool_frame', rospy.Time.now()).pose)  
    pose_base_gripper_desired = pose_base_plug * pose_plug_gripper
    diff = pose_base_gripper_measured.Inverse() * pose_base_gripper_desired
    error = diff.p.Norm()
    (r, p, y) = diff.M.GetRPY()
    error += (r + p + y)/10.0
    rospy.loginfo("Diff between desired and actual plug storing poses = %f"%error)

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
  pose_plug_approach = PyKDL.Frame(PyKDL.Vector(0, 0.05, 0))
  cart_space_goal.pose.pose = toMsg(pose_base_plug * pose_plug_approach * pose_plug_gripper * pose_gripper_wrist)
  cart_space_goal.pose.header.stamp = rospy.Time.now()
  cart_space_goal.pose.header.frame_id = "base_link"
  cart_space_goal.move_duration = rospy.Duration(3.0)
  if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Failed to move arm away from where the plug got released')
    server.set_aborted()
    return

  # detect plug on base
  rospy.loginfo("Detect plug on base...")
  start_time = rospy.Time.now()
  while detect_plug_on_base_client.send_goal_and_wait(DetectPlugOnBaseGoal(), rospy.Duration(120.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    if rospy.Time.now() > start_time + rospy.Duration(60*5):
      rospy.logerr("Can't detect plug on base. It is soo dark in here... Giving up, driving away, living on the edge!  But let's take an image to make sure.")
      image_client = actionlib.SimpleActionClient('image_snapshot', ImageSnapshotAction)
      if not image_client.wait_for_server(rospy.Duration(20.0)):
        rospy.logerr("Imagesnapshot server is down.")
        break
      image_goal = ImageSnapshotGoal()
      image_goal.topic_name = 'r_forearm_cam/image_raw'
      image_goal.num_images = 5
      image_goal.output_file_name = '/removable/continuous_operation/stow_plug_failure_images.bag'
      image_client.send_goal_and_wait(image_goal)
      break
    rospy.logerr('Detecting plug on base failed, trying again...')

  # move the spine down
  rospy.loginfo("Moving down spine...")
  spine_goal.position = 0.01
  spine_client.send_goal(spine_goal)

  # return result
  server.set_succeeded(StowPlugResult())
  rospy.loginfo("Action server goal finished")  


if __name__ == '__main__':
  #Initialize the node
  name = 'stow_plug'
  rospy.init_node(name)

  # transform listener
  TFUtil()

  # create action clients we use
  gripper_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
  gripper_client.wait_for_server()
  gripper_goal = Pr2GripperCommandGoal()

  joint_space_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_generator', JointTrajectoryAction)
  joint_space_client.wait_for_server()
  joint_space_goal = JointTrajectoryGoal()

  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', ArmMoveIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = ArmMoveIKGoal()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  detect_plug_on_base_client = actionlib.SimpleActionClient('detect_plug_on_base', DetectPlugOnBaseAction)
  detect_plug_on_base_client.wait_for_server()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, StowPlugAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
