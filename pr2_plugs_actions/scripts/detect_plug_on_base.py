#!/usr/bin/env python

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


def to_init_position():
  rospy.loginfo("Move in joint space...")
  if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/detect_plug_on_base0'), rospy.Duration(20.0), rospy.Duration(5.0)) != GoalStatus.SUCCEEDED:
    rospy.logerr('Move retract in joint space failed')
    server.set_aborted()
    return



def execute_cb(goal):
  rospy.loginfo("Action server received goal")
  preempt_timeout = rospy.Duration(5.0)

  # move spine up
  rospy.loginfo("Moving up spine...")
  spine_goal.position = 0.16
  if spine_client.send_goal_and_wait(spine_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
    rospy.logerr('Moving up spine failed')
    server.set_aborted()
    return

  # we have 5 different plug detection poses
  to_init_position()
  for i in range(1,5):
    rospy.loginfo('Detecting plug on base from position %i'%i)

    # move to joint space position
    rospy.loginfo("Move in joint space...")
    if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/detect_plug_on_base%i'%i), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Move retract in joint space failed')
      server.set_aborted()
      return

    # call vision plug detection
    rospy.loginfo("Detecting plug...")
    detect_plug_goal.prior.header.stamp = rospy.Time.now()
    if detect_plug_client.send_goal_and_wait(detect_plug_goal, rospy.Duration(5.0), preempt_timeout) == GoalStatus.SUCCEEDED:
      to_init_position()
      server.set_succeeded(DetectPlugOnBaseResult(detect_plug_client.get_result().plug_pose))      
      rospy.loginfo("Action server goal finished")  
      return

  # Failure
  rospy.logerr("Failed to detect plug on base")      
  to_init_position()
  server.set_aborted()




if __name__ == '__main__':
  #Initialize the node
  name = 'detect_plug_on_base'
  rospy.init_node(name)

  joint_space_client = actionlib.SimpleActionClient('r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction)
  joint_space_client.wait_for_server()
  joint_space_goal = JointTrajectoryGoal()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  detect_plug_client = actionlib.SimpleActionClient('vision_plug_detection', VisionPlugDetectionAction)
  detect_plug_client.wait_for_server()
  detect_plug_goal = VisionPlugDetectionGoal()
  detect_plug_goal.camera_name = "/forearm_camera_r"
  detect_plug_goal.prior = PoseStampedMath().fromEuler(0.075, 0.03, 0.24, pi/2, 0, pi/2).msg
  detect_plug_goal.prior.header.frame_id = "base_link"
  detect_plug_goal.origin_on_right = True

  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, DetectPlugOnBaseAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
