#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from joint_trajectory_action_tools.tools import *
from pr2_arm_move_ik.tools import *
from actionlib_msgs.msg import *
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
import math
import PyKDL
from tf_conversions.posemath import fromMsg, toMsg
from pr2_plugs_actions.tf_util import TFUtil
from pr2_image_snapshot_recorder.msg import ImageSnapshotAction, ImageSnapshotGoal


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

    if goal.record_image:
      rospy.loginfo("Recording images of plug on base")
      image_client = actionlib.SimpleActionClient('image_snapshot', ImageSnapshotAction)
      if not image_client.wait_for_server(rospy.Duration(20.0)):
        rospy.logerr("Imagesnapshot server is down.")
      else:
        image_goal = ImageSnapshotGoal()
        image_goal.topic_name = 'r_forearm_cam/image_raw'
        image_goal.num_images = 5
        image_goal.output_file_name = '/removable/continuous_operation/stow_plug_failure_images_%s.bag'%str(rospy.Time.now())
        image_client.send_goal_and_wait(image_goal, rospy.Duration(20.0), preempt_timeout)

    # call vision plug detection
    rospy.loginfo("Detecting plug...")
    detect_plug_goal = VisionPlugDetectionGoal()
    detect_plug_goal.camera_name = "/r_forearm_cam"
    detect_plug_goal.prior.pose = toMsg(PyKDL.Frame(PyKDL.Rotation.RPY(pi/2, 0, pi/2), PyKDL.Vector(0.080, 0.026, 0.23)))
    detect_plug_goal.prior.header.frame_id = "base_link"
    detect_plug_goal.origin_on_right = False
    detect_plug_goal.prior.header.stamp = rospy.Time.now()
    if detect_plug_client.send_goal_and_wait(detect_plug_goal, rospy.Duration(5.0), preempt_timeout) == GoalStatus.SUCCEEDED:
      pose_detect_plug = detect_plug_client.get_result().plug_pose
      try:
        pose_base_plug = fromMsg(TFUtil.wait_and_transform('base_link', pose_detect_plug).pose)
      except rospy.ServiceException, e:
        rospy.logerr('Could not transform between base_link and %s' %pose_detect_plug.header.frame_id)
        server.set_aborted()
        return
      error = (pose_base_plug.Inverse() * fromMsg(detect_plug_goal.prior.pose))
      (r, p, y) = error.M.GetRPY()
      if (math.fabs(r) < 0.8) and (math.fabs(p) < 0.8) and (math.fabs(y) < 0.8) and (math.fabs(error.p[0]) < 0.05) and (math.fabs(error.p[1]) < 0.05) and (math.fabs(error.p[2]) < 0.05):
        to_init_position()
        server.set_succeeded(DetectPlugOnBaseResult(detect_plug_client.get_result().plug_pose))      
        rospy.loginfo("Action server goal finished")  
        return
      else:
        rospy.loginfo('Found the plug, but it is in a different location than I expected.')

  # Failure
  rospy.logerr("Failed to detect plug on base")      
  to_init_position()
  server.set_aborted()




if __name__ == '__main__':
  #Initialize the node
  name = 'detect_plug_on_base'
  rospy.init_node(name)

  # transform listener
  TFUtil()

  joint_space_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_generator', JointTrajectoryAction)
  joint_space_client.wait_for_server()
  joint_space_goal = JointTrajectoryGoal()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  detect_plug_client = actionlib.SimpleActionClient('vision_plug_detection', VisionPlugDetectionAction)
  detect_plug_client.wait_for_server()

  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, DetectPlugOnBaseAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
