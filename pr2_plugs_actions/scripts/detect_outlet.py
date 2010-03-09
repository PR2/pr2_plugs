#!/usr/bin/env python
# stub for outlet detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib
import tf
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from actionlib_msgs.msg import *
from joint_trajectory_action_tools.tools import *
from trajectory_msgs.msg import JointTrajectoryPoint
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from executive_python import *
from math import pi

def wait_and_transform(frame_id,pose):
  try:
    transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
    raise e
  return transformer.transformPose(frame_id, pose)

def execute_cb(goal):
  rospy.loginfo("Sub-action detect_outlet received goal.")
  preempt_timeout = rospy.Duration(5.0)

  try:
    # get wall norm
    rospy.loginfo("Detecting wall norm...")
    while wall_norm_client.send_goal_and_wait(wall_norm_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Wall norm detection failed, trying again')
      rospy.sleep(2.0)

    # move to joint space position
    rospy.loginfo("Move in joint space to view the outlet...")
    if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/move_arm_to_outlet'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Move in joint space to view the outlet failed.')
      raise ActionException(joint_space_client)
    rospy.sleep(2.0)

    # call vision outlet detection
    rospy.loginfo("Detecting outlet with the forearm camera...")
    vision_detect_outlet_goal.camera_name = "/forearm_camera_r"

    vision_detect_outlet_goal.wall_normal = wall_norm_client.get_result().wall_norm
    vision_detect_outlet_goal.wall_normal.header.stamp = rospy.Time.now()

    vision_detect_outlet_goal.prior = pose_base_outlet.msg
    vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
    vision_detect_outlet_goal.prior.header.frame_id = "base_link"

    if vision_detect_outlet_client.send_goal_and_wait(vision_detect_outlet_goal, rospy.Duration(30.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Detecting the outlet with the forearm camera failed.')
      raise ActionException(vision_detect_outlet_client)

    # Transform outlet pose into fixed frame and return the result
    result = DetectOutletResult()
    result.outlet_pose = wait_and_transform("base_link",vision_detect_outlet_client.get_result().outlet_pose)

    server.set_succeeded(result)
    rospy.loginfo("Successfully detected the outlet.")  

  except (ActionException,rospy.ServiceException):
    # Try to clean up the arm
    rospy.loginfo("Move in joint space from the outlet...")
    if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/move_arm_outlet_to_free'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Move in joint space from the outlet failed.')
      server.set_aborted()
      return
    # Set aborted 
    server.set_aborted()
    rospy.loginfo("Failed during sequence.")  



if __name__ == '__main__':
  #Initialize the node
  name = 'detect_outlet'
  rospy.init_node(name)

  # Declare goal structures and parameters

  # Declare wall norm goal
  # This is the point at which we want the head to look
  wall_norm_goal = DetectWallNormGoal()
  wall_norm_goal.look_point.header.frame_id = 'base_link'
  wall_norm_goal.look_point.point.x = -0.14
  wall_norm_goal.look_point.point.y = -0.82
  wall_norm_goal.look_point.point.z = 0.5

  # Declare the goal for the detector
  vision_detect_outlet_goal = VisionOutletDetectionGoal()

  # Declare the pose of the outlet in the 'base_link' frame
  pose_base_outlet = PoseStampedMath()
  pose_base_outlet.fromEuler(-0.14, -0.82, 0.29, 0, 0, -pi/2)

  # Construct tf listener
  transformer = tf.TransformListener(True, rospy.Duration(60.0))  


  # create action clients we use
  joint_space_client = actionlib.SimpleActionClient('r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction)
  joint_space_client.wait_for_server()
  #  joint_space_goal = JointTrajectoryGoal()

  wall_norm_client = actionlib.SimpleActionClient('detect_wall_norm', DetectWallNormAction)
  wall_norm_client.wait_for_server()

  vision_detect_outlet_client = actionlib.SimpleActionClient('vision_outlet_detection', VisionOutletDetectionAction)
  vision_detect_outlet_client.wait_for_server()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, DetectOutletAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
