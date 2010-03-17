#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib
import tf
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
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
    # make sure spine is down
    rospy.loginfo("Make sure spine is down...")
    spine_goal.position = 0.01
    if spine_client.send_goal_and_wait(spine_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Moving down spine failed')
      server.set_aborted()
      return

    # align base
    rospy.loginfo("Rough base alignment...")
    align_base_goal.offset = 0
    if align_base_client.send_goal_and_wait(align_base_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Aligning base failed')
      server.set_aborted()
      return

    # move to joint space position
    rospy.loginfo("Move in joint space to view the outlet...")
    if joint_space_client.send_goal_and_wait(get_action_goal('pr2_plugs_configuration/detect_outlet'), rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Move in joint space to view the outlet failed.')
      raise ActionException(joint_space_client)
    rospy.sleep(2.0)

    found_outlet = False
    offset = [0.0, 0.1, -0.2, 0.3, -0.4]
    it = 0
    while not found_outlet and it<len(offset):
      # align base
      rospy.loginfo("Search base alignment...")
      align_base_goal.offset = offset[it]
      if align_base_client.send_goal_and_wait(align_base_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Aligning base failed')
        server.set_aborted()
        return
      it = it+1

      # call vision outlet detection
      rospy.loginfo("Detecting outlet with the forearm camera...")
      vision_detect_outlet_goal.wall_normal = align_base_client.get_result().wall_norm
      vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
      if vision_detect_outlet_client.send_goal_and_wait(vision_detect_outlet_goal, rospy.Duration(5.0), preempt_timeout) == GoalStatus.SUCCEEDED:
        found_outlet = True

    # check if outlet was found
    if found_outlet:
      # align base precisely
      rospy.loginfo("Precise base alignment...")
      outlet_rough = PoseStamped()
      outlet_rough = wait_and_transform("r_forearm_cam_optical_frame", vision_detect_outlet_client.get_result().outlet_pose)
      align_base_goal.offset = outlet_rough.pose.position.y
      if align_base_client.send_goal_and_wait(align_base_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Aligning base failed')
        server.set_aborted()
        return

      # detect wall
      if wall_norm_client.send_goal_and_wait(wall_norm_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Aligning base failed')
        server.set_aborted()
        return

      # call vision outlet detection
      rospy.loginfo("Detecting outlet with the forearm camera...")
      vision_detect_outlet_goal.wall_normal = wall_norm_client.get_result().wall_norm
      vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
      if vision_detect_outlet_client.send_goal_and_wait(vision_detect_outlet_goal, rospy.Duration(15.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Precise outlet detection failed')
        server.set_aborted()
        return
    else:
      rospy.logerr('Failed to detect outlet')
      server.set_aborted()
      return

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
  look_point = PointStamped()
  look_point.header.frame_id = 'base_link'
  look_point.point.x = -0.14
  look_point.point.y = -0.82
  look_point.point.z = 0.5
  wall_norm_goal = DetectWallNormGoal()
  wall_norm_goal.look_point = look_point
  align_base_goal = AlignBaseGoal()
  align_base_goal.look_point = look_point

  # Declare the goal for the detector
  vision_detect_outlet_goal = VisionOutletDetectionGoal()
  vision_detect_outlet_goal.camera_name = "/forearm_camera_r"
  vision_detect_outlet_goal.prior = PoseStampedMath().fromEuler(-0.14, -0.82, 0.29, 0, 0, -pi/2).msg
  vision_detect_outlet_goal.prior.header.frame_id = "base_link"
      
  # Construct tf listener
  transformer = tf.TransformListener(True, rospy.Duration(60.0))  

  # create action clients we use
  joint_space_client = actionlib.SimpleActionClient('r_arm_plugs_controller/joint_trajectory_generator', JointTrajectoryAction)
  joint_space_client.wait_for_server()

  spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
  spine_client.wait_for_server()
  spine_goal = SingleJointPositionGoal()

  wall_norm_client = actionlib.SimpleActionClient('detect_wall_norm', DetectWallNormAction)
  wall_norm_client.wait_for_server()

  vision_detect_outlet_client = actionlib.SimpleActionClient('vision_outlet_detection', VisionOutletDetectionAction)
  vision_detect_outlet_client.wait_for_server()


  move_base_omnidirectional_client = actionlib.SimpleActionClient('move_base_omnidirectional', MoveBaseAction)
  move_base_omnidirectional_client.wait_for_server()
  move_base_goal = MoveBaseGoal()

  align_base_client = actionlib.SimpleActionClient('align_base', AlignBaseAction)
  align_base_client.wait_for_server()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, DetectOutletAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
