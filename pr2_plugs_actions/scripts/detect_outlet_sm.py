#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
from math import *
import tf

from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *

from executive_python import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from joint_trajectory_action_tools.tools import *

# State machine classes
from smach.state import *
from smach.state_machine import *
from smach.simple_action_state import *

import actionlib

class TFUtil():
  def __init__(self):
    # Construct tf listener
    transformer = tf.TransformListener(True, rospy.Duration(60.0))  
    
  def wait_and_transform(frame_id,pose):
    try:
      transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return transformer.transformPose(frame_id, pose)

# Callback to store the plug detection result
def get_precise_align_goal(state):
  self.goal.offset = self.sm_userdata.outlet_rough_pose.y
  return self.goal

# Code block state for grasping the plug
class OutletSearchState(State):
  def __init__(self, label,
      align_goal, vision_detect_outlet_goal, offsets,
      succeeded="SUCCEEDED",
      aborted="ABORTED",
      preempted="PREEMPTED"):
    # Store goals
    self.algin_goal = align_goal
    self.vision_detect_outlet_goal = vision_detect_outlet_goal
    self.offsets = offsets

    # Create action clients
    self.align_base_client = actionlib.SimpleActionClient('align_base', AlignBaseAction)
    self.align_base_client.wait_for_server()
    self.vision_detect_outlet_client = actionlib.SimpleActionClient('vision_outlet_detection', VisionOutletDetectionAction)
    self.vision_detect_outlet_client.wait_for_server()

  def enter(self):
    """Iterate across a set of offsets to find the outlet"""
    tf_util = TFUtil()

    # Iterate across move_base offsets
    for offset in self.offsets:
      # align base
      rospy.loginfo("Search base alignment...")
      self.align_base_goal.offset = offset
      if self.align_base_client.send_goal_and_wait(self.align_base_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Aligning base failed')
        self.set_aborted()
        return

      # call vision outlet detection
      rospy.loginfo("Detecting outlet with the forearm camera...")
      self.vision_detect_outlet_goal.wall_normal = self.align_base_client.get_result().wall_norm
      self.vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
      if self.vision_detect_outlet_client.send_goal_and_wait(self.vision_detect_outlet_goal, rospy.Duration(5.0), preempt_timeout) == GoalStatus.SUCCEEDED:
        # Store the rough outlet position in the state machiine user data structure
        self.sm_userdata.outlet_rough_pose = PoseStamped()
        self.sm_userdata.outlet_rough_pose = tf_util.wait_and_transform(
            'r_forearm_cam_optical_frame', self.vision_detect_outlet_client.get_result().outlet_pose)
        # Set succeeded, and return
        self.set_succeeded()
        return

    rospy.logerr("Could not find outlet in search")
    self.set_aborted()

# Callback for resetting timestamp in vision detection goal
def update_vision_detect_goal_stamp(state):
  self.goal.prior.header.stamp = rospy.Time.now()
  return self.goal

def store_precise_outlet_result(state, result_state, result):
  tf_util = TFUtil()
  state.sm_userdata.outlet_precise_pose = tf_util.wait_and_transform("base_link",result.outlet_pose)


# Callback to stuff the result for this action
def construct_action_result(sm):
  self.result.outlet_pose = sm.userdata.outlet_precise_pose

def main():
  rospy.init_node("detect_outlet_sm")

  # Define fixed goals
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

  # Construct state machine
  sm = StateMachine('detect_outlet_sm',DetectOutletSMAction,result_cb = construct_action_result)
  # Define nominal sequence
  sm.add_sequence(
      # Raise spine
      SimpleActionState('lower_spine',
        'torso_controller/position_joint_action', SingleJointPositionAction,
        goal = SingleJointPositionGoal(position=0.01)),

      # Perform rough base alignment
      SimpleActionState('rough_align_base',
        'align_base', AlignBaseAction,
        goal = AlignBaseGoal(offset = 0)),

      # Move arm so that it can view the outlet
      SimpleActionState('move_arm_detect_outlet',
        'r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction,
        goal = get_action_goal('pr2_plugs_configuration/detect_outlet'),
        aborted = 'recover_move_arm_outlet_to_free'),

      # Search side-to-side for the outlet
      OutletSearchState('outlet_search',
        align_base_goal, vision_detect_outlet_goal,
        offsets = (0.0, 0.1, -0.2, 0.3, -0.4)),

      # Align precisely
      SimpleActionState('precise_align_base',
        'align_base', AlignBaseAction,
        goal = AlignBaseGoal(offset = 0),
        goal_cb = get_precise_align_goal),

      # Detect the wall norm
      SimpleActionState('detect_wall_norm',
        'detect_wall_norm', DetectWallNormAction,
        goal = wall_norm_goal),
      
      # Lower the spine
      SimpleActionState('vision_outlet_detection',
        'vision_detect_outlet', VisionOutletDetectionAction,
        goal = vision_detect_outlet_goal,
        goal_cb = update_vision_detect_goal_stamp,
        result_cb = store_precise_outlet_result)
      )

  # Define recovery states
  sm.add(SimpleActionState('recover_grasp_to_detect_pose',
    'r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction,
    goal = get_action_goal('pr2_plugs_configuration/move_arm_grasp_to_detect_pose'),
    succeeded = 'detect_plug_on_base'))

  # Populate the sm database with some stubbed out results
  # Run state machine action server with default state
  sm.run_server('lower_spine')


if __name__ == "__main__":
  main()

