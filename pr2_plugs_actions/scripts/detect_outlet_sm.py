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

import dynamic_reconfigure.client
from executive_python import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from joint_trajectory_action_tools.tools import *

# State machine classes
from smach.state import *
from smach.state_machine import *
from smach.simple_action_state import *
from smach.joint_trajectory_state import *

import actionlib
import dynamic_reconfigure.client

class TFUtil():
  transformer = None
    
  @staticmethod
  def wait_and_transform(frame_id,pose):
    if not TFUtil.transformer:
      TFUtil.transformer = tf.TransformListener(True, rospy.Duration(60.0))  

    try:
      TFUtil.transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
    except rospy.ServiceException, e:
      rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
      raise e
    return TFUtil.transformer.transformPose(frame_id, pose)

# Code block state for grasping the plug
class OutletSearchState(State):
  def __init__(self, label,
      offsets,
      succeeded="SUCCEEDED",
      aborted="ABORTED",
      preempted="PREEMPTED"):
    State.__init__(self,label,succeeded,aborted,preempted)
    # Store goals
    self.offsets = offsets

    # Create action clients
    self.align_base_client = actionlib.SimpleActionClient('align_base', AlignBaseAction)
    self.align_base_client.wait_for_server()
    self.vision_detect_outlet_client = actionlib.SimpleActionClient('vision_outlet_detection', VisionOutletDetectionAction)
    self.vision_detect_outlet_client.wait_for_server()

  def enter(self):
    """Iterate across a set of offsets to find the outlet"""
    preempt_timeout = rospy.Duration(10.0)

    align_base_goal = self.sm_userdata.align_base_goal
    vision_detect_outlet_goal = self.sm_userdata.vision_detect_outlet_goal

    # Iterate across move_base offsets
    for offset in self.offsets:
      # align base
      rospy.loginfo("Search base alignment...")
      align_base_goal.offset = offset
      if self.align_base_client.send_goal_and_wait(align_base_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Aligning base failed')
        self.set_aborted()
        return

      # call vision outlet detection
      rospy.loginfo("Detecting outlet with the forearm camera...")
      vision_detect_outlet_goal.wall_normal = self.align_base_client.get_result().wall_norm
      vision_detect_outlet_goal.wall_normal.header.stamp = rospy.Time.now()
      vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
      if self.vision_detect_outlet_client.send_goal_and_wait(vision_detect_outlet_goal, rospy.Duration(5.0), preempt_timeout) == GoalStatus.SUCCEEDED:
        # Store the rough outlet position in the state machiine user data structure
        self.sm_userdata.outlet_rough_pose = TFUtil.wait_and_transform(
            'r_forearm_cam_optical_frame', self.vision_detect_outlet_client.get_result().outlet_pose)
        # Set succeeded, and return
        self.set_succeeded()
        return

    rospy.logerr("Could not find outlet in search")
    self.set_aborted()

# Callback to store the plug detection result
def get_precise_align_goal(state):
  state.goal.offset = state.sm_userdata.outlet_rough_pose.pose.position.y
  return state.goal

# Callbacks for wall norm
def get_wall_norm_goal(state):
  state.sm_userdata.wall_norm_goal.look_point.header.stamp = rospy.Time.now()
  return state.sm_userdata.wall_norm_goal

def store_wall_norm_result(state, result_state, result):
  state.sm_userdata.vision_detect_outlet_goal.wall_normal = result.wall_norm

# Callback for resetting timestamp in vision detection goal
def get_vision_detect_goal(state):
  state.sm_userdata.vision_detect_outlet_goal.wall_normal.header.stamp = rospy.Time.now()
  state.sm_userdata.vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
  return state.sm_userdata.vision_detect_outlet_goal

# Callback for storing the prcise detection result
def store_precise_outlet_result(state, result_state, result):
  state.sm_userdata.sm_result.outlet_pose = TFUtil.wait_and_transform("base_link",result.outlet_pose)

def main():
  rospy.init_node("detect_outlet_sm")#,log_level=rospy.DEBUG)

  #check to see if this is running in sim where the dynamic reconfigure doesn't exist
  sim = rospy.get_param('~sim', False)
  if(not sim):
    #this ensures that the forearm camera triggers when the texture projector is off
    projector_client = dynamic_reconfigure.client.Client('camera_synchronizer_node')
    forearm_projector_off = {'forearm_r_trig_mode': 4} #off
    projector_client.update_configuration(forearm_projector_off)


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

  vision_detect_outlet_goal = VisionOutletDetectionGoal()
  vision_detect_outlet_goal.camera_name = "/forearm_camera_r"
  vision_detect_outlet_goal.prior = PoseStampedMath().fromEuler(-0.14, -0.82, 0.29, 0, 0, -pi/2).msg
  vision_detect_outlet_goal.prior.header.frame_id = "base_link"

  # Declare the goal for the detector
  # Construct state machine
  sm = StateMachine('detect_outlet_sm',DetectOutletSMAction)

  # Default userdata
  sm.userdata.wall_norm_goal = wall_norm_goal
  sm.userdata.align_base_goal = align_base_goal
  sm.userdata.vision_detect_outlet_goal = vision_detect_outlet_goal
  sm.userdata.outlet_rough_pose = PoseStamped()

  # Define nominal sequence
  sm.add_sequence(
      # Lower spine
      SimpleActionState('lower_spine',
        'torso_controller/position_joint_action', SingleJointPositionAction,
        goal = SingleJointPositionGoal(position=0.01)),

      # Perform rough base alignment
      SimpleActionState('rough_align_base',
        'align_base', AlignBaseAction,
        goal = AlignBaseGoal(offset = 0,look_point=look_point)),

      # Move arm so that it can view the outlet
      JointTrajectoryState('move_arm_detect_outlet',
        'r_arm_plugs_controller','pr2_plugs_configuration/detect_outlet',
        aborted = 'recover_move_arm_outlet_to_free'),

      # Search side-to-side for the outlet
      OutletSearchState('outlet_search',
        offsets = (0.0, 0.1, -0.2, 0.3, -0.4),
        aborted = 'recover_move_arm_outlet_to_free'),

      # Align precisely
      SimpleActionState('precise_align_base',
        'align_base', AlignBaseAction,
        goal = AlignBaseGoal(offset = 0,look_point=look_point),
        goal_cb = get_precise_align_goal,
        aborted = 'recover_move_arm_outlet_to_free'),

      # Detect the wall norm
      SimpleActionState('detect_wall_norm',
        'detect_wall_norm', DetectWallNormAction,
        goal_cb = get_wall_norm_goal,
        result_cb = store_wall_norm_result,
        aborted = 'recover_move_arm_outlet_to_free'),
      
      # Precise detection
      SimpleActionState('vision_outlet_detection',
        'vision_outlet_detection', VisionOutletDetectionAction,
        goal_cb = get_vision_detect_goal,
        result_cb = store_precise_outlet_result,
        aborted = 'recover_move_arm_outlet_to_free')
      )

  # Define recovery states
  sm.add(JointTrajectoryState('recover_move_arm_outlet_to_free',
    'r_arm_plugs_controller','pr2_plugs_configuration/recover_outlet_to_free',
    succeeded = 'rough_align_base'))

  # Populate the sm database with some stubbed out results
  # Run state machine action server with default state
  sm.run_server('lower_spine')
  rospy.spin()


if __name__ == "__main__":
  main()

