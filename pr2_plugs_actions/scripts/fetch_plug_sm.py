
import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
import tf

from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from std_srvs.srv import *
from executive_python import *

# State machine classes
from smach.state import *
from smach.state_machine import *
from smach.action_state import *

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
def store_detect_plug_result(state, result_state, result):
  tf_util = TFUtil()
  state.sm_userdata.plug_on_base_pose = tf_util.wait_and_transform('base_link',result.plug_pose) 

# Code block state for grasping the plug
class GraspPlugState(State):
  def enter(self):
    cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
    cart_space_client.wait_for_server()
    cart_space_goal = PR2ArmIKGoal()

    # Grab relevant user data
    pose_base_plug = self.sm_userdata.plug_on_base_pose

    # Get grasp plug IK seed
    cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/grasp_plug_seed')

    # Define the desired grasp on te plug
    pose_plug_gripper = PoseStampedMath()
    pose_plug_gripper.fromEuler(-.03, 0, .01, pi/2, 0, -pi/9)

    pose_gripper_wrist= PoseStampedMath().fromTf(transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", rospy.Time(0)))
    pose_plug_approach = PoseStampedMath().fromEuler(0, 0.05, 0, 0, 0, 0)

    cart_space_goal.pose = (pose_base_plug * pose_plug_approach * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(3.0)
    if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Failed to approach plug')
      self.set_aborted()
      return

    cart_space_goal.pose = (pose_base_plug * pose_plug_gripper * pose_gripper_wrist).msg
    cart_space_goal.pose.header.stamp = rospy.Time.now()
    cart_space_goal.pose.header.frame_id = 'base_link'
    cart_space_goal.move_duration = rospy.Duration(3.0)
    if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
      rospy.logerr('Failed to grasp plug')
      self.set_aborted()
      return

    self.set_succeeded()

# Callback to stuff the result for this action
def construct_action_result(sm):
  self.result.plug_pose = sm.userdata.plug_pose

def main():
  rospy.init_node("fetch_plug_sm")

  # Define fixed goals
  # Plug detection goal
  detect_plug_goal = VisionPlugDetectionGoal()
  detect_plug_goal.camera_name = "/forearm_camera_r"
  detect_plug_goal.prior = PoseStampedMath().fromEuler(0.075, 0.03, 0.24, pi/2, 0, pi/2).msg
  detect_plug_goal.prior.header.stamp = rospy.Time.now()
  detect_plug_goal.prior.header.frame_id = "base_link"
  detect_plug_goal.origin_on_right = False

  # Open gripper goal
  open_gripper_goal = Pr2GripperCommandAction()
  open_gripper_goal.command.position = 0.07
  open_gripper_goal.command.max_effort = 99999

  # Close gripper goal
  close_gripper_goal = Pr2GripperCommandAction()
  close_gripper_goal.command.position = 0.0
  close_gripper_goal.command.max_effort = 99999

  # Construct state machine
  sm = StateMachine('fetch_plug',FetchPlugSMAction,FetchPlugSMResult(),result_cb = construct_action_result)
  # Define nominal sequence
  sm.add_sequence(
      # Raise spine
      SimpleActionState('raise_spine',
        'torso_controller/position_joint_action', SingleJointPositionAction,
        goal = SingleJointPositionGoal(position=0.16)),

      # Move arm to detect the plug on the base
      SimpleActionState('move_arm_base_detect_pose',
        'r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction,
        goal = get_action_goal('pr2_plugs_configuration/detect_plug_on_base')),

      # Detect the plug
      SimpleActionState('detect_plug_on_base',
        'vision_plug_detection',VisionPlugDetectionAction,
        goal = detect_plug_goal,
        aborted = 'move_arm_base_detect_pose',
        result_cb = store_detect_plug_result),

      # Move arm to the grasp pose
      SimpleActionState('move_arm_base_grasp_pose',
        'r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction,
        goal = get_action_goal('pr2_plugs_configuration/detect_plug_on_base'),
        aborted = 'recover_grasp_to_detect_pose'),

      # Open the gripper
      SimpleActionState('open_gripper',
        'r_gripper_controller/gripper_action', Pr2GripperCommandAction,
        goal = open_gripper_goal)

      # Grasp the plug
      GraspPlugState('grasp_plug',aborted = 'recover_grasp_to_detect_pose'),

      # Close the gripper
      SimpleActionState('close_gripper',
        'r_gripper_controller/gripper_action', Pr2GripperCommandAction,
        goal = close_gripper_goal,
        aborted='recover_grasp_to_detect_pose'),

      # Remove the plug form the base
      SimpleActionState('move_arm_remove_plug',
        'r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction,
        goal = get_action_goal('pr2_plugs_configuration/remove_plug')),
      
      # Lower the spine
      SimpleActionState('raise_spine',
        'torso_controller/position_joint_action', SingleJointPositionAction,
        goal = SingleJointPositionGoal(position=0.01))
      )

  # Define recovery states
  sm.add(SimpleActionState('recover_grasp_to_detect_pose',
    'r_arm_plugs_controller/joint_trajectory_action', JointTrajectoryAction,
    goal = get_action_goal('pr2_plugs_configuration/move_arm_grasp_to_detect_pose'),
    succeeded = 'detect_plug_on_base'))

  # Populate the sm database with some stubbed out results
  # Run state machine action server with default state
  sm.run_server('raise_spine')


if __name__ == "__main__":
  main()

