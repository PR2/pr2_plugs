#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
from math import *
import copy
import tf

from std_srvs.srv import *

from actionlib_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from geometry_msgs.msg import *
from joint_trajectory_action_tools.tools import *

# State machine classes
from smach import *

import actionlib

class TFUtil():
    transformer = None
    def __init__(self):
        if not TFUtil.transformer:
            TFUtil.transformer = tf.TransformListener(True, rospy.Duration(60.0))    
        
    @staticmethod
    def wait_and_transform(frame_id,pose):
        try:
            TFUtil.transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
        except rospy.ServiceException, ex:
            rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
            raise ex
        return TFUtil.transformer.transformPose(frame_id, pose)

# Code block state for grasping the plug
class GraspPlugState(SPAState):
    def enter(self):
        cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
        cart_space_client.wait_for_server()
        cart_space_goal = PR2ArmIKGoal()

        preempt_timeout = rospy.Duration(5.0)
        # Grab relevant user data
        pose_tf_plug = self.userdata.sm_result.plug_on_base_pose

        # Get grasp plug IK seed
        cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/grasp_plug_seed')

        # Define the desired grasp on te plug
        pose_plug_gripper = PoseStampedMath()
        pose_plug_gripper.fromEuler(-.02, 0, .01, pi/2, 0, -pi/9)

        pose_base_plug= PoseStampedMath(TFUtil.transformer.transformPose("base_link", pose_tf_plug))
        pose_gripper_wrist= PoseStampedMath().fromTf(TFUtil.transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", rospy.Time(0)))
        pose_plug_approach = PoseStampedMath().fromEuler(0, 0.05, 0, 0, 0, 0)

        cart_space_goal.pose = (pose_base_plug * pose_plug_approach * pose_plug_gripper * pose_gripper_wrist).msg
        cart_space_goal.pose.header.stamp = rospy.Time.now()
        cart_space_goal.pose.header.frame_id = 'base_link'
        cart_space_goal.move_duration = rospy.Duration(3.0)
        if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
            rospy.logerr('Failed to approach plug')
            return 'aborted'

        cart_space_goal.pose = (pose_base_plug * pose_plug_gripper * pose_gripper_wrist).msg
        cart_space_goal.pose.header.stamp = rospy.Time.now()
        cart_space_goal.pose.header.frame_id = 'base_link'
        cart_space_goal.move_duration = rospy.Duration(3.0)
        if cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
            rospy.logerr('Failed to grasp plug')
            return 'aborted'

        return 'succeeded'

# Callback to store the plug detection result
def store_detect_plug_result(ud, result_state, result):
    if result_state == actionlib.GoalStatus.SUCCEEDED:
        ud.sm_result.plug_on_base_pose = TFUtil.wait_and_transform('base_link',result.plug_pose) 

def main():
    rospy.init_node("fetch_plug")#,log_level=rospy.DEBUG)

    TFUtil()
    # Define fixed goals

    # Open gripper goal
    open_gripper_goal = Pr2GripperCommandGoal()
    open_gripper_goal.command.position = 0.07
    open_gripper_goal.command.max_effort = 99999

    # Close gripper goal
    close_gripper_goal = Pr2GripperCommandGoal()
    close_gripper_goal.command.position = 0.0
    close_gripper_goal.command.max_effort = 99999

    # Construct state machine
    sm = StateMachine(['succeeded','aborted','preempted'])

    # Default userdata
    sm.local_userdata.plug_pose = PoseStamped()

    # Define nominal sequence
    with sm:
        StateMachine.add('RAISE_SPINE',
                SimpleActionState('torso_controller/position_joint_action',
                    SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.16)),
                {'succeeded':'MOVE_ARM_BASE_DETECT_POSE'})

        # Move arm to detect the plug on the base
        StateMachine.add('MOVE_ARM_BASE_DETECT_POSE',
                JointTrajectoryState('r_arm_plugs_controller',
                    'pr2_plugs_configuration/detect_plug_on_base'),
                {'succeeded':'DETECT_PLUG_ON_BASE'})

        # Detect the plug
        StateMachine.add('DETECT_PLUG_ON_BASE',
                SimpleActionState('detect_plug_on_base',DetectPlugOnBaseAction,
                    goal = DetectPlugOnBaseGoal(),
                    result_cb = store_detect_plug_result),
                {'succeeded':'MOVE_ARM_BASE_GRASP_POSE',
                    'aborted':'MOVE_ARM_BASE_DETECT_POSE'})

        # Move arm to the grasp pose
        StateMachine.add('MOVE_ARM_BASE_GRASP_POSE',
                JointTrajectoryState('r_arm_plugs_controller',
                    'pr2_plugs_configuration/grasp_plug'),
                {'succeeded':'OPEN_GRIPPER',
                    'aborted':'RECOVER_GRASP_TO_DETECT_POSE'})

        StateMachine.add('OPEN_GRIPPER',
                SimpleActionState('r_gripper_controller/gripper_action',
                    Pr2GripperCommandAction,
                    goal = open_gripper_goal),
                {'succeeded':'GRASP_PLUG'})

        StateMachine.add('GRASP_PLUG',
                GraspPlugState(),
                {'succeeded':'CLOSE_GRIPPER',
                    'aborted':'DETECT_PLUG_ON_BASE'})
        
        StateMachine.add('CLOSE_GRIPPER',
                SimpleActionState('r_gripper_controller/gripper_action',
                    Pr2GripperCommandAction,
                    goal = close_gripper_goal),
                { 'succeeded':'DETECT_PLUG_ON_BASE',
                    'aborted':'REMOVE_PLUG'})

        StateMachine.add('REMOVE_PLUG',
                JointTrajectoryState('r_arm_plugs_controller',
                    'pr2_plugs_configuration/remove_plug'),
                {'succeeded':'LOWER_SPINE'})
            
        StateMachine.add('LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action',
                    SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.01)),
                {'succeeded':'succeeded'})
        
        # Define recovery states
        StateMachine.add('RECOVER_GRASP_TO_DETECT_POSE',
                JointTrajectoryState('r_arm_plugs_controller',
                    'pr2_plugs_configuration/recover_grasp_to_detect'),
                { 'succeeded':'DETECT_PLUG_ON_BASE',
                    'aborted':'RECOVER_GRASP_TO_DETECT_POSE'})

    # Run state machine introspection server
    intro_server = smach.IntrospectionServer('fetch_plug',sm,'/RECHARGE/FETCH_PLUG')
    intro_server.start()

    # Run state machine action server 
    sms = ActionServerWrapper(
            'fetch_plug', FetchPlugAction, sm,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted']
            )
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()

