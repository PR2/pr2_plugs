#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
import math
import tf

from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *
from sensor_msgs.msg import *

from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from joint_trajectory_action_tools.tools import *

# State machine classes
from smach import *
from executive_python_common.tf_util import TFUtil

import actionlib

def drange(start, stop, step):
  r = start
  while r < stop:
    yield r
    r += step

def get_plugin_goal(ud,goal):
    goal = PluginGoal()
    goal.gripper_to_plug = ud.sm_result.gripper_to_plug
    goal.base_to_outlet = ud.sm_goal.base_to_outlet
    return goal

def get_wiggle_goal(ud,goal):
    goal = WigglePlugGoal()
    goal.gripper_to_plug = ud.sm_result.gripper_to_plug
    goal.gripper_to_plug.header.stamp = rospy.Time.now()

    goal.base_to_outlet = ud.sm_goal.base_to_outlet
    goal.base_to_outlet.header.stamp = rospy.Time.now()

    goal.wiggle_period = rospy.Duration(0.5)
    goal.insert = 1
    return goal

def get_outlet_to_plug(pose_base_outlet, pose_plug_gripper):
    """Get the pose from the outlet to the plug."""
    time = rospy.Time.now()

    TFUtil.transformer.waitForTransform("base_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
    pose_base_gripper = PoseStampedMath().fromTf(TFUtil.transformer.lookupTransform("base_link","r_gripper_tool_frame", time))

    pose_outlet_base = PoseStampedMath(pose_base_outlet).inverse()
    pose_gripper_plug = PoseStampedMath(pose_plug_gripper).inverse()
    outlet_to_plug = (pose_outlet_base*pose_base_gripper*pose_gripper_plug).msg

    return outlet_to_plug

def get_outlet_to_plug_ik_goal(ud, pose):
    """Get an IK goal for a pose in the frame of the outlet"""
    pose_gripper_wrist = PoseStampedMath().fromTf(TFUtil.transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", time))
    pose_base_outlet = PoseStampedMath(ud.base_to_outlet)
    pose_plug_gripper = PoseStampedMath(ud.gripper_to_plug).inverse()

    goal = PR2ArmIKGoal()
    goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
    goal.pose = (pose_base_outlet * pose * pose_plug_gripper * pose_gripper_wrist).msg
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.header.frame_id = 'base_link'

    return goal

def construct_sm():
    TFUtil()

    # Construct state machine
    sm = StateMachine(['succeeded','aborted','preempted'])

    # Define nominal sequence
    with sm:
        Container.map_parent_ud_keys(['base_to_outlet','base_to_plug','gripper_to_plug'])

        # Detect the plug in the gripper
        def store_detect_plug_result(ud, result_state, result):
            ud.gripper_to_plug = TFUtil.wait_and_transform('r_gripper_tool_frame',result.plug_pose)

        StateMachine.add('DETECT_PLUG_IN_GRIPPER',
                SimpleActionState('detect_plug',
                    DetectPlugInGripperAction,
                    goal = DetectPlugInGripperGoal(),
                    result_cb = store_detect_plug_result),
                {'succeeded':'LOWER_SPINE'})

        StateMachine.add('LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.01)),
                {'succeeded':'APPROACH_OUTLET'})

        # Approach plug
        approach_it = Iterator(['succeeded','preempted','aborted'], drange(-0.07, 0.09, 0.005),'approach_offset')
        with approach_it:
            Container.map_parent_ud_keys([
                'base_to_outlet',
                'base_to_plug',
                'outlet_to_plug_contact'])
            approach_sm = StateMachine(['succeeded','preempted','aborted','keep_moving'])
            with approach_sm:
                Container.map_parent_ud_keys([
                    'base_to_outlet',
                    'base_to_plug',
                    'outlet_to_plug_contact',
                    'approach_offset'])
                # Move closer
                def get_move_closer_goal(ud, goal):
                    """Generate an ik goal to move along the local x axis of the outlet."""
                    approach_pose = PoseStampedMath().fromEuler(ud.approach_offset, 0, 0, 0, 0, 0)

                    goal = get_outlet_to_plug_ik_goal(ud, approach_pose)
                    goal.move_duration = rospy.Duration(0.5)
                    return goal

                StateMachine.add('MOVE_CLOSER',
                        SimpleActionState('r_arm_ik', PR2ArmIKAction, goal_cb = get_move_closer_goal),
                        {'succeeded':'CHECK_FOR_CONTACT','aborted':'CHECK_FOR_CONTACT'})

                def plug_in_contact(ud):
                    """Returns true if the plug is in contact with something."""
                    outlet_to_plug = get_outlet_to_plug(ud.base_to_outlet, ud.gripper_to_plug)
                    in_contact = math.fabs(outlet_to_plug.pose.position.x - ud.approach_offset) > 0.002

                    # Store the offset from the plug where we actually made contact
                    if in_contact:
                        ud.outlet_to_plug_contact = outlet_to_plug
                        return True

                    return False

                StateMachine.add('CHECK_FOR_CONTACT',
                    ConditionState(cond_cb = plug_in_contact),
                    {'true':'succeeded','false':'keep_moving'})

            Iterator.set_contained_state('APPROACH',approach_sm,
                loop_outcomes=['keep_moving'])

        StateMachine.add('APPROACH_OUTLET',approach_it, {'succeeded':'TWIST_PLUG'})

        # Twist the plug to check if it's in the outlet
        twist_it = Iterator(['succeeded','preempted','aborted'], drange(0.0, 0.25, 0.025),'twist_angle')
        with twist_it:
            Container.map_parent_ud_keys([
                'base_to_outlet',
                'base_to_plug',
                'outlet_to_plug_contact'])
            twist_sm = StateMachine(['succeeded','preempted','aborted','keep_moving'])
            with twist_sm:
                Container.map_parent_ud_keys([
                    'base_to_outlet',
                    'base_to_plug',
                    'outlet_to_plug_contact',
                    'twist_angle'])
                def get_twist_goal(ud, goal):
                    """Generate an ik goal to rotate the plug"""
                    twist_pose = PoseStampedMath(ud.outlet_to_plug_contact) * PoseStampedMath().fromEuler(0, 0, 0, ud.twist_angle, 0, 0)

                    goal = get_outlet_to_plug_ik_goal(ud, twist_pose)
                    goal.move_duration = rospy.Duration(0.5)
                    return goal

                StateMachine.add('TWIST_PLUG',
                        SimpleActionState('r_arm_ik', PR2ArmIKAction, goal_cb = get_twist_goal),
                        {'succeeded':'CHECK_PLUG_IN_SOCKET','aborted':'CHECK_PLUG_IN_SOCKET'})

                # Check for mate
                def plug_in_socket(ud):
                    """Determine if the plug is in the socket yet"""

                    MIN_EFFORT = 1.0
                    roll_effort = None

                    # Local cb
                    def joint_states_cb(self, msg):
                        roll_effort = dict(zip(msg.name, msg.effort))['r_wrist_roll_joint']

                    # Subscribe to joint state messages
                    joint_sub = rospy.Subscriber("joint_states", JointState, joint_states_cb)

                    # Wait for effort
                    while roll_effort is None:
                        rospy.sleep(0.05)

                    joint_sub.unregister()
                    
                    if roll_effort > MIN_EFFORT:
                        return True
                    
                    return False

                StateMachine.add('CHECK_PLUG_IN_SOCKET',
                    ConditionState(cond_cb = plug_in_socket),
                    {'true':'succeeded','false':'keep_moving'})

            Iterator.set_contained_state('TWIST',
                    twist_sm,
                    loop_outcomes=['keep_moving'])

        StateMachine.add('TWIST_PLUG',twist_it, {'succeeded':'WIGGLE_IN'})

        # Wiggle the plug
        StateMachine.add('WIGGLE_IN',
                SimpleActionState('wiggle_plug',
                    WigglePlugAction,
                    goal_cb = get_wiggle_goal),
                {'succeeded':'succeeded'})
    return sm

