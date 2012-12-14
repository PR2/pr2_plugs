#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os,sys,time
import math
import PyKDL

from actionlib_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import *
from move_base_msgs.msg import *
from sensor_msgs.msg import *

from pr2_arm_move_ik.tools import *
from tf_conversions.posemath import fromMsg, toMsg
from joint_trajectory_action_tools.tools import get_action_goal as get_generator_goal

# State machine classes
import smach
from smach import *
from smach_ros import *
from pr2_plugs_actions.tf_util import TFUtil

import actionlib

def drange(start, stop, step):
  r = start
  while r < stop:
    yield r
    r += step

def get_outlet_to_plug(pose_base_outlet, pose_plug_gripper):
    """Get the pose from the outlet to the plug."""
    pose_base_gripper = fromMsg(TFUtil.wait_and_lookup("base_link","r_gripper_tool_frame", 
                                                       rospy.Time.now(), rospy.Duration(2.0)).pose)

    outlet_to_plug = pose_base_outlet.Inverse() * pose_base_gripper * pose_plug_gripper.Inverse()

    return outlet_to_plug

@smach.cb_interface(input_keys=['base_to_outlet','gripper_to_plug'])
def get_outlet_to_plug_ik_goal(ud, pose):
    """Get an IK goal for a pose in the frame of the outlet"""
    pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup("r_gripper_tool_frame", "r_wrist_roll_link", 
                                                        rospy.Time.now(), rospy.Duration(2.0)).pose)

    goal = ArmMoveIKGoal()
    goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
    goal.pose.pose = toMsg(fromMsg(ud.base_to_outlet) * pose * fromMsg(ud.gripper_to_plug).Inverse() * pose_gripper_wrist)
    goal.pose.header.stamp = rospy.Time.now()
    goal.pose.header.frame_id = 'base_link'

    return goal

@smach.cb_interface(input_keys=['gripper_to_plug','base_to_outlet'])
def get_wiggle_goal(ud,goal):
    goal = WigglePlugGoal()
    goal.gripper_to_plug = ud.gripper_to_plug
    goal.base_to_outlet = ud.base_to_outlet
    goal.wiggle_period = rospy.Duration(0.5)
    goal.insert = 1
    return goal

def construct_sm():
    TFUtil()

    # Construct state machine
    sm = StateMachine(
            ['succeeded','aborted','preempted'],
            input_keys = ['base_to_outlet'],
            output_keys = ['gripper_to_plug'])

    # Define nominal sequence
    with sm:

        # Detect the plug in the gripper
        @smach.cb_interface(output_keys=['gripper_to_plug'])
        def store_detect_plug_result(ud, result_state, result):
            if result_state == GoalStatus.SUCCEEDED:
                gripper_to_plug = TFUtil.wait_and_transform('r_gripper_tool_frame',result.plug_pose).pose
                ud.gripper_to_plug = gripper_to_plug
                print 'gripper_to_plug'
                print gripper_to_plug

        StateMachine.add('DETECT_PLUG_IN_GRIPPER',
                SimpleActionState('detect_plug',
                    DetectPlugInGripperAction,
                    goal = DetectPlugInGripperGoal(),
                    result_cb = store_detect_plug_result),
                {'succeeded':'LOWER_SPINE'})

        StateMachine.add('LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.01)),
                {'succeeded':'APPROACH_OUTLET_ITER'})

        # Approach outlet
        approach_it = Iterator(
                ['succeeded','preempted','aborted'],
                input_keys = ['base_to_outlet','gripper_to_plug'],
                output_keys = ['outlet_to_plug_contact'],
                it = lambda: drange(-0.07, 0.09, 0.005),
                it_label = 'approach_offset',
                exhausted_outcome = 'aborted')
        StateMachine.add('APPROACH_OUTLET_ITER',approach_it,
                {'succeeded':'TWIST_PLUG_ITER',
                    'aborted':'FAIL_PULL_BACK_FROM_WALL'})
        with approach_it:
            approach_sm = StateMachine(
                    ['succeeded','preempted','aborted','keep_moving'],
                    input_keys=['base_to_outlet','approach_offset','gripper_to_plug'],
                    output_keys=['outlet_to_plug_contact'])
            Iterator.set_contained_state('APPROACH',approach_sm,
                loop_outcomes=['keep_moving'])

            with approach_sm:
                @smach.cb_interface(
                        input_keys=['base_to_outlet','approach_offset','gripper_to_plug'])
                def get_move_closer_goal(ud, goal):
                    """Generate an ik goal to move along the local x axis of the outlet."""

                    pose_outlet_plug = PyKDL.Frame(PyKDL.Vector(ud.approach_offset, 0, 0))
                    pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link').pose)
                    base_to_outlet = fromMsg(ud.base_to_outlet)
                    gripper_to_plug = fromMsg(ud.gripper_to_plug)

                    pose_base_wrist = base_to_outlet * pose_outlet_plug * gripper_to_plug.Inverse() * pose_gripper_wrist

                    goal = ArmMoveIKGoal()
                    goal.pose.pose = toMsg(pose_base_wrist)
                    goal.pose.header.stamp = rospy.Time.now()
                    goal.pose.header.frame_id = 'base_link'
                    goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
                    goal.move_duration = rospy.Duration(1.0)
                    return goal

                StateMachine.add('MOVE_CLOSER',
                        SimpleActionState('r_arm_ik', ArmMoveIKAction, goal_cb = get_move_closer_goal),
                        {'succeeded':'CHECK_FOR_CONTACT','aborted':'CHECK_FOR_CONTACT'})

                @smach.cb_interface(
                        input_keys=['base_to_outlet','approach_offset','gripper_to_plug'],
                        output_keys=['outlet_to_plug_contact'])
                def plug_in_contact(ud):
                    """Returns true if the plug is in contact with something."""

                    pose_base_gripper = fromMsg(TFUtil.wait_and_lookup('base_link', 'r_gripper_tool_frame').pose)
                    pose_outlet_plug = fromMsg(ud.base_to_outlet).Inverse() * pose_base_gripper * fromMsg(ud.gripper_to_plug)

                    # check if difference between desired and measured outlet-plug along x-axis is more than 1 cm
                    ud.outlet_to_plug_contact = toMsg(pose_outlet_plug)
                    if math.fabs(pose_outlet_plug.p[0] - ud.approach_offset) > 0.01:
                        return True
                    return False

                StateMachine.add('CHECK_FOR_CONTACT',
                    ConditionState(cond_cb = plug_in_contact),
                    {'true':'succeeded','false':'keep_moving'})


        # Twist the plug to check if it's in the outlet
        twist_it = Iterator(
                ['succeeded','preempted','aborted'],
                input_keys = ['base_to_outlet','gripper_to_plug','outlet_to_plug_contact'],
                it = lambda: drange(0.0, 0.25, 0.025),
                output_keys = [],
                it_label = 'twist_angle',
                exhausted_outcome = 'aborted')
        with twist_it:
            twist_sm = StateMachine(
                    ['succeeded','preempted','aborted','keep_moving'],
                    input_keys = ['base_to_outlet','gripper_to_plug','twist_angle', 'outlet_to_plug_contact'])
            with twist_sm:
                @smach.cb_interface(
                        input_keys=['base_to_outlet','gripper_to_plug','twist_angle', 'outlet_to_plug_contact'])
                def get_twist_goal(ud, goal):
                    """Generate an ik goal to rotate the plug"""
                    pose_contact_plug = PyKDL.Frame(PyKDL.Rotation.RPY(ud.twist_angle, 0, 0))
                    pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link').pose)

                    pose_base_wrist = fromMsg(ud.base_to_outlet) * fromMsg(ud.outlet_to_plug_contact) * pose_contact_plug * fromMsg(ud.gripper_to_plug).Inverse() * pose_gripper_wrist

                    goal = ArmMoveIKGoal()
                    goal.pose.pose = toMsg(pose_base_wrist)
                    goal.pose.header.stamp = rospy.Time.now()
                    goal.pose.header.frame_id = 'base_link'
                    goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
                    goal.move_duration = rospy.Duration(1.0)
                    return goal

                StateMachine.add('TWIST_PLUG',
                        SimpleActionState('r_arm_ik', ArmMoveIKAction, goal_cb = get_twist_goal),
                        {'succeeded':'CHECK_PLUG_IN_SOCKET','aborted':'CHECK_PLUG_IN_SOCKET'})

                # Check for mate
                @smach.cb_interface(input_keys=['roll_effort'],output_keys=['roll_effort'])
                def plug_in_socket(ud):
                    """Determine if the plug is in the socket yet"""

                    MIN_EFFORT = 1.0
                    ud.roll_effort = None

                    # Local cb
                    def joint_states_cb(msg, ud):
                        ud.roll_effort = dict(zip(msg.name, msg.effort))['r_wrist_roll_joint']

                    # Subscribe to joint state messages
                    joint_sub = rospy.Subscriber("joint_states", JointState, joint_states_cb, ud)

                    # Wait for effort
                    while ud.roll_effort is None:
                        rospy.sleep(0.05)

                    joint_sub.unregister()
                    
                    if ud.roll_effort > MIN_EFFORT:
                        return True
                    
                    return False

                StateMachine.add('CHECK_PLUG_IN_SOCKET',
                    ConditionState(cond_cb = plug_in_socket),
                    {'true':'succeeded','false':'keep_moving'})

            Iterator.set_contained_state('TWIST',
                    twist_sm,
                    loop_outcomes=['keep_moving'])

        StateMachine.add('TWIST_PLUG_ITER',twist_it,
                {'succeeded':'STRAIGHTEN_PLUG',
                    'aborted':'FAIL_PULL_BACK_FROM_WALL'})

        @smach.cb_interface(input_keys=['base_to_outlet','outlet_to_plug_contact','gripper_to_plug'])
        def get_straighten_goal(ud, goal):
            """Generate an ik goal to straighten the plug in the outlet."""
            pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link').pose)

            pose_base_wrist = fromMsg(ud.base_to_outlet) * fromMsg(ud.outlet_to_plug_contact) * fromMsg(ud.gripper_to_plug).Inverse() * pose_gripper_wrist

            goal = ArmMoveIKGoal()
            goal.pose.pose = toMsg(pose_base_wrist)
            goal.pose.header.stamp = rospy.Time.now()
            goal.pose.header.frame_id = 'base_link'
            goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
            goal.move_duration = rospy.Duration(0.5)
            return goal

        StateMachine.add('STRAIGHTEN_PLUG',
            SimpleActionState('r_arm_ik', ArmMoveIKAction, goal_cb = get_straighten_goal),
            {'succeeded':'WIGGLE_IN',
                'aborted':'FAIL_PULL_BACK_FROM_WALL'})

        # Wiggle the plug
        StateMachine.add('WIGGLE_IN',
                SimpleActionState('wiggle_plug',
                    WigglePlugAction,
                    goal_cb = get_wiggle_goal),
                {'succeeded':'succeeded',
                    'aborted':'FAIL_PULL_BACK_FROM_WALL'})

        ### Recovery states
        @smach.cb_interface(input_keys=['base_to_outlet','gripper_to_plug'])
        def get_pull_back_goal(ud, goal):
            """Generate an ik goal to move along the local x axis of the outlet."""
            pose_outlet_plug = PyKDL.Frame(PyKDL.Vector(-0.10, 0, 0))
            pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link').pose)

            pose_base_wrist = fromMsg(ud.base_to_outlet) * pose_outlet_plug * fromMsg(ud.gripper_to_plug).Inverse() * pose_gripper_wrist

            goal = ArmMoveIKGoal()
            goal.pose.pose = toMsg(pose_base_wrist)
            goal.pose.header.stamp = rospy.Time.now()
            goal.pose.header.frame_id = 'base_link'
            goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
            goal.move_duration = rospy.Duration(3.0)
            return goal

        StateMachine.add('FAIL_PULL_BACK_FROM_WALL',
            SimpleActionState('r_arm_ik', ArmMoveIKAction, goal_cb = get_pull_back_goal),
            {'succeeded':'aborted',
                'aborted':'FAIL_PULL_BACK_FROM_WALL'})
    return sm


if __name__ == "__main__":
    rospy.init_node("plug_in")#,log_level=rospy.DEBUG)
    TFUtil()

    sm_plug_in = construct_sm()

    # Run state machine introspection server
    intro_server = IntrospectionServer('plug_in',sm_plug_in,'/RECHARGE/PLUG_IN')
    intro_server.start()

    # Run state machine action server 
    asw = ActionServerWrapper(
            'plug_in', PlugInAction, sm_plug_in,
            succeeded_outcomes = ['succeeded'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            expand_goal_slots = True,
            pack_result_slots = True)
    asw.run_server()

    rospy.spin()

    intro_server.stop()
