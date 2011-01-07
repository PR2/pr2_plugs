#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of the Willow Garage nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy

import threading

import actionlib
import PyKDL

from pr2_plugs_msgs.msg import *
from pr2_plugs_msgs.srv import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *

from std_srvs.srv import *

from tf_conversions.posemath import fromMsg, toMsg
from pr2_arm_move_ik.tools import *

# State machine classes
import smach
from smach import *
from smach_ros import *

from detect_outlet import construct_sm as construct_detect_outlet_sm
from fetch_plug import construct_sm as construct_fetch_plug_sm
from plug_in import construct_sm as construct_plug_in_sm

from pr2_plugs_actions.tf_util import TFUtil

def main():
    rospy.init_node("recharge_toplevel")
    TFUtil()

    # Close gripper goal
    close_gripper_goal = Pr2GripperCommandGoal()
    close_gripper_goal.command.position = 0.0
    close_gripper_goal.command.max_effort = 99999

    open_gripper_goal = Pr2GripperCommandGoal()
    open_gripper_goal.command.position = 0.07
    open_gripper_goal.command.max_effort = 99999

    # Construct state machine
    recharge_sm = StateMachine(
            outcomes=['plugged_in','unplugged','aborted','preempted'],
            input_keys = ['recharge_command'],
            output_keys = ['recharge_state'])

    # Set the initial state explicitly
    recharge_sm.set_initial_state(['PROCESS_RECHARGE_COMMAND'])
    recharge_sm.userdata.recharge_state = RechargeState(state=RechargeState.UNPLUGGED)

    with recharge_sm:
        ### PLUGGING IN ###
        @smach.cb_interface(input_keys=['recharge_command'])
        def plug_in_cond(ud):
            command = ud.recharge_command.command
            if command is RechargeCommand.PLUG_IN:
                return True
            elif command is RechargeCommand.UNPLUG:
                return False
        StateMachine.add('PROCESS_RECHARGE_COMMAND',
                ConditionState(cond_cb = plug_in_cond),
                { 'true':'NAVIGATE_TO_OUTLET',
                    'false':'UNPLUG'})
        
        sm_nav = StateMachine(
                outcomes=['succeeded','aborted','preempted'],
                input_keys = ['recharge_command'])
        StateMachine.add('NAVIGATE_TO_OUTLET', sm_nav,
                {'succeeded':'DETECT_OUTLET',
                    'aborted':'FAIL_STILL_UNPLUGGED'})
        with sm_nav:
            StateMachine.add('GOAL_IS_LOCAL', 
                    ConditionState(
                        cond_cb = lambda ud: ud.recharge_command.plug_id == 'local',
                        input_keys = ['recharge_command']),
                    {'true': 'UNTUCK_AT_OUTLET',
                        'false': 'SAFETY_TUCK'})
            StateMachine.add('SAFETY_TUCK', 
                    SimpleActionState('tuck_arms', TuckArmsAction,
                        goal = TuckArmsGoal(True,True)),
                    { 'succeeded':'GET_OUTLET_LOCATIONS',
                      'aborted':'SAFETY_TUCK'})
            StateMachine.add('GET_OUTLET_LOCATIONS',
                    ServiceState('outlet_locations', GetOutlets,
                        response_slots=['poses']),
                    {'succeeded':'NAVIGATE'},
                    remapping={'poses':'approach_poses'})

            @smach.cb_interface(input_keys=['approach_poses','recharge_command'])
            def get_outlet_approach_goal(ud,goal):
                """Get the approach pose from the outlet approach poses list"""

                # Get id from command
                plug_id = ud.recharge_command.plug_id

                # Grab the relevant outlet approach pose
                for outlet in ud.approach_poses:
                    if outlet.name == plug_id or outlet.id == plug_id:
                        target_pose = PoseStamped(pose=outlet.approach_pose)

                # Create goal for move base
                move_base_goal = MoveBaseGoal()
                move_base_goal.target_pose = target_pose
                move_base_goal.target_pose.header.stamp = rospy.Time.now()
                move_base_goal.target_pose.header.frame_id = "map"

                return move_base_goal
            StateMachine.add('NAVIGATE', 
                             SimpleActionState('pr2_move_base',MoveBaseAction,
                                               goal_cb=get_outlet_approach_goal,
                                               exec_timeout = rospy.Duration(20*60.0)),
                             { 'succeeded':'UNTUCK_AT_OUTLET' })
            StateMachine.add('UNTUCK_AT_OUTLET', 
                             SimpleActionState('tuck_arms', TuckArmsAction,
                                               goal = TuckArmsGoal(False, False)))

        StateMachine.add('DETECT_OUTLET', 
                         SimpleActionState('detect_outlet',DetectOutletAction,
                                           result_slots = ['base_to_outlet_pose']),
                         {'succeeded':'FETCH_PLUG',
                          'aborted':'FAIL_STILL_UNPLUGGED'},
                         remapping = {'base_to_outlet_pose':'base_to_outlet'})

        StateMachine.add('FETCH_PLUG',
                         SimpleActionState('fetch_plug',FetchPlugAction,
                                           result_slots = ['plug_on_base_pose', 'gripper_plug_grasp_pose']),
                         {'succeeded':'PLUG_IN',
                          'aborted':'FAIL_OPEN_GRIPPER'},
                         remapping = {'plug_on_base_pose':'base_to_plug_on_base', 'gripper_plug_grasp_pose':'gripper_to_plug_grasp'})
        
        @smach.cb_interface(input_keys=['recharge_state'], output_keys=['recharge_state'])
        def set_plug_in_result(ud, result_status, result):
            if result_status == GoalStatus.SUCCEEDED:
                ud.recharge_state.state = RechargeState.PLUGGED_IN
        StateMachine.add('PLUG_IN',
                         SimpleActionState('plug_in',PlugInAction,
                                           goal_slots = ['base_to_outlet'],
                                           result_slots = ['gripper_to_plug'],
                                           result_cb = set_plug_in_result),
                         { 'succeeded':'plugged_in',
                           'aborted':'RECOVER_STOW_PLUG'})
        
        ### UNPLUGGING ###
        unplug_sm = StateMachine(
            outcomes = ['succeeded','aborted','preempted'],
            input_keys=['recharge_state','gripper_to_plug_grasp','gripper_to_plug','base_to_outlet','base_to_plug_on_base'],
            output_keys=['recharge_state'])
        StateMachine.add('UNPLUG', unplug_sm,
                         { 'succeeded':'unplugged',
                           'aborted':'FAIL_OPEN_GRIPPER'})
        with unplug_sm:
            """Unplug from outlet"""
            # Make sure the gripper is held tightly
            StateMachine.add('CLOSE_GRIPPER',
                    SimpleActionState('r_gripper_controller/gripper_action', Pr2GripperCommandAction,
                        goal = close_gripper_goal),
                    { 'succeeded':'aborted',
                        'aborted':'WIGGLE_OUT'})
            
            # Wiggle out
            def get_wiggle_out_goal(ud, goal):
                # Set period
                goal.wiggle_period = rospy.Duration(0.5)
                goal.insert = 0
                return goal
            StateMachine.add('WIGGLE_OUT',
                    SimpleActionState('wiggle_plug',WigglePlugAction,
                        goal_slots = ['gripper_to_plug','base_to_outlet'],
                        goal_cb = get_wiggle_out_goal),
                    {'succeeded':'PULL_BACK_FROM_WALL'})

            @smach.cb_interface(input_keys=['base_to_outlet','gripper_to_plug'])
            def get_pull_back_goal(ud, goal):
                """Generate an ik goal to move along the local x axis of the outlet."""

                pose_outlet_plug = PyKDL.Frame(PyKDL.Vector(-0.10, 0, 0))
                pose_gripper_wrist = fromMsg(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link').pose)

                pose_base_wrist = fromMsg(ud.base_to_outlet) * pose_outlet_plug * fromMsg(ud.gripper_to_plug).Inverse() * pose_gripper_wrist

                goal.pose.pose = toMsg(pose_base_wrist)
                goal.pose.header.stamp = rospy.Time.now()
                goal.pose.header.frame_id = 'base_link'
                goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
                goal.move_duration = rospy.Duration(5.0)

            StateMachine.add('PULL_BACK_FROM_WALL',
                             SimpleActionState('r_arm_ik', ArmMoveIKAction, goal_cb = get_pull_back_goal),
                             {'succeeded':'STOW_PLUG',
                              'aborted':'WIGGLE_OUT'})
            
            # Stow plug
            @smach.cb_interface(input_keys=['recharge_state'],output_keys=['recharge_state'])
            def set_unplug_result(ud, result_state, result):
                if result_state is GoalStatus.SUCCEEDED:
                    ud.recharge_state.state = RechargeState.UNPLUGGED

            StateMachine.add('STOW_PLUG', SimpleActionState('stow_plug',StowPlugAction,
                                                            goal_slots = ['gripper_to_plug_grasp','base_to_plug'],
                                                            result_cb = set_unplug_result),
                             {'succeeded':'succeeded'},
                             remapping = {'base_to_plug':'base_to_plug_on_base'})

        ### RECOVERY STATES ###
        StateMachine.add('RECOVER_STOW_PLUG', SimpleActionState('stow_plug',StowPlugAction,
                                                                goal_slots = ['gripper_to_plug_grasp','base_to_plug']),
                         { 'succeeded':'FAIL_STILL_UNPLUGGED',
                           'aborted':'FAIL_OPEN_GRIPPER'},
                         remapping = {'base_to_plug':'base_to_plug_on_base'})

        ### FAILURE STATES ###
        # State to fail to if we're still unplugged
        @smach.cb_interface(input_keys=['recharge_state'],output_keys=['recharge_state'],outcomes=['done'])
        def remain_unplugged(ud):
            ud.recharge_state.state = RechargeState.UNPLUGGED
            return 'done'
        StateMachine.add('FAIL_STILL_UNPLUGGED', 
                         CBState(cb = remain_unplugged),
                         {'done':'FAIL_LOWER_SPINE'})
        
        # Make sure we're not holding onto the plug
        StateMachine.add('FAIL_OPEN_GRIPPER',
                         SimpleActionState('r_gripper_controller/gripper_action', Pr2GripperCommandAction,
                                           goal = open_gripper_goal),
                         {'succeeded':'FAIL_UNTUCK'})
        
        StateMachine.add('FAIL_UNTUCK',
                         SimpleActionState('tuck_arms',TuckArmsAction,
                                           goal = TuckArmsGoal(False, False)),
                         {'succeeded':'FAIL_LOWER_SPINE'})
        
        # Lower the spine on cleanup
        StateMachine.add('FAIL_LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                                  goal = SingleJointPositionGoal(position=0.02)),
                {'succeeded':'aborted'})


    # Run state machine introspection server
    intro_server = IntrospectionServer('recharge',recharge_sm,'/RECHARGE')
    intro_server.start()

    # Run state machine action server 
    sms = ActionServerWrapper(
            'recharge', RechargeAction, recharge_sm,
            succeeded_outcomes = ['plugged_in','unplugged'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'],
            goal_slots_map = {'command':'recharge_command'},
            result_slots_map = {'state':'recharge_state'})
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()

