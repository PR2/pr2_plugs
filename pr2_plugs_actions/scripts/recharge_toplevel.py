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
import tf

from pr2_plugs_msgs.msg import *
from pr2_plugs_msgs.srv import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *

from std_srvs.srv import *

from pr2_plugs_actions.posestampedmath import PoseStampedMath
from pr2_arm_ik_action.tools import *

# State machine classes
from smach import *

from detect_outlet import construct_sm as construct_detect_outlet_sm
from fetch_plug import construct_sm as construct_fetch_plug_sm
from plug_in import construct_sm as construct_plug_in_sm

from executive_python_common.tf_util import TFUtil

class NavigateToOutletState(SimpleActionState):
    def __init__(self,*args,**kwargs):
        SimpleActionState.__init__(self,*args,
                action_name = 'move_base',
                action_spec = MoveBaseAction,
                goal_cb = self.get_move_base_goal,
                **kwargs)
        # Construct service to access outlet location db
        self.get_outlet_locations = rospy.ServiceProxy('outlet_locations', GetOutlets)


    def get_move_base_goal(self,ud,goal):

        # Get outlet locations
        outlets = self.get_outlet_locations().poses

        # Get id from command
        plug_id = ud.action_goal.command.plug_id

        # Grab the relevant outlet approach pose
        for outlet in outlets:
            if outlet.name == plug_id or outlet.id == plug_id:
                target_pose = PoseStamped(pose=outlet.approach_pose)

        # Create goal for move base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose = target_pose
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.header.frame_id = "map"

        return move_base_goal

class GetNavGoalState(State):
    def __init__(self):
        State.__init__(self, outcomes=['local', 'non-local'])
    def enter(self):
        if self.userdata.action_goal.command.plug_id == 'local':
            return 'local'
        else:
            return 'non-local'


# Define state to process the recharge goal
class ProcessRechargeCommandState(State):
    def __init__(self):
        State.__init__(self, outcomes=['nav_plug_in','unplug','aborted'])
    def enter(self):
        # Process the command to determine if we should plug in or unplug
        command = self.userdata.action_goal.command.command
        if command is RechargeCommand.PLUG_IN:
            return 'nav_plug_in'
        elif command is RechargeCommand.UNPLUG:
            return 'unplug'
        # Set the default result, which is error
        self.userdata.action_result.state.state = RechargeState.FAILED
        return 'aborted'

class RemainUnpluggedState(State):
    def __init__(self):
        State.__init__(self, default_outcome='done')
    def enter(self):
        self.userdata.action_result.state.state = RechargeState.UNPLUGGED

class AbortedState(State):
    def __init__(self):
        State.__init__(self, default_outcome='aborted')

def main():
    rospy.init_node("recharge_toplevel",log_level=rospy.DEBUG)
    TFUtil()

    # Close gripper goal
    close_gripper_goal = Pr2GripperCommandGoal()
    close_gripper_goal.command.position = 0.0
    close_gripper_goal.command.max_effort = 99999

    open_gripper_goal = Pr2GripperCommandGoal()
    open_gripper_goal.command.position = 0.07
    open_gripper_goal.command.max_effort = 99999

    # Construct state machine
    sm_recharge = StateMachine(outcomes=['plugged_in','unplugged','aborted','preempted'])

    # Default userdata fields
    #sm_recharge.local_userdata.base_to_outlet = PoseStamped()
    #sm_recharge.local_userdata.plug_on_base_pose = PoseStamped()
    #sm_recharge.local_userdata.plug_in_gripper_pose = PoseStamped()

    # Define entry states
    with sm_recharge:
        StateMachine.map_parent_ud_keys(['action_goal','action_feedback','action_result'])

        StateMachine.add('PROCESS_RECHARGE_COMMAND',
                ProcessRechargeCommandState(),
                { 'nav_plug_in':'NAVIGATE_TO_OUTLET',
                    'unplug':'UNPLUG'})
        
        # Define navigation sm
        sm_nav = StateMachine(outcomes=['succeeded','aborted','preempted'])
        StateMachine.add('NAVIGATE_TO_OUTLET', sm_nav,
                {'succeeded':'DETECT_OUTLET',
                    'aborted':'FAIL_STILL_UNPLUGGED'})
        with sm_nav:
            Container.map_parent_ud_keys(['action_goal','action_result'])

            StateMachine.add('GET_NAV_GOAL', 
                    GetNavGoalState(),
                    {'local': 'UNTUCK_AT_OUTLET',
                        'non-local': 'SAFETY_TUCK'})
            StateMachine.add('SAFETY_TUCK', 
                    SimpleActionState('tuck_arms', TuckArmsAction,
                        goal = TuckArmsGoal(False,True,True)),
                    { 'succeeded':'NAVIGATE' })
            StateMachine.add('NAVIGATE', 
                    NavigateToOutletState(exec_timeout = rospy.Duration(20*60.0)),
                    { 'succeeded':'UNTUCK_AT_OUTLET' })
            StateMachine.add('UNTUCK_AT_OUTLET', 
                    SimpleActionState('tuck_arms', TuckArmsAction,
                        goal = TuckArmsGoal(True,True,True)))

        # Detect the outlet
        StateMachine.add('DETECT_OUTLET', 
                SimpleActionState('detect_outlet',DetectOutletAction,
                    result_slots_map = {'outlet_pose':'base_to_outlet'}),
                {'succeeded':'FETCH_PLUG',
                    'aborted':'FAIL_STILL_UNPLUGGED'})

        # Fetch plug
        StateMachine.add('FETCH_PLUG',
                SimpleActionState('fetch_plug',FetchPlugAction,
                    result_slots_map = {'plug_on_base_pose':'base_to_plug_on_base'}),
                {'succeeded':'PLUG_IN',
                    'aborted':'FAIL_OPEN_GRIPPER'})
        
        # Plug in
        def set_plug_in_result(ud, result_status, result):
            if result_status == GoalStatus.SUCCEEDED:
                ud.action_result.state.state = RechargeState.PLUGGED_IN
        StateMachine.add('PLUG_IN',
                SimpleActionState('plug_in',PlugInAction,
                    goal_slots = ['base_to_outlet'],
                    result_slots = ['gripper_to_plug'],
                    result_cb = set_plug_in_result),
                { 'succeeded':'plugged_in',
                    'aborted':'RECOVER_STOW_PLUG'})
        
        # Define unplug sm
        sm_unplug = StateMachine(outcomes = ['succeeded','aborted','preempted'])
        with sm_unplug:
            Container.share_parent_userdata()

            # Make sure the gripper is held tightly
            StateMachine.add('CLOSE_GRIPPER',
                    SimpleActionState('r_gripper_controller/gripper_action', Pr2GripperCommandAction,
                        goal = close_gripper_goal),
                    { 'succeeded':'aborted',
                        'aborted':'WIGGLE_OUT'})
            
            # Wiggle out
            def get_wiggle_out_goal(ud, goal):
                # Flash timestamps
                goal.gripper_to_plug.header.stamp = rospy.Time.now()
                goal.base_to_outlet.header.stamp = rospy.Time.now()

                # Set period
                goal.wiggle_period = rospy.Duration(0.5)
                goal.insert = 0
                return goal

            StateMachine.add('WIGGLE_OUT',
                    SimpleActionState('wiggle_plug',WigglePlugAction,
                        goal_slots = ['gripper_to_plug','base_to_outlet'],
                        goal_cb = get_wiggle_out_goal),
                    {'succeeded':'PULL_BACK_FROM_WALL'})

            def get_pull_back_goal(ud, goal):
                """Generate an ik goal to move along the local x axis of the outlet."""

                pose_base_outlet = PoseStampedMath(ud.base_to_outlet)
                pose_outlet_plug = PoseStampedMath().fromEuler(-0.10, 0, 0, 0, 0, 0)
                pose_plug_gripper = PoseStampedMath(ud.gripper_to_plug).inverse()
                pose_gripper_wrist = PoseStampedMath().fromTf(TFUtil.wait_and_lookup('r_gripper_tool_frame', 'r_wrist_roll_link'))

                pose_base_wrist = (pose_base_outlet
                        * pose_outlet_plug
                        * pose_plug_gripper
                        * pose_gripper_wrist).msg

                goal.pose.pose = pose_base_wrist.pose
                goal.pose.header.stamp = rospy.Time.now()
                goal.pose.header.frame_id = 'base_link'
                goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
                goal.move_duration = rospy.Duration(5.0)

            StateMachine.add('PULL_BACK_FROM_WALL',
                SimpleActionState('r_arm_ik', PR2ArmIKAction, goal_cb = get_pull_back_goal),
                {'succeeded':'STOW_PLUG',
                    'aborted':'WIGGLE_OUT'})
            
            # Stow plug
            def get_stow_plug_goal(ud, goal):
                # Flash timestamps
                goal.gripper_to_plug.header.stamp = rospy.Time.now()
                goal.base_to_plug.header.stamp = rospy.Time.now()

            def set_unplug_result(ud, result_state, result):
                if result_state is GoalStatus.SUCCEEDED:
                    ud.action_result.state.state = RechargeState.UNPLUGGED
            StateMachine.add('STOW_PLUG',
                    SimpleActionState('stow_plug',StowPlugAction,
                        goal_slots = ['gripper_to_plug'],
                        goal_slots_map = {'base_to_plug_on_base':'base_to_plug'},
                        goal_cb = get_stow_plug_goal,
                        result_cb = set_unplug_result),
                    {'succeeded':'succeeded'})

        StateMachine.add('UNPLUG', sm_unplug,
                { 'succeeded':'unplugged',
                    'aborted':'FAIL_OPEN_GRIPPER'})

        StateMachine.add('RECOVER_STOW_PLUG',
                SimpleActionState('stow_plug',StowPlugAction,
                    goal_cb = get_stow_plug_goal),
                { 'succeeded':'FAIL_STILL_UNPLUGGED',
                    'aborted':'FAIL_OPEN_GRIPPER'})

        # Add failure states
        # State to fail to if we're still unplugged
        StateMachine.add('FAIL_STILL_UNPLUGGED', 
                RemainUnpluggedState(),
                {'done':'FAIL_LOWER_SPINE'})
        
        # Make sure we're not holding onto the plug
        StateMachine.add('FAIL_OPEN_GRIPPER',
                SimpleActionState('r_gripper_controller/gripper_action', Pr2GripperCommandAction,
                    goal = open_gripper_goal),
                {'succeeded':'FAIL_UNTUCK'})
        
        StateMachine.add('FAIL_UNTUCK',
                SimpleActionState('tuck_arms',TuckArmsAction,
                    goal = TuckArmsGoal(True,True,True)),
                {'succeeded':'FAIL_LOWER_SPINE'})
        
        # Lower the spine on cleanup
        StateMachine.add('FAIL_LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.01)),
                {'succeeded':'aborted'})

        # Set the initial state explicitly
        sm_recharge.set_initial_state(['PROCESS_RECHARGE_COMMAND'])


    # Run state machine introspection server
    intro_server = IntrospectionServer('recharge',sm_recharge,'/RECHARGE')
    intro_server.start()

    # Run state machine action server 
    sms = ActionServerWrapper(
            'recharge', RechargeAction, sm_recharge,
            succeeded_outcomes = ['plugged_in','unplugged'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted'])
    sms.run_server()

    rospy.spin()

    intro_server.stop()

if __name__ == "__main__":
    main()

