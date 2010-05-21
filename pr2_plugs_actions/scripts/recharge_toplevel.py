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
import tf

from pr2_plugs_msgs.msg import *
from pr2_plugs_msgs.srv import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from std_srvs.srv import *

# State machine classes
from smach import *

import actionlib

class TFUtil():
    transformer = None
        
    @staticmethod
    def wait_and_transform(frame_id,pose):
        if not TFUtil.transformer:
            TFUtil.transformer = tf.TransformListener(True, rospy.Duration(60.0))    

        try:
            TFUtil.transformer.waitForTransform(frame_id, pose.header.frame_id, pose.header.stamp, rospy.Duration(2.0))
        except rospy.ServiceException, ex:
            rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
            raise ex
        return TFUtil.transformer.transformPose(frame_id, pose)

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
        plug_id = ud.sm_goal.command.plug_id

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

def store_outlet_result(ud, result_state, result):
    ud.outlet_pose = result.outlet_pose

def store_fetch_plug_result(ud, result_state, result):
    ud.plug_on_base_pose = result.plug_on_base_pose

def get_plug_in_goal(ud, goal):
    ud.outlet_pose.header.stamp = rospy.Time.now()
    return PlugInGoal(base_to_outlet = ud.outlet_pose)

def store_plug_in_result(ud, result_state, result):
    ud.plug_in_gripper_pose = result.gripper_to_plug
    if result_state is GoalStatus.SUCCEEDED:
        ud.sm_result.state.state = RechargeState.PLUGGED_IN

def get_wiggle_out_goal(ud, goal):
    goal = WigglePlugGoal()
    goal.gripper_to_plug = ud.plug_in_gripper_pose
    goal.gripper_to_plug.header.stamp = rospy.Time.now()

    goal.base_to_outlet = ud.outlet_pose
    goal.base_to_outlet.header.stamp = rospy.Time.now()

    goal.wiggle_period = rospy.Duration(0.5)
    goal.insert = 0
    return goal

def get_stow_plug_goal(ud, goal):
    goal = StowPlugGoal()
    goal.gripper_to_plug = ud.plug_in_gripper_pose
    goal.base_to_plug = ud.plug_on_base_pose
    return goal

def set_unplug_result(ud, result_state, result):
    if result_state is GoalStatus.SUCCEEDED:
        ud.sm_result.state.state = RechargeState.UNPLUGGED

class GetNavGoalState(State):
    def __init__(self):
        State.__init__(self, outcomes=['local', 'non-local'])
    def enter(self):
        if self.userdata.sm_goal.command.plug_id == 'local':
            return 'local'
        else:
            return 'non-local'


# Define state to process the recharge goal
class ProcessRechargeCommandState(State):
    def __init__(self):
        State.__init__(self, outcomes=['nav_plug_in','unplug','aborted'])
    def enter(self):
        # Process the command to determine if we should plug in or unplug
        command = self.userdata.sm_goal.command.command
        if command is RechargeCommand.PLUG_IN:
            return 'nav_plug_in'
        elif command is RechargeCommand.UNPLUG:
            return 'unplug'
        # Set the default result, which is error
        self.userdata.sm_result.state.state = RechargeState.FAILED
        return 'aborted'

class RemainUnpluggedState(State):
    def __init__(self):
        State.__init__(self, default_outcome='done')
    def enter(self):
        self.userdata.sm_result.state.state = RechargeState.UNPLUGGED

class AbortedState(State):
    def __init__(self):
        State.__init__(self, default_outcome='aborted')

def main():
    rospy.init_node("recharge_toplevel",log_level=rospy.DEBUG)

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
    sm_recharge.local_userdata.outlet_pose = PoseStamped()
    sm_recharge.local_userdata.plug_on_base_pose = PoseStamped()
    sm_recharge.local_userdata.plug_in_gripper_pose = PoseStamped()

    # Define entry states
    with sm_recharge:
        StateMachine.add_state('PROCESS_RECHARGE_COMMAND',
                         ProcessRechargeCommandState(),
                         { 'nav_plug_in':'NAVIGATE_TO_OUTLET',
                           'unplug':'UNPLUG'})
        
        # Define navigation sm_recharge
        sm_nav = StateMachine(outcomes=['succeeded','aborted','preempted'])
        with sm_nav:
            StateMachine.add_state('GET_NAV_GOAL', 
                                   GetNavGoalState(),
                                   {'local': 'UNTUCK_AT_OUTLET',
                                    'non-local': 'SAFETY_TUCK'})
            StateMachine.add_state('SAFETY_TUCK', 
                                   SimpleActionState('tuck_arms', TuckArmsAction,
                                                     goal = TuckArmsGoal(False,True,True)),
                                   { 'succeeded':'NAVIGATE' })
            StateMachine.add_state('NAVIGATE', 
                                   NavigateToOutletState(exec_timeout = rospy.Duration(20*60.0)),
                                   { 'succeeded':'UNTUCK_AT_OUTLET' })
            StateMachine.add_state('UNTUCK_AT_OUTLET', 
                                   SimpleActionState('tuck_arms', TuckArmsAction,
                                                     goal = TuckArmsGoal(True,True,True)))
            sm_nav.set_initial_state(['GET_NAV_GOAL'])
            sm_nav.set_retrieve_keys(['sm_goal'])

        StateMachine.add_state('NAVIGATE_TO_OUTLET', sm_nav,
                               {'succeeded':'DETECT_OUTLET',
                                'aborted':'FAIL_STILL_UNPLUGGED'}),
        
        StateMachine.add_state('DETECT_OUTLET', 
                               SimpleActionState('detect_outlet', DetectOutletAction,
                                                 goal = DetectOutletGoal(),
                                                 result_cb = store_outlet_result),
                               {'succeeded':'FETCH_PLUG',
                                'aborted':'FAIL_STILL_UNPLUGGED'}),
        StateMachine.add_state('FETCH_PLUG',
                               SimpleActionState('fetch_plug',FetchPlugAction,
                                                 exec_timeout=rospy.Duration(300.0),
                                                 goal = FetchPlugGoal(),
                                                 result_cb = store_fetch_plug_result),
                               {'succeeded':'PLUG_IN',
                                'aborted':'FAIL_OPEN_GRIPPER'}),
        
        StateMachine.add_state('PLUG_IN',
                               SimpleActionState('plug_in',PlugInAction,
                                                 goal = PluginGoal(),
                                                 goal_cb = get_plug_in_goal,
                                                 result_cb = store_plug_in_result,
                                                 exec_timeout = rospy.Duration(5*60.0)),
                               { 'succeeded':'plugged_in',
                                 'aborted':'RECOVER_STOW_PLUG'})
        
        # Define unplug sub-state machine
        sm_unplug = StateMachine(outcomes = ['succeeded','aborted','preempted'])
        unplug_keys = ('sm_result','plug_in_gripper_pose','plug_on_base_pose','outlet_pose')
        sm_unplug.set_retrieve_keys(unplug_keys)
        sm_unplug.set_return_keys(unplug_keys)
        with sm_unplug:
            # Make sure the gripper is held tightly
            StateMachine.add_state('CLOSE_GRIPPER',
                             SimpleActionState('r_gripper_controller/gripper_action', Pr2GripperCommandAction,
                                               goal = close_gripper_goal),
                             { 'succeeded':'aborted',
                               'aborted':'WIGGLE_OUT'})
            StateMachine.add_state('WIGGLE_OUT',
                             SimpleActionState('wiggle_plug',WigglePlugAction,
                                               goal_cb = get_wiggle_out_goal),
                             {'succeeded':'STOW_PLUG'})
            
            StateMachine.add_state('STOW_PLUG',
                             SimpleActionState('stow_plug',StowPlugAction,
                                               goal_cb = get_stow_plug_goal,
                                               result_cb = set_unplug_result),
                             {'succeeded':'succeeded'})
            
            sm_unplug.set_initial_state(['CLOSE_GRIPPER'])

        StateMachine.add_state('UNPLUG',
                         sm_unplug,
                         { 'succeeded':'unplugged',
                           'aborted':'FAIL_OPEN_GRIPPER'})
        
        # Add recovery states
        # Stow the plug
        StateMachine.add_state('RECOVER_STOW_PLUG',
                         SimpleActionState('stow_plug',StowPlugAction,
                                           goal_cb = get_stow_plug_goal),
                         { 'succeeded':'FAIL_STILL_UNPLUGGED',
                           'aborted':'FAIL_OPEN_GRIPPER'})
        # Add failure states
        # State to fail to if we're still unplugged
        StateMachine.add_state('FAIL_STILL_UNPLUGGED', 
                         RemainUnpluggedState(),
                         {'done':'FAIL_LOWER_SPINE'})
        
        # Make sure we're not holding onto the plug
        StateMachine.add_state('FAIL_OPEN_GRIPPER',
                         SimpleActionState('r_gripper_controller/gripper_action', Pr2GripperCommandAction,
                                           goal = open_gripper_goal),
                         {'succeeded':'FAIL_UNTUCK'})
        
        StateMachine.add_state('FAIL_UNTUCK',
                         SimpleActionState('tuck_arms',TuckArmsAction,
                                           goal = TuckArmsGoal(True,True,True)),
                         {'succeeded':'FAIL_LOWER_SPINE'})
        
        # Lower the spine on cleanup
        StateMachine.add_state('FAIL_LOWER_SPINE',
                         SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                                           goal = SingleJointPositionGoal(position=0.01)),
                         {'succeeded':'aborted'})
        # Set the initial state
        sm_recharge.set_initial_state(['PROCESS_RECHARGE_COMMAND'])


    # Run state machine introspection server
    intro_server = smach.IntrospectionServer('recharge',sm_recharge,'/RECHARGE')
    intro_server.start()

    # Run state machine action server 
    sms = ActionServerStateMachine(
            'recharge', RechargeAction, sm_recharge,
            succeeded_outcomes = ['plugged_in','unplugged'],
            aborted_outcomes = ['aborted'],
            preempted_outcomes = ['preempted']
            )
    sms.run_server()

    rospy.spin()

    intro_server.stop()


if __name__ == "__main__":
    main()

