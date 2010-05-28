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

from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
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
        except rospy.ServiceException, e:
            rospy.logerr('Could not transform between %s and %s' % (frame_id,pose.header.frame_id))
            raise e
        return TFUtil.transformer.transformPose(frame_id, pose)

# Callback to store the plug detection result
def store_detect_plug_result(ud, result_state, result):
    ud.sm_result.gripper_to_plug = TFUtil.wait_and_transform('r_gripper_tool_frame',result.plug_pose)

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

class Plugin:
    def __init__(self, name):
        # transform listener
        self.transformer = tf.TransformListener()

        # create action clients we use
        self.cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
        self.cart_space_client.wait_for_server()

        self.spine_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
        self.spine_client.wait_for_server()

        rospy.loginfo('Connected to action clients')

        self.jnt_effort = 0.0
        self.jnt_counter = 0
        self.preempt_timeout = rospy.Duration(5.0)

        # create action server
        self.server = actionlib.simple_action_server.SimpleActionServer(name, PluginAction, self.execute_cb)
        rospy.loginfo('%s: Action server running', name)

    def outlet_to_plug_error(self):
        time = rospy.Time.now()
        try:
            self.transformer.waitForTransform("base_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
        except rospy.ServiceException, e:
            rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
            self.server.set_aborted()
            return
        pose_base_gripper= PoseStampedMath().fromTf(self.transformer.lookupTransform("base_link","r_gripper_tool_frame", time))
        pose_outlet_base = self.pose_base_outlet.inverse()
        pose_gripper_plug = self.pose_plug_gripper.inverse()
        outlet_to_plug = (pose_outlet_base*pose_base_gripper*pose_gripper_plug).msg
        return outlet_to_plug

    def move_outlet_plug(self, pose):
        cart_space_goal = PR2ArmIKGoal()
        cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')
        cart_space_goal.pose = (self.pose_base_outlet * pose * self.pose_plug_gripper * self.pose_gripper_wrist).msg
        cart_space_goal.pose.header.stamp = rospy.Time.now()
        cart_space_goal.pose.header.frame_id = 'base_link'
        cart_space_goal.move_duration = rospy.Duration(1.5)
        return self.cart_space_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), self.preempt_timeout)

    def js_cb(self, msg):
        for name,effort in zip(msg.name, msg.effort):
            if name == 'r_wrist_roll_joint':
                self.jnt_effort = effort
                self.jnt_counter = self.jnt_counter+1

    def execute_cb(self, goal):
        rospy.loginfo("Action server received goal")

        # subscribing to joint state messages
        sub = rospy.Subscriber("joint_states", JointState, self.js_cb)

        """
        # make sure spine is down
        rospy.loginfo("Make sure spine is down...")
        spine_goal = SingleJointPositionGoal()
        spine_goal.position = 0.01
        if self.spine_client.send_goal_and_wait(spine_goal, rospy.Duration(20.0), self.preempt_timeout) != GoalStatus.SUCCEEDED:
            rospy.logerr('Moving down spine failed')
            sub.unregister()
            self.server.set_aborted()
            return
        """

        # get poses for later use
        time = rospy.Time.now()
        try:
            self.transformer.waitForTransform("r_wrist_roll_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
        except rospy.ServiceException, e:
            rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
            sub.unregister()
            self.server.set_aborted()
            return
        self.pose_gripper_wrist= PoseStampedMath().fromTf(self.transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", time))
        self.pose_base_outlet = PoseStampedMath(goal.base_to_outlet)
        self.pose_plug_gripper = PoseStampedMath(goal.gripper_to_plug).inverse()

        # approach outlet
        step = 0.005
        for offset in drange(-0.07, 0.09, step):
            desired_pose = PoseStampedMath().fromEuler(offset, 0, 0, 0, 0, 0)
            self.move_outlet_plug(desired_pose)
            measured_pose = self.outlet_to_plug_error()
            if math.fabs(measured_pose.pose.position.x - desired_pose.msg.pose.position.x) > 0.002:
                rospy.loginfo("hit the wall or inserted into outlet")
                break

        # get current error 
        initial_pose = self.outlet_to_plug_error()    
        motion_list = drange(0, 0.25, 0.025)
        for motion in motion_list:
            pose_outlet_plug = PoseStampedMath(initial_pose) * PoseStampedMath().fromEuler(0, 0, 0, motion, 0, 0)
            self.move_outlet_plug(pose_outlet_plug)
            # wait for effort measurement to come in
            self.jnt_counter = 0
            while self.jnt_counter == 0:
                rospy.sleep(0.01)
            rospy.loginfo('Measured effort of %f Nm when turned %f radians', self.jnt_effort, motion)
            if math.fabs(self.jnt_effort) > 1.0:
                # we are in the outlet
                rospy.loginfo("Action server goal finished")
                sub.unregister()
                self.server.set_succeeded(PluginResult())
                return
            
        # we are not in the outlet
        self.move_outlet_plug(PoseStampedMath(initial_pose) * PoseStampedMath().fromEuler(-0.05, 0, 0, 0, 0, 0))
        rospy.logerr('We are not in the outlet')
        sub.unregister()
        self.server.set_aborted()

def construct_sm():
    TFUtil()

    # Construct state machine
    sm = StateMachine(['succeeded','aborted','preempted'])

    # Define nominal sequence
    with sm:
        StateMachine.add('DETECT_PLUG_IN_GRIPPER',
                SimpleActionState('detect_plug',
                    DetectPlugInGripperAction,
                    goal = DetectPlugInGripperGoal(),
                    result_cb = store_detect_plug_result),
                {'succeeded':'INSERT_PLUG'})

        StateMachine.add('LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.01)),
                {'succeeded':'aborted'})

        class ApproachOutletState(SPAState):
            def enter():


        StateMachine.add('INSERT_PLUG',
                SimpleActionState('plugin', PluginAction,
                    goal_cb = get_plugin_goal),
                {'succeeded':'WIGGLE_IN'})

        StateMachine.add('WIGGLE_IN',
                SimpleActionState('wiggle_plug',
                    WigglePlugAction,
                    goal_cb = get_wiggle_goal),
                {'succeeded':'succeeded'})
    return sm
