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
from executive_python_common.tf_util import TFUtil

import actionlib
import dynamic_reconfigure.client

__all__ = ['construct_sm']

class OutletSearchState(SPAState):
    def __init__(self, offsets):
        SPAState.__init__(self)

        # Store lateral offset goals
        self.offsets = offsets

        # Create action clients
        self.align_base_client = actionlib.SimpleActionClient('align_base', AlignBaseAction)
        self.align_base_client.wait_for_server()
        self.vision_detect_outlet_client = actionlib.SimpleActionClient('vision_outlet_detection', VisionOutletDetectionAction)
        self.vision_detect_outlet_client.wait_for_server()

    def enter(self):
        """Iterate across a set of offsets to find the outlet"""
        preempt_timeout = rospy.Duration(10.0)

        align_base_goal = self.userdata.align_base_goal
        vision_detect_outlet_goal = self.userdata.vision_detect_outlet_goal

        # Iterate across move_base offsets
        for offset in self.offsets:
            # align base
            rospy.loginfo("Search base alignment...")
            align_base_goal.offset = offset
            if self.align_base_client.send_goal_and_wait(align_base_goal, rospy.Duration(40.0), preempt_timeout) != GoalStatus.SUCCEEDED:
                rospy.logerr('Aligning base failed')
                return 'aborted'

            # call vision outlet detection
            rospy.loginfo("Detecting outlet with the forearm camera...")
            vision_detect_outlet_goal.wall_normal = self.align_base_client.get_result().wall_norm
            vision_detect_outlet_goal.wall_normal.header.stamp = rospy.Time.now()
            vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
            if self.vision_detect_outlet_client.send_goal_and_wait(vision_detect_outlet_goal, rospy.Duration(5.0), preempt_timeout) == GoalStatus.SUCCEEDED:
                # Store the rough outlet position in the state machiine user data structure
                self.userdata.outlet_rough_pose = TFUtil.wait_and_transform(
                        'r_forearm_cam_optical_frame', self.vision_detect_outlet_client.get_result().outlet_pose)
                # Set succeeded, and return
                return 'succeeded'

        rospy.logerr("Could not find outlet in search")
        return 'aborted'


# Code block state for grasping the plug
def construct_sm():
    TFUtil()

    # Check to see if this is running in sim where the dynamic reconfigure doesn't exist
    sim = rospy.get_param('~sim', False)
    rospy.logdebug('sim is %s', sim)
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
    look_point.point.z = 0.3

    wall_norm_goal = DetectWallNormGoal()
    wall_norm_goal.look_point = look_point

    align_base_goal = AlignBaseGoal()
    align_base_goal.look_point = look_point

    vision_detect_outlet_goal = VisionOutletDetectionGoal()
    vision_detect_outlet_goal.camera_name = "/r_forearm_cam"
    vision_detect_outlet_goal.prior = PoseStampedMath().fromEuler(-0.14, -0.82, 0.29, 0, 0, -pi/2).msg
    vision_detect_outlet_goal.prior.header.frame_id = "base_link"

    # Construct state machine
    sm = StateMachine(outcomes=['succeeded','aborted','preempted'])

    # Default userdata
    sm.local_userdata.wall_norm_goal = wall_norm_goal
    sm.local_userdata.align_base_goal = align_base_goal
    sm.local_userdata.vision_detect_outlet_goal = vision_detect_outlet_goal
    sm.local_userdata.outlet_rough_pose = PoseStamped()

    # Define nominal sequence
    with sm:
        # Define SMACH interface
        Container.map_parent_ud_keys({'outlet_pose':'outlet_pose'})

        StateMachine.add('LOWER_SPINE',
                SimpleActionState('torso_controller/position_joint_action', SingleJointPositionAction,
                    goal = SingleJointPositionGoal(position=0.01)),
                {'succeeded':'ROUGH_ALIGN_BASE'})

        StateMachine.add('ROUGH_ALIGN_BASE',
                SimpleActionState('align_base', AlignBaseAction,
                    goal = AlignBaseGoal(offset = 0,look_point=look_point)),
                {'succeeded':'MOVE_ARM_DETECT_OUTLET'})

        StateMachine.add('MOVE_ARM_DETECT_OUTLET',
                JointTrajectoryState('r_arm_plugs_controller','pr2_plugs_configuration/detect_outlet'),
                {'succeeded':'OUTLET_LATERAL_SEARCH',
                    'aborted':'FAIL_MOVE_ARM_OUTLET_TO_FREE'}),

        StateMachine.add('OUTLET_LATERAL_SEARCH',
                OutletSearchState(offsets = (0.0, 0.1, -0.2, 0.3, -0.4)),
                {'succeeded':'PRECISE_ALIGN_BASE',
                    'aborted':'FAIL_MOVE_ARM_OUTLET_TO_FREE'})

        # Align the base precisely
        def get_precise_align_goal(ud, goal):
            goal.offset = ud.outlet_rough_pose.pose.position.y
            return goal
        StateMachine.add('PRECISE_ALIGN_BASE',
                SimpleActionState('align_base', AlignBaseAction,
                    goal = AlignBaseGoal(offset = 0,look_point=look_point),
                    goal_cb = get_precise_align_goal),
                {'succeeded':'DETECT_WALL_NORM',
                    'aborted':'FAIL_MOVE_ARM_OUTLET_TO_FREE'})

        # Get wall norm
        def get_wall_norm_goal(ud, goal):
            ud.wall_norm_goal.look_point.header.stamp = rospy.Time.now()
            return ud.wall_norm_goal

        def store_wall_norm_result(ud, result_state, result):
            ud.vision_detect_outlet_goal.wall_normal = result.wall_norm

        StateMachine.add('DETECT_WALL_NORM',
                SimpleActionState('detect_wall_norm', DetectWallNormAction,
                    goal_cb = get_wall_norm_goal,
                    result_cb = store_wall_norm_result),
                {'succeeded':'DETECT_OUTLET',
                    'aborted':'DETECT_WALL_NORM'})

        # Precise detection
        def get_vision_detect_goal(ud, goal):
            ud.vision_detect_outlet_goal.wall_normal.header.stamp = rospy.Time.now()
            ud.vision_detect_outlet_goal.prior.header.stamp = rospy.Time.now()
            return ud.vision_detect_outlet_goal

        def store_precise_outlet_result(ud, result_state, result):
            if result_state == GoalStatus.SUCCEEDED:
                ud.outlet_pose = TFUtil.wait_and_transform("base_link",result.outlet_pose)

        StateMachine.add('DETECT_OUTLET',
                SimpleActionState('vision_outlet_detection', VisionOutletDetectionAction,
                    goal_cb = get_vision_detect_goal,
                    result_cb = store_precise_outlet_result),
                {'succeeded':'succeeded',
                    'aborted':'FAIL_MOVE_ARM_OUTLET_TO_FREE'})


        # Define recovery states
        StateMachine.add('FAIL_MOVE_ARM_OUTLET_TO_FREE',
                JointTrajectoryState('r_arm_plugs_controller','pr2_plugs_configuration/recover_outlet_to_free'),
                {'succeeded':'aborted'})

    return sm
