#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib
import pr2_plugs_msgs.msg
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from math import pi

def plug_on_base_goal():
    goal = pr2_plugs_msgs.msg.VisionPlugDetectionGoal()
    goal.camera_name = '/l_forearm_cam'

    goal.prior.header.stamp = rospy.Time.now()
    goal.prior.header.frame_id = 'base_link'

    goal.prior.pose.position.x = 0.247
    goal.prior.pose.position.y = 0.013
    goal.prior.pose.position.z = 0.322
    goal.prior.pose.orientation.x = -0.5
    goal.prior.pose.orientation.y = -0.5
    goal.prior.pose.orientation.z = 0.5
    goal.prior.pose.orientation.w = 0.5
    goal.origin_on_right = False
    return goal

def plug_on_side_goal():
    goal = pr2_plugs_msgs.msg.VisionPlugDetectionGoal()
    goal.camera_name = '/r_forearm_cam'

    goal.prior = PoseStampedMath().fromEuler(-0.31, -0.24, 0.56, pi, -pi/2, pi).msg
    goal.prior.header.stamp = rospy.Time.now()
    goal.prior.header.frame_id = 'base_link'
    goal.origin_on_right = True
    return goal

def plug_in_gripper_goal():
    goal = pr2_plugs_msgs.msg.VisionPlugDetectionGoal()
    goal.camera_name = "/r_forearm_cam"
    goal.prior = PoseStampedMath().fromEuler(-.03, 0, 0, pi/2, 0, -pi/6).inverse().msg
    goal.prior.header.stamp = rospy.Time.now()
    goal.prior.header.frame_id = "r_gripper_tool_frame"
    goal.origin_on_right = True
    return goal

def new_plug_on_base_goal():
    goal = pr2_plugs_msgs.msg.VisionPlugDetectionGoal()
    goal.camera_name = '/r_forearm_cam'

    goal.prior.header.stamp = rospy.Time.now()
    goal.prior.header.frame_id = 'base_link'

    goal.prior.pose.position.x = 0.075
    goal.prior.pose.position.y = 0.03
    goal.prior.pose.position.z = 0.24
    goal.prior.pose.orientation.x = 0.5
    goal.prior.pose.orientation.y = 0.5
    goal.prior.pose.orientation.z = 0.5
    goal.prior.pose.orientation.w = 0.5
    goal.origin_on_right = False
    return goal

def detect_plug_client():
    client = actionlib.SimpleActionClient('vision_plug_detection', pr2_plugs_msgs.msg.VisionPlugDetectionAction)
    client.wait_for_server()
    print 'action server up'
    #goal = plug_on_base_goal()
    #goal = plug_on_side_goal()
    #goal = plug_in_gripper_goal()
    goal = new_plug_on_base_goal()

    client.send_goal(goal)
    print 'goal sent'
    client.wait_for_goal_to_finish()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('vision_detect_plug_client')
    result = detect_plug_client()
    print 'Result:'
    print result
