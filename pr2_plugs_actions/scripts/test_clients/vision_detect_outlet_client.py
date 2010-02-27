#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib
import pr2_plugs_msgs.msg
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from math import pi

def outlet_goal():
    goal = pr2_plugs_msgs.msg.VisionOutletDetectionGoal()
    goal.camera_name = '/forearm_camera_r'

    goal.prior = PoseStampedMath().fromEuler(0.6, -0.15, 0.29, 0.0, 0.0, 0.0).msg
    goal.prior.header.stamp = rospy.Time.now()
    goal.prior.header.frame_id = 'base_link'

    # TODO: set wall_normal

    return goal

def detect_outlet_client():
    client = actionlib.SimpleActionClient('vision_outlet_detection', pr2_plugs_msgs.msg.VisionOutletDetectionAction)
    client.wait_for_server()
    print 'action server up'
    goal = outlet_goal()

    client.send_goal(goal)
    print 'goal sent'
    client.wait_for_goal_to_finish()
    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('vision_detect_outlet_client')
    result = detect_outlet_client()
    print 'Result:'
    print result
