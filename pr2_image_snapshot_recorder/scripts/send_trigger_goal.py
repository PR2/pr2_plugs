#! /usr/bin/env python

PKG = 'pr2_image_snapshot_recorder'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib

from pr2_image_snapshot_recorder.msg import *

if __name__ == '__main__':
    rospy.init_node('snapshot_client')
    client = actionlib.SimpleActionClient('image_snapshot', ImageSnapshotAction)
    rospy.loginfo('Waiting for action server')
    client.wait_for_server()
    

    goal = ImageSnapshotGoal()
    goal.num_images = 5
    goal.topic_name = '/r_forearm_cam/image_rect'
    #goal.output_file_name = '/home/watts/image_snapshot.bag'
    # Fill in the goal here
    rospy.loginfo('Sending snapshot goal')
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo('Got action response')
