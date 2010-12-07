#!/usr/bin/env python

PKG = 'pr2_image_snapshot_recorder'
import roslib; roslib.load_manifest(PKG)

import rospy, actionlib

import rosbag

from sensor_msgs.msg import Image
from pr2_image_snapshot_recorder.msg import ImageSnapshotAction, ImageSnapshotGoal, ImageSnapshotResult

import time
import threading

def get_bagname(topic_name):
    if topic_name.startswith('/'):
        topic_name = topic_name[1:]
    return topic_name.replace('/', '_') + '_' + time.strftime('%Y%m%d_%I%M%S', time.localtime()) + '.bag'
    

class ImageSnapshotter(object):
    def __init__(self):
        self._reset()
        self._mutex = threading.Lock()

        self._server = actionlib.SimpleActionServer('image_snapshot', ImageSnapshotAction, self._execute)

    def _reset(self):
        self._image_sub = None
        self._max_image_count = -1
        self._filename = -1
        self._images_received = 0
        self._bag = None
        self._topic_name = None
        self._done = False

    def _cb(self, msg):
        with self._mutex:
            # Check if we're full
            if self._max_image_count > 0 and self._images_received >= self._max_image_count:
                self._done = True
                return

            # Add msg to bag file
            self._bag.write(self._topic_name, msg)
            self._images_received += 1
        

    def _execute(self, goal):
        # Open bag
        filename = goal.output_file_name if goal.output_file_name else get_bagname(goal.topic_name)

        rospy.loginfo('Snapshot recording of %d images on \"%s\" to file \"%s\"' % \
                          (goal.num_images, goal.topic_name, filename))

        if not filename.endswith('.bag'):
            filename += '.bag'

        self._bag = rosbag.Bag(filename, 'w')

        self._topic_name = goal.topic_name
        self._max_image_count = goal.num_images

        # Set up image subscription
        self._image_sub = rospy.Subscriber(goal.topic_name, Image, self._cb)

        # Poll until we're done
        while not self._done and not rospy.is_shutdown(): 
            if self._server.is_preempt_requested() or \
                    self._server.is_new_goal_available():
                break
            rospy.sleep(0.05)
            
        self._image_sub.unregister()
        
        self._bag.close()

        if self._done:
            self._server.set_succeeded()
        else:
            self._server.set_preempted()
        self._reset()

        rospy.loginfo('Finished snapshot recording of images')

if __name__ == '__main__':
    rospy.init_node('image_snapshotter')

    ImageSnapshotter()
    
    rospy.spin()

    
