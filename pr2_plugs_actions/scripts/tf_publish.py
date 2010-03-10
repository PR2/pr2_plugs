#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import sys
import actionlib
import tf
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint


class tfPublisher:
    def __init__(self, my_topic, my_type, my_field, my_tf_name):
        self.initialized = False
        self.pose = PoseStamped()
        rospy.Subscriber(my_topic, eval(my_type), self.callback)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.my_tf_name = my_tf_name
        self.my_field = my_field

    def callback(self, data):
        self.pose = eval("data."+self.my_field)
        if not self.initialized:
            rospy.loginfo('Initializing tf publisher')
        self.initialized = True

    def publish_tf(self): 
        if self.initialized:
            if(len(self.pose.header.frame_id)>0):
                position = self.pose.pose.position
                orientation = self.pose.pose.orientation
                self.tf_broadcaster.sendTransform((position.x, position.y, position.z), 
                                                  (orientation.x, orientation.y, orientation.z, orientation.w),
                                                  rospy.Time.now(), self.my_tf_name, self.pose.header.frame_id)
        


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    my_argv = rospy.myargv()

    if len(my_argv) != 5:
        rospy.logerr('tf publisher requires 4 arguments: topic type field tf_name')
        
    else:
        my_topic = my_argv[1]
        rospy.loginfo('Topic: %s' % my_topic)
        
        my_type = my_argv[2]
        rospy.loginfo('Type: %s' % my_type)
        
        my_field = my_argv[3]
        rospy.loginfo('Field: %s' % my_field)

        my_tf_name = my_argv[4]
        rospy.loginfo('Tf name: %s' % my_tf_name)

        tf_pub = tfPublisher(my_topic, my_type, my_field, my_tf_name)
        while not rospy.is_shutdown():
            tf_pub.publish_tf()
            rospy.sleep(0.1)



        
