#!/usr/bin/env python
# stub for plug detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *



#server actionlib.simple_action_server.SimpleActionServer

def execute_cb(goal):
  rospy.loginfo("Action server received goal")

  # return the prior as the result
  result = VisionPlugDetectionResult()
  result.plug_pose = goal.prior
  result.plug_pose = server.set_succeeded(result)
  


if __name__ == '__main__':
  #Initialize the node
  name = 'vision_plug_detection'
  rospy.init_node(name)
  server = actionlib.simple_action_server.SimpleActionServer(name, VisionPlugDetectionAction, execute_cb)
  rospy.loginfo('%s: Action server running', name)
  rospy.spin()
