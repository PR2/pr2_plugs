#!/usr/bin/env python
# stub for outlet detection action

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy;
import actionlib;
from pr2_common_action_msgs.msg import *
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_arm_ik_action.tools import *
from pr2_plugs_actions.posestampedmath import PoseStampedMath
from actionlib_msgs.msg import *
import geometry_msgs.msg
from math import pi
import math
import tf

def drange(start, stop, step):
  r = start
  while r < stop:
    yield r
    r += step

def execute_cb(goal):
  rospy.loginfo("Action server received goal")
  preempt_timeout = rospy.Duration(5.0)

  # approach outlet
  cart_space_goal.ik_seed = get_action_seed('pr2_plugs_configuration/approach_outlet_seed')

  time = rospy.Time.now()
  try:
    transformer.waitForTransform("r_wrist_roll_link", "r_gripper_tool_frame", time, rospy.Duration(2.0))
  except rospy.ServiceException, e:
    rospy.logerr('Could not transform between gripper and wrist at time %f' %time.to_sec())
    server.set_aborted()
    return
  pose_gripper_wrist= PoseStampedMath().fromTf(transformer.lookupTransform("r_gripper_tool_frame", "r_wrist_roll_link", time))
  pose_base_outlet = PoseStampedMath(goal.base_to_outlet)
  pose_plug_gripper = PoseStampedMath(goal.gripper_to_plug).inverse()

  rate =rospy.rate(10.0)
  start = rospy.Time.now()
  while(rospy.Time.now() < start + goal.timeout):
    for offset in drange(-0.02, -0.04, 0.002):
      t = rospy.Time.now() - start
      wiggle = goal.wiggle_amplitude * math.sin(t.to_sec() * 2 * math.pi / goal.wiggle_period.to_sec())
      pose_outlet_plug = PoseStampedMath().fromEuler(offset, 0, wiggle, 0, 0, 0)
      cart_space_goal.pose = (pose_base_outlet * pose_outlet_plug * pose_plug_gripper * pose_gripper_wrist).msg
      cart_space_goal.pose.header.stamp = rospy.Time.now()
      cart_space_goal.pose.header.frame_id = 'base_link'
      cart_space_goal.move_duration = rospy.Duration(0.5)
      if self.ik_client.send_goal_and_wait(cart_space_goal, rospy.Duration(20.0), preempt_timeout) != GoalStatus.SUCCEEDED:
        rospy.logerr('Failed to wiggle')

      rate.sleep()



  # return result
  result = WigglePlug2Result()
  server.set_succeeded(result)
  rospy.loginfo("Action server goal finished")  



if __name__ == '__main__':
  #Initialize the node
  name = 'wiggle_plug2'
  rospy.init_node(name)

  # transform listener
  transformer = tf.TransformListener()

  # create action clients we use
  cart_space_client = actionlib.SimpleActionClient('r_arm_ik', PR2ArmIKAction)
  cart_space_client.wait_for_server()
  cart_space_goal = PR2ArmIKGoal()
  rospy.loginfo('Connected to action clients')

  # create action server
  server = actionlib.simple_action_server.SimpleActionServer(name, WigglePlug2Action, execute_cb)
  rospy.loginfo('%s: Action server running', name)


  rospy.spin()
