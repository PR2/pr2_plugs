#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_plugs_actions')

import rospy
from pr2_plugs_msgs.msg import *
from pr2_controllers_msgs.msg import *
import actionlib


def main():
  rospy.init_node("open_gripper_test")

  # Open gripper goal
  open_gripper_goal = Pr2GripperCommandGoal()
  open_gripper_goal.command.position = 0.07
  open_gripper_goal.command.max_effort = 99999

  # publisher for commands
  client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
  client.wait_for_server(rospy.Duration(3.0))
  client.send_goal(open_gripper_goal)

  client.wait_for_result()

  print client.get_result()

if __name__ == "__main__":
  main()

