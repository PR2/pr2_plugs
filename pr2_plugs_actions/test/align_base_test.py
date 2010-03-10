#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
import signal
roslib.load_manifest('pr2_plugs_actions')

import rospy

import os
import sys
import time

from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *

import actionlib

def main():
	rospy.init_node("align_base_test")

	rospy.loginfo("Starting action client")
	detect_wall_norm = actionlib.SimpleActionClient('align_base', AlignBaseAction)
	detect_wall_norm.wait_for_server()

	rospy.loginfo("Sending goal")
	goal = AlignBaseGoal()
	goal.look_point.header.frame_id = 'base_link'
	goal.look_point.header.stamp = rospy.Time.now()
	goal.look_point.point.x = -0.14
	goal.look_point.point.y = -0.82
	goal.look_point.point.z = 0.5

	detect_wall_norm.send_goal(goal)
	detect_wall_norm.wait_for_result(rospy.Duration(60.0))


if __name__ == "__main__":
	main()
