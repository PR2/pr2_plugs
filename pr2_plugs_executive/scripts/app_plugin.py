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
roslib.load_manifest('pr2_plugs_executive')

import rospy
from pr2_plugs_msgs.msg import *

import smach

def main():
  rospy.init_node("plugs_app_test")


  intro_client = smach.IntrospectionClient()

  # Set initial state
  injected_ud = smach.UserData()
#  init_set = intro_client.set_initial_state('/recharge','/RECHARGE',['NAVIGATE_TO_OUTLET'],injected_ud)
#  init_set = intro_client.set_initial_state('/recharge','/RECHARGE/NAVIGATE_TO_OUTLET',['UNTUCK_AT_OUTLET'],injected_ud)

  # publisher for commands
  pub = rospy.Publisher('recharge_command', RechargeCommand)
  rospy.sleep(3)

  cmd = RechargeCommand()
  #cmd.plug_id = 'phasespace'
  #cmd.plug_id = 'whiteboard'
  cmd.plug_id = 'local'
  cmd.command = RechargeCommand.PLUG_IN
  pub.publish(cmd)
  
  rospy.sleep(3)

if __name__ == "__main__":
  main()

