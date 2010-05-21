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

from actionlib import *
from pr2_plugs_msgs.msg import *
from pr2_plugs_msgs.srv import *

class RechargeActionWebAdapter:
  def __init__(self):
    # Construct action client
    self.recharge_client = SimpleActionClient('recharge',RechargeAction)
    if self.recharge_client.wait_for_server(rospy.Duration(120.0)):
      rospy.loginfo("Recharge action web adapter connected to recharge action server.")
    else:
      rospy.logerr("Recharge action web adapter timed out while waiting for recharge action server.")
      raise Exception()

    # Construct publisher for recharge state, and broadcast initial state
    self.recharge_state = RechargeState()
    self.recharge_state.state = RechargeState.UNPLUGGED

    self.recharge_state_pub = rospy.Publisher('recharge_state', RechargeState, None, False, True)
    self.recharge_state_pub.publish(self.recharge_state)

    # Construct subscriber for recharge commands
    self.recharge_command_sub = rospy.Subscriber("recharge_command", RechargeCommand, self.recharge_command_cb)
    self.recharge_request_srv = rospy.Service('recharge_request', RechargeRequest, self.recharge_request_cb)


  def recharge_request_cb(self, req):
    self.recharge_command_cb(req.command)

  def recharge_command_cb(self,msg):
    if self.recharge_state.state == RechargeState.WAITING_FOR_STATE:
      rospy.logerr("Recharge action web adapter can't execute commands while still waiting for state.")
      return
    
    # There are only two valid things to do
    valid_plug_in_cmd = (self.recharge_state.state == RechargeState.UNPLUGGED and msg.command == RechargeCommand.PLUG_IN)
    valid_unplug_cmd = (self.recharge_state.state == RechargeState.PLUGGED_IN and msg.command == RechargeCommand.UNPLUG)

    if valid_plug_in_cmd or valid_unplug_cmd:
      # Publish ack to web
      rospy.loginfo("Recharge action web adapter sending command to action server...")
      self.recharge_state.state = RechargeState.WAITING_FOR_STATE
      self.recharge_state_pub.publish(self.recharge_state)

      # Send action to action server
      self.recharge_client.send_goal(
          RechargeGoal(command = msg),
          done_cb = self.action_done_cb)

      # Store recharge command
      self.recharge_command = msg.command
    else:
      rospy.logerr("Invalid command for the current recharge state.")

  def action_done_cb(self,result_state,result):
    rospy.loginfo("Recharge action completed with state: %d and result: %s" % (result_state, str(result)))

    # Store result
    self.recharge_state.state = result.state.state

    # Publish result to the web
    self.recharge_state_pub.publish(self.recharge_state)



def main():
  rospy.init_node("recharge_action_web_adapter")
  adapter = RechargeActionWebAdapter()
  rospy.spin()

if __name__ == "__main__":
  main()

