#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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
#
# Author: Jonathan Bohren

import roslib
roslib.load_manifest('executive_python')
import rospy

import os,sys,time
import threading
import actionlib

class ActionException(Exception):
  def __init__(self,client=None,name="",result=None):
    self.client = client
    self.name = name
    self.result = result

class ActionExecutive():
  def __init__(self,actions):
    self.preempt_timeout = rospy.Duration(5.0)

    # Declare list of actions for easy construction
    self.actions = actions
    self.action_clients = dict()

    # Construct action clients
    rospy.loginfo("Starting action clients.")
    for name, action_params in self.actions.iteritems():
      # Grab action parameters
      action_spec = action_params[0]
      action_timeout = action_params[1]

      # Create an action client
      ac = actionlib.SimpleActionClient(name,action_spec)
      # Store the action client in the actionclient dictionary for iteration
      self.action_clients[name] = ac

      # Set this action client as a member of this class for convenience
      assert not hasattr(self, name)
      setattr(self, name+"_client", ac)

      # Construct functions that call the actions with default timeouts.
      # This is convienent for actions with an invariant timeout, since it's a
      # property of the action and not of the goal
      assert not hasattr(self, name+"_and_wait")
      setattr(self, name+"_and_wait",
          lambda goal,timeout=action_timeout,_ac=ac,_name=name:
          self.send_goal_and_wait(_ac,_name,goal,timeout))

    # Wait for all the action clients to start (If we do this in parallel it happens a lot faster)
    action_wait_threads = dict()
    for name, ac in self.action_clients.iteritems():
      wait_thread = threading.Thread(target=ac.wait_for_server)
      wait_thread.start()
      action_wait_threads[name]=wait_thread
    for name,thread in action_wait_threads.iteritems():
      rospy.logdebug("Waiting for "+name+" action server...")
      thread.join(60.0)
      if not thread.isAlive():
        rospy.loginfo("Connected to "+name+" action server.")
      else:
        rospy.logerr("Timed out while waiting for "+name+" action server.")
    rospy.loginfo("All action servers ready.")

  # Wrapper to allow for rapid goal passing
  def send_goal_and_wait(self,client,name,goal,timeout):
    rospy.loginfo("Sending blocking goal to "+name+" action for "+str(timeout)+" seconds...");
    rospy.logdebug("Goal for "+name+":\n"+str(goal))
    result = client.send_goal_and_wait(goal,rospy.Duration(timeout),self.preempt_timeout)
    if result != actionlib.GoalStatus.SUCCEEDED:
      rospy.logerr("Goal for "+name+" did not succeed!")
      raise ActionException(client,name,result)
    return client.get_result()
