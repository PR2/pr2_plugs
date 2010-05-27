#! /usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*
#* Author: Eitan Marder-Eppstein
#***********************************************************
PKG = 'pr2_plugs_executive'
NAME = 'dangerous_auto_restarter'
import roslib; roslib.load_manifest(PKG)
import rospy

import std_msgs.msg
import std_srvs.srv

MAX_RESETS = 1000

class AutoRestarter:
  def __init__(self):
    self.resets = 0
    self.really_halted = False
    
    rospy.wait_for_service('pr2_etherCAT/reset_motors')
    rospy.wait_for_service('pr2_etherCAT/halt_motors')

    self.sub_halted    = rospy.Subscriber("pr2_etherCAT/motors_halted", std_msgs.msg.Bool, self.motors_halted_callback)

    self.reset_service_proxy = rospy.ServiceProxy('pr2_etherCAT/reset_motors', std_srvs.srv.Empty)
    self.halt_service_proxy  = rospy.ServiceProxy('pr2_etherCAT/halt_motors', std_srvs.srv.Empty)

    self.reset_motors_service = rospy.Service('~/reset_motors', std_srvs.srv.Empty, self.handle_reset_motors)
    self.halt_motors_service = rospy.Service('~/halt_motors',  std_srvs.srv.Empty, self.handle_halt_motors)

  def handle_reset_motors(self, req):
    self.resets = 0
    self.really_halted = False
    self.do_reset_motors()
    return std_srvs.srv.EmptyResponse()

  def handle_halt_motors(self, req):
    self.resets = MAX_RESETS
    self.really_halted = True
    self.do_halt_motors()
    return std_srvs.srv.EmptyResponse()
    
  def motors_halted_callback(self, msg):
    if msg.data:
      if not self.really_halted:
        if self.resets < MAX_RESETS:
          rospy.logwarn("Motors halted!")
        self.resets += 1
        self.do_reset_motors()
      else:
        rospy.logerr("Maximum resets exceeded -- not resetting motors")

  def do_reset_motors(self):
    rospy.loginfo("Resetting motors.")
    try:
      resp = self.reset_service_proxy()
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def do_halt_motors(self):
    rospy.loginfo("Halting motors.")
    try:
      resp = self.halt_service_proxy()
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    

if __name__ == '__main__':
  rospy.init_node(NAME, anonymous=False)
  ar = AutoRestarter()
  rospy.spin()
