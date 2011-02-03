#! /usr/bin/python
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
#* Author: Wim Meeussen
#***********************************************************
from __future__ import with_statement

import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import threading
import copy
from std_msgs.msg import Bool
from pr2_msgs.msg import PowerState
from collections import deque


class Battery:
    def __init__(self, low_cb=None, high_cb=None, name='battery_monitor'):
        self.lock = threading.Lock()  
        self.state = None
        self.low_trigger = 0
        self.high_trigger = 100
        self.low_cb = low_cb
        self.high_cb = high_cb
        self.buffer_size = rospy.get_param("~%s/buffer_size"%name, 5)
        self.msgs = deque()
        self.power_sub = rospy.Subscriber('power_state', PowerState, self._power_cb)
        rospy.loginfo("Start monitoring battery")

    def set_thresholds(self, low=None, high=None):
        with self.lock:
            if low:
                self.low_trigger = low
            if high:
                self.high_trigger = high
            self.state = None
            rospy.loginfo("Set battery thresholds to %f and %f"%(self.low_trigger, self.high_trigger))


    def get_state(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            with self.lock:
                if self.state:
                    return copy.deepcopy(self.state)



    def _power_cb(self, msg):
        with self.lock:
            self.msgs.append(msg)

            # make sure that the queue doesn't go over length
            while len(self.msgs) > self.buffer_size:
                self.msgs.popleft()

            # dont't do anything if buffer is not filled up
            if not len(self.msgs) >= self.buffer_size:
                return 

            # check for low/high transitions
            state = 0
            for msg in self.msgs:
                if msg.relative_capacity <= self.low_trigger:
                    state -= 1
                elif msg.relative_capacity >= self.high_trigger:
                    state += 1

            # send high trigger only if ac is plugged in
            if self.high_cb and state == self.buffer_size and self.msgs[-1].AC_present == 1:
                self.msgs = deque()  # clear queue to avoid triggering again in the next callback
                if self.state != 'high':
                    rospy.loginfo('Calling high battery callback')
                    self.high_cb()
                self.state = 'high'

            # send low trigger only if ac is not plugged in
            elif self.low_cb and state == -self.buffer_size and self.msgs[-1].AC_present != 1:
                self.msgs = deque()  # clear queue to avoid triggering again in the next callback
                if self.state != 'low':
                    rospy.loginfo('Calling low battery callback')
                    self.low_cb()
                self.state = 'low'

            elif self.msgs[-1].AC_present == 1:
                self.state = 'charging'

            elif self.msgs[-1].AC_present != 1:
                self.state = 'discharging'



