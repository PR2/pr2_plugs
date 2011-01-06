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
#    * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above
#        copyright notice, this list of conditions and the following
#        disclaimer in the documentation and/or other materials provided
#        with the distribution.
#    * Neither the name of the Willow Garage nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
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
roslib.load_manifest('pr2_plugs_actions')

import rospy
import actionlib
import threading

from pr2_plugs_msgs.msg import EmptyAction, RechargeAction, RechargeGoal, RechargeCommand


class Monitor():
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('recharge', RechargeAction)        
        self.ac.wait_for_server()
        self.state = State()

        self.app_action = actionlib.SimpleActionServer('recharge_application_monitor', EmptyAction, self.application_cb) 
        self.app_action = actionlib.SimpleActionServer('recharge_application', RechargeAction, self.recharge_cb) 


    def recharge_cb(self, goal):
        if self.state.start_working():
            ac.send_goal_and_wait(goal)
            res = ac.get_result()
            if res.state == RechargeState.UNPLUGGED:
                self.state.set_state('Unplugged')
            if res.state == RechargeState.PLUGGED_IN:
                self.state.set_state('Plugged')

        

    def application_cb(self, goal):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.app_action.is_preempt_requested():	
                self.state.stop_working()
                if self.state.get_state() == 'Plugged':
                    unplug_goal = RechargeGoal()
                    unplug_goal.command = RechargeCommand.UNPLUG
                    self.ac.send_goal_and_wait(unplug_goal)
                return
                

class State():
    def __init__(self):
        self.state = 'Unplugged'
        self.preempting = False
        self.lock = threading.Lock()  
    
    def set_state(state):
        with self.lock:
            self.state = state

    def get_state():
        with self.lock:
            return self.state

    def stop_working():
        with self.lock:
            self.preempting = True
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            with self.lock:
                if self.state != 'Working':
                    return

    def start_working():
        with self.lock:
            if not self.preempting:
                self.state = 'Working'
                return True
            else:
                return False



def main():
    rospy.init_node("recharge_application_monitor")

    m = Monitor()
    rospy.spin()
    

if __name__ == "__main__":
    main()

