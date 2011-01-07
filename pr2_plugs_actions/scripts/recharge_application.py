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

from pr2_plugs_msgs.msg import EmptyAction, RechargeAction, RechargeGoal, RechargeCommand, RechargeState


class Monitor():
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('recharge', RechargeAction)        
        self.ac.wait_for_server()
        self.state = State()

        self.app_action = actionlib.SimpleActionServer('recharge_application_monitor', EmptyAction, self.application_cb) 
        self.recharge_action = actionlib.SimpleActionServer('recharge_application', RechargeAction, self.recharge_cb) 


    def recharge_cb(self, goal):
        if goal.command.command == RechargeCommand.PLUG_IN and self.state.get_state() != 'Unplugged':
            rospy.logerr('Cannot plug in when robot is not unplugged. Not passing through command.')
            self.recharge_action.set_aborted()
            return
        if goal.command.command == RechargeCommand.UNPLUG and self.state.get_state() != 'Plugged':
            rospy.logerr('Cannot unplug when robot is not plugged in. Not passing through command.')
            self.recharge_action.set_aborted()
            return

        original_state = self.state.get_state()
        if self.state.start_working():
            rospy.loginfo('Passing through recharge goal')
            if self.ac.send_goal_and_wait(goal) == actionlib.GoalStatus.SUCCEEDED:
                if goal.command.command == RechargeCommand.UNPLUG:
                    rospy.logerr('New state: Unplugged')
                    self.state.set_state('Unplugged')
                else:
                    rospy.logerr('New state: Plugged in')
                    self.state.set_state('Plugged')
                self.recharge_action.set_succeeded()
            else:
                self.state.set_state(original_state)
                rospy.logerr('Reverting to old state %s'%original_state)
                self.recharge_action.set_aborted()
        


    def application_cb(self, goal):
        rospy.loginfo('Recharge application became active.')    
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.app_action.is_preempt_requested():	
                rospy.loginfo('Preempt requested for recharge application')
                self.state.stop_working()
                rospy.loginfo('Unplugging if needed')
                if self.state.get_state() == 'Plugged':
                    rospy.loginfo('Unplugging robot to complete preemption of recharge application.')
                    unplug_goal = RechargeGoal()
                    unplug_goal.command.command = RechargeCommand.UNPLUG
                    self.ac.send_goal_and_wait(unplug_goal)
                self.app_action.set_succeeded(self.ac.get_result())
                return
                

class State():
    def __init__(self):
        self.state = 'Unplugged'
        self.preempting = False
        self.lock = threading.Lock()  
    
    def set_state(self, state):
        with self.lock:
            self.state = state

    def get_state(self):
        with self.lock:
            return self.state

    def stop_working(self):
        with self.lock:
            self.preempting = True
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            with self.lock:
                if self.state != 'Working':
                    return

    def start_working(self):
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

