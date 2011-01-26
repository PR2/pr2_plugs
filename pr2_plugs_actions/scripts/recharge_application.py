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
from random import choice

from pr2_plugs_msgs.msg import EmptyAction, RechargeAction, RechargeGoal, RechargeCommand, RechargeState

class Application():
    def __init__(self):
        self.charging_locations = rospy.get_param("~charging_locations", ['wim', 'whiteboard', 'james'])

        self.recharge = actionlib.SimpleActionClient('recharge', RechargeAction)        
        self.recharge.wait_for_server()

        self.app_action = actionlib.SimpleActionServer('recharge_application_server', EmptyAction, self.application_cb) 

    def application_cb(self, goal):
        rospy.loginfo('Recharge application became active.')    
        # first plug in robot
        goal = RechargeGoal()
        goal.command.command = RechargeCommand.PLUG_IN
        while not rospy.is_shutdown() and self.recharge.send_goal_and_wait(goal) != actionlib.GoalStatus.SUCCEEDED:
            goal.command.plug_id = choice(self.charging_locations)
            rospy.sleep(0.1)
            if self.app_action.is_preempt_requested():
                self.app_action.set_preempted()
                return
            
        # check for preemption while plugged in
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.app_action.is_preempt_requested():
                break

        # unplug robot
        goal = RechargeGoal()
        goal.command.command = RechargeCommand.UNPLUG
        if self.recharge.send_goal_and_wait(goal) == actionlib.GoalStatus.SUCCEEDED:
            self.app_action.set_preempted()
        else:
            self.app_action.set_aborted()





def main():
    rospy.init_node("recharge_application_monitor")

    a = Application()
    rospy.spin()
    

if __name__ == "__main__":
    main()

