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
#
# Author Melonee Wise


import roslib; roslib.load_manifest('pr2_plugs_actions')
import rospy
import actionlib

from pr2_controllers_msgs.msg import *
from pr2_plugs_msgs.msg import *
from actionlib_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import *
import dynamic_reconfigure.client
from stereo_wall_detection.srv import *


class DetectWallNormServer:
  def __init__(self, name):
    self.name = name
    self.sim = rospy.get_param('~sim', False)

    self.head_client = actionlib.SimpleActionClient('head_traj_controller/point_head_action', PointHeadAction)
    self.head_client.wait_for_server()

    rospy.wait_for_service('stereo_wall_detection/detect_wall')
    rospy.loginfo('detect wall service found')
    self.detect_wall_srv = rospy.ServiceProxy('stereo_wall_detection/detect_wall', DetectWall)
    self.server = actionlib.simple_action_server.SimpleActionServer(self.name, DetectWallNormAction, self.execute_cb)

    if(not self.sim):
      self.projector_client = dynamic_reconfigure.client.Client('camera_synchronizer_node')
      self.projector_on = {'narrow_stereo_trig_mode': 3}
      self.projector_off = {'narrow_stereo_trig_mode': 4}
      self.projector_sub = rospy.Subscriber("projector_controller/rising_edge_timestamps", roslib.msg.Header, self.projector_cb)



  def execute_cb(self, goal):
    rospy.loginfo('detect wall norm')

    #point head at wall
    point_head_goal = PointHeadGoal()
    point_head_goal.target.header.frame_id = goal.look_point.header.frame_id
    point_head_goal.target.point.x = goal.look_point.point.x
    point_head_goal.target.point.y = goal.look_point.point.y
    point_head_goal.target.point.z = goal.look_point.point.z
    point_head_goal.pointing_frame = 'head_pan_link'
    point_head_goal.pointing_axis.x = 1
    point_head_goal.pointing_axis.y = 0
    point_head_goal.pointing_axis.z = 0

    self.head_client.send_goal(point_head_goal)
    self.head_client.wait_for_result(rospy.Duration(20.0))
    if self.head_client.get_state() != GoalStatus.SUCCEEDED:
      rospy.logerr('%s: Failed to move the head', self.name)
      self.server.set_aborted()
      return

    # turn on projector
    if(not self.sim):
      rospy.loginfo('turn on projector')
      self.projector_ready = False
      while not self.turn_projector_on():
        rospy.loginfo('still trying to turn on projector')
        rospy.sleep(1.0)
      while not self.projector_ready:
        rospy.sleep(1.0)
        rospy.loginfo('waiting for projector to be ready')
      rospy.loginfo('projector truned on')

    # detect wall norm
    try:
      rospy.loginfo('Call stereo wall detector')
      wall = self.detect_wall_srv(DetectWallRequest())
      rospy.loginfo('Call stereo wall detector succeeded')
    except rospy.ServiceException, e:
      rospy.logerr("Service call to wall detector failed")
      if(not self.sim):
        while not self.turn_projector_off():
          rospy.loginfo('still trying to turn off projector')
          rospy.sleep(1.0)
      self.server.set_aborted()
      return

    # turn off projector
    if(not self.sim):
      rospy.loginfo('turn off projector')
      while not self.turn_projector_off():
        rospy.loginfo('still trying to turn off projector')
        rospy.sleep(1.0)
    result = DetectWallNormResult()
    result.wall_norm = wall.wall_norm
    result.wall_point = wall.wall_point
    self.server.set_succeeded(result)

  def projector_cb(self, data):
    if(not self.server.is_active()):
      self.projector_ready = False
      return

    self.projector_ready = True

  def turn_projector_on(self):
    try:
      self.projector_client.update_configuration(self.projector_on)
      return True
    except rospy.ServiceException, e:
      rospy.logerr('Failed to turn projector on')
      return False

  def turn_projector_off(self):
    try:
      self.projector_client.update_configuration(self.projector_off)
      return True
    except rospy.ServiceException, e:
      rospy.logerr('Failed to turn projector off')
      return False



if __name__ == '__main__':

  # Initializes a rospy node so that the SimpleActionClient can
  # publish and subscribe over ROS.
  name ='detect_wall_norm'
  rospy.init_node(name)
  detect_wall_norm_server = DetectWallNormServer(name)
  rospy.loginfo('%s: Action server running', name)
  rospy.spin()
