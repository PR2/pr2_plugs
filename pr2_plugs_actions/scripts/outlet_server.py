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
#* Author: Eitan Marder-Eppstein
#***********************************************************

PKG = 'pr2_plugs_actions'

import roslib; roslib.load_manifest(PKG)
import roslib.message
import rospy
import yaml

from pr2_plugs_msgs.srv import GetOutlets, GetOutletsRequest, GetOutletsResponse
from pr2_plugs_msgs.msg import OutletPose

def serve_plug_locations(req):
  rospy.loginfo("Serving plug locations")
  try:
    poses = rospy.get_param('outlet_approach_poses')
  except:
    print 'no poses'
    poses = {}
  if not 'local' in poses:
    poses['local'] = {'position':[0,0,0], 'orientation': [0,0,0,0]}
  print poses

  resp = GetOutletsResponse()
  for pose in poses:
    op = OutletPose()
    op.name = pose
    roslib.message.fill_message_args(op.approach_pose, poses[pose])
    resp.poses.append(op)
  print resp

  return resp

if __name__ == "__main__":
  rospy.init_node("outlet_server")
  s = rospy.Service("outlet_locations", GetOutlets, serve_plug_locations) 
  rospy.spin()
