/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <ros/ros.h>
#include "stereo_wall_detection/DetectWall.h"

using namespace stereo_wall_detection;

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "stereo_wall_trigger", ros::init_options::AnonymousName);

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<DetectWall>("stereo_wall_detection/detect_wall");

  DetectWall srv;
  if (client.call (srv))
  {
    ROS_INFO ("Service call successful. Wall detected at: [%g,%g,%g] with norm [%g,%g,%g].", 
        srv.response.wall_point.point.x, 
        srv.response.wall_point.point.y, 
        srv.response.wall_point.point.z, 
        srv.response.wall_norm.vector.x, 
        srv.response.wall_norm.vector.y, 
              srv.response.wall_norm.vector.z);
  }
  else
  {
    ROS_ERROR ("Failed to call service!");
    return (-1);
  }

  return (0);
}
