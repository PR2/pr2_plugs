/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
* Author: Wim Meeussen
*********************************************************************/


#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kdl/frames.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "detector_stub");
  double x, y, z, Rx, Ry, Rz;
  std::string frame_id;
  ros::NodeHandle node("~");
  node.param("x", x, 0.0);
  node.param("y", y, 0.0);
  node.param("z", z, 0.0);
  node.param("Rx", Rx, 0.0);
  node.param("Ry", Ry, 0.0);
  node.param("Rz", Rz, 0.0);
  node.param("frame_id", frame_id, std::string("frame_id_not_specified"));

  ros::NodeHandle node_top;
  ros::Publisher pub_pose = node_top.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose", 10);
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = z;
  KDL::Rotation tmp = KDL::Rotation::RPY(Rx, Ry, Rz);
  double Qx, Qy, Qz, Qw;
  tmp.GetQuaternion(Qx, Qy, Qz, Qw);
  pose.pose.pose.orientation.x = Qx;
  pose.pose.pose.orientation.y = Qy;
  pose.pose.pose.orientation.z = Qz;
  pose.pose.pose.orientation.w = Qw;

  double cov[36] =  {0.0001, 0, 0, 0, 0, 0,
                     0, 0.0001, 0, 0, 0, 0,
                     0, 0, 0.0001, 0, 0, 0,
                     0, 0, 0, 0.0001, 0, 0,
                     0, 0, 0, 0, 0.0001, 0,
                     0, 0, 0, 0, 0, 0.0001};
  for (unsigned int i=0; i<36; i++)
    pose.pose.covariance[i] = cov[i];

  while (ros::ok()){
    pose.header.stamp = ros::Time::now();
    pub_pose.publish(pose);
    ros::Duration(0.4).sleep();
  }
  return 0;
}
