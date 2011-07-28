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
 * author: Wim Meeussen
 *
 */




#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_wall_detection/DetectWall.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/io.h>
#include <pcl/features/feature.h>
#include <pcl/point_types.h>



class PlanarFit
{
public:
  PlanarFit () : nh_ ("~")
  {
    serv_ = nh_.advertiseService ("detect_wall", &PlanarFit::detect_wall, this);
  }
  
  virtual ~PlanarFit () { }
  
  bool detect_wall (stereo_wall_detection::DetectWall::Request &req, stereo_wall_detection::DetectWall::Response &resp)
  {
    // get pointcloud
    sensor_msgs::PointCloud2ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("narrow_stereo_textured/points", nh_, ros::Duration(10.0));
    if (!cloud_msg){
      ROS_ERROR("Did not receive a pointcloud in 10 seconds");
      return false;
    }
    ROS_INFO("point cloud received");

    // find plane in pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud.makeShared ());
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
      return false;
    }
    pcl::copyPointCloud(cloud, *inliers, cloud);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);

    resp.wall_norm.header = cloud_msg->header;
    resp.wall_norm.vector.x = coefficients->values[0];
    resp.wall_norm.vector.y = coefficients->values[1];
    resp.wall_norm.vector.z = coefficients->values[2];

    resp.wall_point.header = cloud_msg->header;
    resp.wall_point.point.x = centroid[0];
    resp.wall_point.point.y = centroid[1];
    resp.wall_point.point.z = centroid[2];

    ROS_INFO("Found wall at %f %f %f with normal: %f %f %f in frame %s",
             resp.wall_point.point.x, resp.wall_point.point.y, resp.wall_point.point.z,
             resp.wall_norm.vector.x, resp.wall_norm.vector.y, resp.wall_norm.vector.z, 
             cloud_msg->header.frame_id.c_str());

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer serv_;
};



int main (int argc, char** argv)
{
  ros::init (argc, argv, "wall_extractor");

  PlanarFit p;
  ros::spin();

  return (0);
}

