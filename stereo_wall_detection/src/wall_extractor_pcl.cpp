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
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/bind.hpp>

#include <visualization_msgs/Marker.h>

class PlanarFit
{
public:
  PlanarFit () : nh_ ("~")
  {
    serv_ = nh_.advertiseService ("detect_wall", &PlanarFit::detect_wall, this);

    // subscribe to pointcloud messages from stereo camera
    //cloud_sub_ = nh_.subscribe("points2", 10, boost::bind(&PlanarFit::cloudCb, this, _1));  // what's wrong with this line???
    cloud_sub_ = nh_.subscribe("points2", 10, &PlanarFit::cloudCb, this);

    // advertise selected points from wall detection algorithm
    plane_points_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 10);
    plane_marker_ = nh_.advertise<visualization_msgs::Marker>("wall_marker", 10);
  }
  
  virtual ~PlanarFit () { }
  


  //void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud)
  void cloudCb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud)
  {
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_msg_ = cloud;
    cloud_condition_.notify_all();
  }



  bool detect_wall (stereo_wall_detection::DetectWall::Request &req, stereo_wall_detection::DetectWall::Response &resp)
  {
    ros::Time start_time = ros::Time::now();

    // wait for new pointcloud
    boost::mutex::scoped_lock lock(cloud_mutex_);
    bool cloud_found = false;
    while (ros::ok() && !cloud_found){
      // check for timeout
      if (ros::Time::now() > start_time + ros::Duration(10.0)){
        ROS_ERROR("Did not receive a pointcloud in 10 seconds");
        return false;
      }

      // check if this is a good pointcloud
      if (cloud_msg_->header.stamp < start_time){
        unsigned count = 0;
        for (unsigned i=0; i<cloud_msg_->height * cloud_msg_->width; i++){
          if (!std::isnan(cloud_msg_->data[i])){
            count++;
          }
        }
        if (count > 30000)
          cloud_found = true;
        else
          ROS_INFO("Received a cloud, but it only had %d points", count);
      }

      // wait for another cloud
      if (!cloud_found)     
        cloud_condition_.wait(lock);
    }
    ROS_INFO("point cloud received");


    double threshold;
    nh_.param<double>("distance_threshold", threshold, 0.01);

    int tmp_points;
    // minimum number of inliers; a good detection appears to have over 
    // 100,000 inliers
    nh_.param<int>("minimum_points", tmp_points, 20000);
    size_t min_points = tmp_points;

    // find plane in pointcloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg_, cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (threshold);
    seg.setInputCloud (cloud.makeShared ());
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) {
      ROS_ERROR ("Could not estimate a planar model for the given dataset.");
      return false;
    }

    // check for minimum number of inliers in plane; if we have too few, our
    // detection is bad and we should return an error
    if( inliers->indices.size() < min_points ) {
       ROS_ERROR("Too few inliers in deteted plane: %zd", inliers->indices.size());
       return false;
    }

    pcl::copyPointCloud(cloud, *inliers, cloud); // add publisher here to see resulting point cloud.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);

    // publish cloud with detected points for debugging
    if( plane_points_.getNumSubscribers() > 0 ) {
      sensor_msgs::PointCloud2 plane_msg;
      pcl::toROSMsg(cloud, plane_msg); 
      plane_points_.publish(plane_msg);
    }

    geometry_msgs::Vector3 normal;
    normal.x = coefficients->values[0];
    normal.y = coefficients->values[1];
    normal.z = coefficients->values[2];

    if( normal.z < 0 ) {
      // Normal points towards camera; reverse it
      ROS_WARN("Wall Normal points towards camera frame: %s", 
        cloud_msg_->header.frame_id.c_str());
      normal.x = -normal.x;
      normal.y = -normal.y;
      normal.z = -normal.z;
    } else {
      ROS_WARN("Wall normal points away from camera frame: %s", 
        cloud_msg_->header.frame_id.c_str());
    }

    resp.wall_norm.header = cloud_msg_->header;
    resp.wall_norm.vector = normal;

    resp.wall_point.header = cloud_msg_->header;
    resp.wall_point.point.x = centroid[0];
    resp.wall_point.point.y = centroid[1];
    resp.wall_point.point.z = centroid[2];


    // publish Marker vector of wall normal for debugging
    if( plane_marker_.getNumSubscribers() > 0 ) {
      visualization_msgs::Marker marker;

      // set up marker
      marker.header = cloud_msg_->header;
      marker.id = 0;
      marker.type =   visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD; // add/modify

      // visible, red marker
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      geometry_msgs::Point end;   // vector end
      end.x = resp.wall_point.point.x + resp.wall_norm.vector.x;
      end.y = resp.wall_point.point.y + resp.wall_norm.vector.y;
      end.z = resp.wall_point.point.z + resp.wall_norm.vector.z;

      marker.points.push_back(resp.wall_point.point);
      marker.points.push_back(end);

      marker.scale.x = 0.05;
      marker.scale.y = 0.1;
      //marker.scale.z = 0.1;

      plane_marker_.publish(marker);
    }

    ROS_INFO("Found wall at %f %f %f with normal: %f %f %f in frame %s",
             resp.wall_point.point.x, resp.wall_point.point.y, resp.wall_point.point.z,
             resp.wall_norm.vector.x, resp.wall_norm.vector.y, resp.wall_norm.vector.z, 
             cloud_msg_->header.frame_id.c_str());

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer serv_;
  ros::Publisher plane_points_;
  ros::Publisher plane_marker_;
  ros::Subscriber cloud_sub_;

  sensor_msgs::PointCloud2ConstPtr cloud_msg_;
  boost::condition cloud_condition_;
  boost::mutex cloud_mutex_;
};



int main (int argc, char** argv)
{
  ros::init (argc, argv, "wall_extractor");

  PlanarFit p;
  ros::MultiThreadedSpinner spinner(2);  // extra thread so we can receive cloud messages while in the service call
  spinner.spin();

  return (0);
}

