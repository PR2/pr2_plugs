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

/**
@mainpage

\author Radu Bogdan Rusu

@b wall_extractor attempts to find the best fit plane to a given PointCloud message extracted from a stereo camera.
 **/

#include <boost/thread/mutex.hpp>

// ROS core
#include <ros/node_handle.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
// For the normals visualization
#include <visualization_msgs/Marker.h>

// Cloud geometry
#include <stereo_wall_detection/geometry/angles.h>
#include <stereo_wall_detection/geometry/point.h>
#include <stereo_wall_detection/geometry/areas.h>
#include <stereo_wall_detection/geometry/nearest.h>
#include <stereo_wall_detection/geometry/intersections.h>
#include <Eigen/Geometry>

// Sample Consensus
#include <stereo_wall_detection/sample_consensus/sac.h>
#include <stereo_wall_detection/sample_consensus/msac.h>
#include <stereo_wall_detection/sample_consensus/sac_model_normal_plane.h>

#include <stereo_wall_detection/DetectWall.h>
using namespace std;

class PlanarFit
{
  protected:
    ros::NodeHandle nh_;

  public:
    // ROS messages
    sensor_msgs::PointCloud cloud_msg_;
    boost::mutex cloud_msg_mutex_;
    ros::ServiceServer serv_;
    ros::Publisher plane_normal_pub_;
    ros::Publisher vis_pub_;

    // Parameters
    double radius_;
    int k_;

    // Additional downsampling parameters
    geometry_msgs::Point leaf_width_;

    // If normals_fidelity_, then use the original cloud data to estimate the k-neighborhood and thus the normals
    int normals_fidelity_;

    double sac_distance_threshold_;
    int sac_use_normals_;
    double sac_normal_distance_weight_;	// for finding the plane
    double sac_normal_inlier_distance_weight_; // for selecting inliers

    double max_dist_;
    string cloud_topic_;
    bool subscriber_enabled_, received_cloud_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlanarFit () : nh_ ("~"), cloud_topic_ ("narrow_stereo_textured/points"), subscriber_enabled_(false)
    {
      nh_.param ("search_radius", radius_, 0.03);                      // 3cm radius by default

      nh_.param ("downsample_leaf_width_x", leaf_width_.x, 0.015);     // 1.5cm radius by default
      nh_.param ("downsample_leaf_width_y", leaf_width_.y, 0.015);     // 1.5cm radius by default
      nh_.param ("downsample_leaf_width_z", leaf_width_.z, 0.015);     // 1.5cm radius by default
      nh_.param ("downsample_max_dist", max_dist_, 3.0);              // 3m by default

      nh_.param ("normals_high_fidelity", normals_fidelity_, 1);       // compute the downsampled normals from the original data

      nh_.param ("sac_distance_threshold", sac_distance_threshold_, 0.03);   // 3 cm threshold
      nh_.param ("sac_use_normals", sac_use_normals_, 0);                    // use normals in SAC
      nh_.param ("sac_normal_distance_weight", sac_normal_distance_weight_, 0.05);   // weight normals by .05 in the SAC distance
      nh_.param ("sac_normal_inlier_distance_weight", sac_normal_inlier_distance_weight_, 0.05);   // weight normals by .05 in the SAC inlier retrieval

      ros::NodeHandle nh_toplevel_;
      serv_ = nh_.advertiseService ("detect_wall", &PlanarFit::detect_wall, this);

      // Visualize the plane normal
      vis_pub_ = nh_.advertise<visualization_msgs::Marker> ("visualization_marker", 0);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PlanarFit () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (const sensor_msgs::PointCloud::ConstPtr& cloud)
    {
      if(!subscriber_enabled_)
        return;
      boost::mutex::scoped_lock(cloud_msg_mutex_);
      cloud_msg_ = *cloud;
      received_cloud_ = true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Service trigger
    bool
      detect_wall (stereo_wall_detection::DetectWall::Request &req, stereo_wall_detection::DetectWall::Response &resp)
    {
      received_cloud_ = false;

      // subscribe to point cloud and wait for message to come in
      ros::NodeHandle nh_toplevel;
      ros::Subscriber cloud_sub = nh_toplevel.subscribe<sensor_msgs::PointCloud> (cloud_topic_, 1, &PlanarFit::cloud_cb, this);
      ros::Time start = ros::Time::now();
      bool have_cloud = false;
      subscriber_enabled_ = true;
      sensor_msgs::PointCloud cloud_msg_local;
      while (!have_cloud){
	ros::Duration(0.1).sleep();
        boost::mutex::scoped_lock(cloud_msg_mutex_);
        if (received_cloud_)
        {
          if(cloud_msg_.points.size() >= 50000){
            // make a copy for our own use
            cloud_msg_local = cloud_msg_;
            have_cloud = true;
          }
        }
        else
        {
          if (ros::Time::now() > start + ros::Duration(10.0)){
            if (!received_cloud_)
              ROS_ERROR("Timed out waiting for point cloud");
            else
              ROS_ERROR("Only received a point cloud of size %d", (int)cloud_msg_local.points.size());
            subscriber_enabled_ = false;
            return false;
          }
        }
      }

      subscriber_enabled_ = false;

      ROS_INFO("point cloud of size %u", (unsigned int)(cloud_msg_local.points.size()));
      if (cloud_msg_local.points.size () == 0){
	ROS_ERROR("Received point cloud of size zero");
        return (false);
      }

      sensor_msgs::PointCloud cloud_down;
      cloud_down.header = cloud_msg_local.header;

      ros::Time ts = ros::Time::now ();
      // Figure out the viewpoint value in the cloud_frame frame
      geometry_msgs::PointStamped viewpoint_cloud;
      viewpoint_cloud.point.x = 0; viewpoint_cloud.point.y = 0; viewpoint_cloud.point.z = 0;

      // Downsample addon for points in the stereo frame (z = distance)
      int nrp = 0;
      vector<int> indices_down (cloud_msg_local.points.size ());
      for (size_t i = 0; i < indices_down.size (); ++i)
        if (cloud_msg_local.points[i].z < max_dist_)
          indices_down[nrp++] = i;
      indices_down.resize (nrp);

      if (indices_down.size () == 0){
	ROS_ERROR("No points found in plane");
        return (false);
      }
      try
      {
        // We sacrifice functionality for speed. Use a fixed 3D grid to downsample the data instead of an octree structure
	vector<cloud_geometry::Leaf> leaves;
        cloud_geometry::downsamplePointCloud (cloud_msg_local, indices_down, cloud_down, leaf_width_, leaves, -1);  // -1 means use all data
      }
      catch (std::bad_alloc)
      {
        ROS_ERROR ("Not enough memory to create a simple downsampled representation. Change the downsample_leaf_width parameters.");
        return (false);
      }

      if (normals_fidelity_)
        cloud_geometry::nearest::computePointCloudNormals (cloud_down, cloud_msg_local, radius_, viewpoint_cloud);  // Estimate point normals in the original point cloud using a fixed radius search
      else
        cloud_geometry::nearest::computePointCloudNormals (cloud_down, radius_, viewpoint_cloud);          // Estimate point normals in the downsampled point cloud using a fixed radius search


      // ---[ Fit a planar model
      vector<int> inliers;
      vector<double> coeff;
      if (!fitSACPlane (&cloud_down, inliers, coeff, viewpoint_cloud, sac_distance_threshold_)){
	ROS_ERROR("Failed to fit planar model");
        return (false);
      }

      // Flip normal to be "inside the wall"
      coeff[0] = -coeff[0]; coeff[1] = -coeff[1]; coeff[2] = -coeff[2];
      
      resp.wall_norm.header = cloud_msg_local.header;
      resp.wall_norm.vector.x = coeff[0];  
      resp.wall_norm.vector.y = coeff[1];  
      resp.wall_norm.vector.z = coeff[2];   
   
      geometry_msgs::Point32 centroid;
      cloud_geometry::nearest::computeCentroid (cloud_down, inliers, centroid);

      //ROS_INFO ("Planar model with %d / %d inliers, coefficients: [%g, %g, %g, %g] found in %g seconds.", (int)inliers.size (), (int)cloud_down.points.size (),
      //    coeff[0], coeff[1], coeff[2], coeff[3], (ros::Time::now () - ts).toSec ());

      resp.wall_point.header = cloud_msg_local.header;
      resp.wall_point.point.x = centroid.x;
      resp.wall_point.point.y = centroid.y;
      resp.wall_point.point.z = centroid.z;
      // Convert to quaternion
      Eigen::AngleAxis<float> aa (acos (coeff[0]), Eigen::Vector3f (0, -coeff[2], coeff[1]));
      Eigen::Quaternion<float> q (aa);

      publishNormal (centroid, q, cloud_msg_local.header, 0.1);
      ROS_INFO("Found wall at %f %f %f with normal: %f %f %f in frame %s",
	       centroid.x, centroid.y, centroid.z, coeff[0], coeff[1], coeff[2], cloud_msg_local.header.frame_id.c_str());

      //cloud_msg_.reset ();
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    bool
      fitSACPlane (sensor_msgs::PointCloud *points, vector<int> &inliers, vector<double> &coeff,
                   const geometry_msgs::PointStamped &viewpoint_cloud, double dist_thresh)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model;
      sample_consensus::SACModelNormalPlane *normal_model = new sample_consensus::SACModelNormalPlane ();
	  
      if (sac_use_normals_)
	    {
    	  normal_model->setNormalDistanceWeight(sac_normal_distance_weight_);
    	  model = (sample_consensus::SACModelPlane *) normal_model;
    	}
      else
    	  model = new sample_consensus::SACModelPlane ();

      sample_consensus::SAC *sac = new sample_consensus::MSAC (model, dist_thresh);
      model->setDataSet (points);

      // Search for the best plane
      if (sac->computeModel (0))
      {
        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
    	  // NOTE (kk): can reweight normals for inlier selection
      	normal_model->setNormalDistanceWeight (sac_normal_inlier_distance_weight_);
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->points.at (inliers[0]), viewpoint_cloud);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (false);
      }

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Publish the normals as cute little lines
      * \NOTE: assumes normalized point normals !
      * \param points pointer to the point cloud message
      * \param indices indices of the point cloud normals to publish
      * \param nx_idx the index of the x normal
      * \param ny_idx the index of the y normal
      * \param nz_idx the index of the z normal
      * \param length the length of the normal lines
      * \param width the width of the normal lines
      */
    void
      publishNormal (const geometry_msgs::Point32 &position, Eigen::Quaternion<float> orientation, const roslib::Header &header, double length)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = header.frame_id;

      marker.header.stamp = header.stamp;
      marker.ns = "plane_normal";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = position.x;
      marker.pose.position.y = position.y;
      marker.pose.position.z = position.z;
      marker.pose.orientation.x = orientation.x ();
      marker.pose.orientation.y = orientation.y ();
      marker.pose.orientation.z = orientation.z ();
      marker.pose.orientation.w = orientation.w ();
      marker.scale.x = length;
      marker.scale.y = length;
      marker.scale.z = length;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      vis_pub_.publish (marker);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "wall_extractor");

  PlanarFit p;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return (0);
}
/* ]--- */

