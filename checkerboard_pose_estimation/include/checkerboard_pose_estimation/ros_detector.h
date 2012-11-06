#ifndef CHECKERBOARD_POSE_ESTIMATION_ROS_DETECTOR_H
#define CHECKERBOARD_POSE_ESTIMATION_ROS_DETECTOR_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <checkerboard_pose_estimation/detector.h>
#include <checkerboard_pose_estimation/estimator.h>

namespace checkerboard_pose_estimation {

class RosDetector
{
public:
  RosDetector(const std::string& name = ros::this_node::getName());

  bool initFromParameters(const ros::NodeHandle& nh = ros::NodeHandle("~"));

  Detector& getDetector() { return detector_; }
  const Detector& getDetector() const { return detector_; }
  
  visual_pose_estimation::PoseEstimator& getPoseEstimator() { return pose_estimator_; }
  const visual_pose_estimation::PoseEstimator& getPoseEstimator() const { return pose_estimator_; }

  // detected object = checkerboard, target = plug
  bool detectObject(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg,
                    const tf::Stamped<tf::Pose>& target_prior, const tf::Transformer& transformer,
                    tf::Stamped<tf::Pose>& target_pose);

  void publishDisplayImage(const cv::Mat& source, const std::vector<cv::Point2f>& corners, bool success);
  
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::string name_;

  // Publications
  tf::TransformBroadcaster tf_broadcaster_;
  image_transport::Publisher display_pub_;
  sensor_msgs::Image display_img_;
  cv::Mat display_img_cv_;
  ros::Publisher marker_pub_;

  // Message processing
  image_geometry::PinholeCameraModel cam_model_;
  Detector detector_;
  visual_pose_estimation::PoseEstimator pose_estimator_;
  /// @todo Change behavior depending on whether target frame differs from detected object
  tf::Transform target_in_detected_object_;
  tf::Stamped<tf::Pose> target_prior_;
};

} //namespace checkerboard_pose_estimation

#endif
