#include "checkerboard_pose_estimation/ros_detector.h"

namespace checkerboard_pose_estimation {

RosDetector::RosDetector(const std::string& name)
  : it_(nh_), name_(name)
{
  // Advertise visualization topics
  display_pub_ = it_.advertise("display_image", 1);
  //marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

bool RosDetector::initFromParameters(const ros::NodeHandle& nh)
{
  bool have_all_required = true;

  int width;
  if (!nh.getParam("board_width", width)) {
    ROS_ERROR("Board width [~board_width] unspecified");
    have_all_required = false;
  }

  int height;
  if (!nh.getParam("board_height", height)) {
    ROS_ERROR("Board height [~board_height] unspecified");
    have_all_required = false;
  }

  double square_size;
  if (!nh.getParam("square_size", square_size)) {
    ROS_ERROR("Square size [~square_size] unspecified");
    have_all_required = false;
  }

  bool subpixel;
  nh.param("subpixel_corners", subpixel, true);

  // Making origin side non-required since it's specified in action goal
  bool origin_on_right;
  nh.param("origin_on_right", origin_on_right, true);

  // Checkerboard frame (looking down on the board, prongs forward): (left, forward, down)
  tf::Transform& plug_in_board = target_in_detected_object_;
  double pos[3], ori[4];
  if (nh.getParam("plug_position_x", pos[0]) &&
      nh.getParam("plug_position_y", pos[1]) &&
      nh.getParam("plug_position_z", pos[2]) &&
      nh.getParam("plug_orientation_x", ori[0]) &&
      nh.getParam("plug_orientation_y", ori[1]) &&
      nh.getParam("plug_orientation_z", ori[2]) &&
      nh.getParam("plug_orientation_w", ori[3]) )
  {
    plug_in_board.getOrigin().setValue(pos[0], pos[1], pos[2]);
    plug_in_board.setRotation(tf::Quaternion(ori[0], ori[1], ori[2], ori[3]));
    ROS_DEBUG("plug_in_board: T(%f,%f,%f), R(%f,%f,%f,%f)",
              pos[0], pos[1], pos[2], ori[0], ori[1], ori[2], ori[3]);
  } else {
    ROS_WARN("Plug in board pose unspecified, setting it to identity");
    plug_in_board.setIdentity();
  }

  if (!have_all_required) return false;

  // Initialize detector
  detector_.setDimensions(width, height);
  detector_.setSubpixel(subpixel);
  detector_.setOriginSide(origin_on_right ? Detector::RIGHT : Detector::LEFT);

  // Initialize pose estimator with checkerboard object model
  pose_estimator_ = createCheckerboardEstimator(width, height, square_size);

  return true;
}

// detected object = checkerboard, target = plug
bool RosDetector::detectObject(const sensor_msgs::ImageConstPtr& image_msg,
                               const sensor_msgs::CameraInfoConstPtr& info_msg,
                               const tf::Stamped<tf::Pose>& target_prior, const tf::Transformer& transformer,
                               tf::Stamped<tf::Pose>& target_pose)
{
  // Convert image message
  if (!img_bridge_.fromImage(*image_msg, "mono8")) {
    ROS_ERROR("%s: Failed to convert image from %s -> mono8", name_.c_str(), image_msg->encoding.c_str());
    return false;
  }
  cv::Mat image(img_bridge_.toIpl());

  // Detect the checkerboard
  std::vector<cv::Point2f> corners;
  if (!detector_.detect(image, corners)) {
    ROS_DEBUG("%s: Failed to detect checkerboard", name_.c_str());
    publishDisplayImage(image, corners, false);
    /// @todo Publish feedback?
    return false;
  }

  // Estimate its pose
  cam_model_.fromCameraInfo(info_msg);
  tf::Stamped<tf::Pose> target_prior_in_camera;
  try {
    transformer.transformPose(cam_model_.tfFrame(), target_prior, target_prior_in_camera);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN("%s: TF exception\n%s", name_.c_str(), ex.what());
    return false;
  }
  tf::Pose detected_object_prior = target_prior_in_camera * target_in_detected_object_.inverse();
  tf::Pose detected_object_pose = pose_estimator_.solveWithPrior(corners, cam_model_, detected_object_prior);
  target_pose = tf::Stamped<tf::Pose>(detected_object_pose * target_in_detected_object_,
                                      image_msg->header.stamp, cam_model_.tfFrame());

  // Publish visualization messages
  tf_broadcaster_.sendTransform(tf::StampedTransform(detected_object_prior, image_msg->header.stamp,
                                                     cam_model_.tfFrame(), "checkerboard_prior_frame"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(target_prior, target_prior.stamp_,
                                                     target_prior.frame_id_, "plug_prior_frame"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(detected_object_pose, image_msg->header.stamp,
                                                     cam_model_.tfFrame(), "checkerboard_frame"));
  tf_broadcaster_.sendTransform(tf::StampedTransform(target_pose, image_msg->header.stamp,
                                                     cam_model_.tfFrame(), "plug_frame"));
  publishDisplayImage(image, corners, true);

  return true;
}

void RosDetector::publishDisplayImage(const cv::Mat& source, const std::vector<cv::Point2f>& corners,
                                      bool success)
{
  if (display_pub_.getNumSubscribers() == 0) return;
  detector_.getDisplayImage(source, corners, success, display_img_cv_);
  IplImage ipl = (IplImage)display_img_cv_;
  sensor_msgs::CvBridge::fromIpltoRosImage(&ipl, display_img_);
  display_img_.encoding = "bgr8";
  display_pub_.publish(display_img_);
}

} //namespace checkerboard_pose_estimation
