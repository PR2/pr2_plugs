#ifndef VISUAL_POSE_ESTIMATION_POSE_ESTIMATOR_H
#define VISUAL_POSE_ESTIMATION_POSE_ESTIMATOR_H

#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>

namespace visual_pose_estimation {

class PoseEstimator
{
public:
  PoseEstimator();

  PoseEstimator(const cv::Mat& object_points);

  const cv::Mat_<cv::Vec3f>& objectModel() const;

  void setObjectModel(const cv::Mat& object_points);

  // Solve without any prior information
  tf::Pose solve(const std::vector<cv::Point2f>& image_points,
                 const image_geometry::PinholeCameraModel& model) const;

  // Solve with prior in the camera frame
  tf::Pose solveWithPrior(const std::vector<cv::Point2f>& image_points,
                          const image_geometry::PinholeCameraModel& model,
                          const tf::Pose& prior) const;

  // Solve, prior can be in any frame
  tf::Pose solveWithPrior(const std::vector<cv::Point2f>& image_points,
                          const image_geometry::PinholeCameraModel& model,
                          const tf::Stamped<tf::Pose>& prior,
                          const tf::Transformer& transformer) const;

protected:
  cv::Mat_<cv::Vec3f> object_points_;

  void solveImpl(const std::vector<cv::Point2f>& image_points,
                 const image_geometry::PinholeCameraModel& model,
                 tf::Pose& pose, bool have_prior) const;
};


inline const cv::Mat_<cv::Vec3f>& PoseEstimator::objectModel() const
{
  return object_points_;
}

} //namespace visual_pose_estimation

#endif
