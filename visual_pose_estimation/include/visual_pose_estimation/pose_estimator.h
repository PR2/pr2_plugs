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

  bool getPlanar() const;
  void setPlanar(bool is_planar);

  // Solve without any prior information
  tf::Pose solve(const std::vector<cv::Point2f>& image_points,
                 const image_geometry::PinholeCameraModel& model) const;

  // Solve with prior in the camera frame
  tf::Pose solveWithPrior(const std::vector<cv::Point2f>& image_points,
                          const image_geometry::PinholeCameraModel& model,
                          const tf::Pose& prior) const;

protected:
  cv::Mat_<cv::Vec3f> object_points_;
  bool use_planar_solve_;

  void solveImpl(const std::vector<cv::Point2f>& image_points,
                 const image_geometry::PinholeCameraModel& model,
                 tf::Pose& pose, bool have_prior) const;
};


inline const cv::Mat_<cv::Vec3f>& PoseEstimator::objectModel() const
{
  return object_points_;
}

inline bool PoseEstimator::getPlanar() const { return use_planar_solve_; }
inline void PoseEstimator::setPlanar(bool is_planar) { use_planar_solve_ = is_planar; }

} //namespace visual_pose_estimation

#endif
