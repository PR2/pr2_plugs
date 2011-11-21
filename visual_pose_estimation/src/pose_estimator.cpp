#include "visual_pose_estimation/pose_estimator.h"
#include "visual_pose_estimation/planar.h"

namespace visual_pose_estimation {

PoseEstimator::PoseEstimator()
  : use_planar_solve_(false)
{
}

PoseEstimator::PoseEstimator(const cv::Mat& object_points)
  : object_points_(object_points),
    use_planar_solve_(false)
{
}

void PoseEstimator::setObjectModel(const cv::Mat& object_points)
{
  object_points_ = object_points;
}

tf::Pose PoseEstimator::solve(const std::vector<cv::Point2f>& image_points,
                              const image_geometry::PinholeCameraModel& model) const
{
  tf::Pose pose;
  solveImpl(image_points, model, pose, false);
  return pose;
}

tf::Pose PoseEstimator::solveWithPrior(const std::vector<cv::Point2f>& image_points,
                                       const image_geometry::PinholeCameraModel& model,
                                       const tf::Pose& prior) const
{
  tf::Pose pose = prior;
  solveImpl(image_points, model, pose, true);
  return pose;
}

void PoseEstimator::solveImpl(const std::vector<cv::Point2f>& image_points,
                              const image_geometry::PinholeCameraModel& model,
                              tf::Pose& pose, bool have_prior) const
{
  // Set up buffers
  double R3_buffer[3], T3_buffer[3], rot3x3_buffer[9];
  double D_buffer[4] = {0}; // Assume image already rectified, so zero distortion.
  cv::Mat_<double> R3(3, 1, R3_buffer);
  cv::Mat_<double> T3(3, 1, T3_buffer);
  cv::Mat_<double> D(1, 4, D_buffer);
  cv::Mat_<cv::Vec2f> image_pts_cv(object_points_.rows, 1, (cv::Vec2f*)(&image_points[0]));
  cv::Mat_<double> rot3x3(3, 3, rot3x3_buffer);

  if (have_prior) {
    T3(0,0) = pose.getOrigin().x();
    T3(1,0) = pose.getOrigin().y();
    T3(2,0) = pose.getOrigin().z();
    
    // Convert to Rodrigues rotation
    btMatrix3x3 &basis = pose.getBasis();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        rot3x3(i,j) = basis[i][j];
    cv::Rodrigues(rot3x3, R3);
    ROS_DEBUG("Prior pose: T(%.3f, %.3f, %.3f), R(%.3f, %.3f, %.3f)",
              T3(0,0), T3(1,0), T3(2,0), R3(0,0), R3(1,0), R3(2,0));
  }

  // First three columns of projection matrix
  cv::Mat_<double> projection(3,3);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      projection(i,j) = model.projectionMatrix()(i,j);
    }
  }

  // Find/refine object pose
  if (use_planar_solve_)
    cv::solvePlanarPnP(object_points_, image_pts_cv, projection, D, R3, T3, have_prior);
  else
    cv::solvePnP(object_points_, image_pts_cv, projection, D, R3, T3, have_prior);
  
  ROS_DEBUG("Refined pose: T(%.3f, %.3f, %.3f), R(%.3f, %.3f, %.3f)",
            T3(0,0), T3(1,0), T3(2,0), R3(0,0), R3(1,0), R3(2,0));
  pose.getOrigin().setValue(T3(0,0), T3(1,0), T3(2,0));

  // Convert from Rodrigues to rotation matrix
  cv::Rodrigues(R3, rot3x3);
  btMatrix3x3 rot3x3_tf(rot3x3(0,0), rot3x3(0,1), rot3x3(0,2),
                        rot3x3(1,0), rot3x3(1,1), rot3x3(1,2),
                        rot3x3(2,0), rot3x3(2,1), rot3x3(2,2));
  pose.setBasis(rot3x3_tf);
}

} //namespace visual_pose_estimation
