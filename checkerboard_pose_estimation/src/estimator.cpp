#include "checkerboard_pose_estimation/estimator.h"

namespace checkerboard_pose_estimation {

visual_pose_estimation::PoseEstimator createCheckerboardEstimator(int width, int height, float square_size)
{
  cv::Mat_<cv::Vec3f> grid_points(width*height, 1);
  int j = 0;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      grid_points(j, 0) = cv::Vec3f(x*square_size, y*square_size, 0.0f);
      ++j;
    }
  }

  return visual_pose_estimation::PoseEstimator(grid_points);
}

} //namespace checkerboard_pose_estimation
