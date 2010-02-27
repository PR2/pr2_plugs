#ifndef CHECKERBOARD_POSE_ESTIMATION_ESTIMATOR_H
#define CHECKERBOARD_POSE_ESTIMATION_ESTIMATOR_H

#include <visual_pose_estimation/pose_estimator.h>

namespace checkerboard_pose_estimation {

visual_pose_estimation::PoseEstimator createCheckerboardEstimator(int width, int height, float square_size);

} //namespace checkerboard_pose_estimation

#endif
