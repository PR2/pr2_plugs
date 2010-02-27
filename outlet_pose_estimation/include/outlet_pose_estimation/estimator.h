#ifndef OUTLET_POSE_ESTIMATION_ESTIMATOR_H
#define OUTLET_POSE_ESTIMATION_ESTIMATOR_H

#include <visual_pose_estimation/pose_estimator.h>
#include "outlet_pose_estimation/detail/outlet_tuple.h"

namespace outlet_pose_estimation {

visual_pose_estimation::PoseEstimator createOutletEstimator(const outlet_template_t& _template);

} //namespace outlet_pose_estimation

#endif
