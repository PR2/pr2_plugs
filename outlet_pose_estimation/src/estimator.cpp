#include "outlet_pose_estimation/estimator.h"
#include <cassert>

namespace outlet_pose_estimation {

visual_pose_estimation::PoseEstimator createOutletEstimator(const outlet_template_t& _template)
{
  std::vector<cv::Point3f> pts;
  _template.get_holes_3d(pts);
  assert(pts.size() == 6);
  
  cv::Mat_<cv::Vec3f> hole_points(pts.size(), 1);
  cv::Point3f origin = pts[5]; // top ground hole
  for (size_t i = 0; i < pts.size(); ++i) {
    // Translate each point so the top ground hole is the origin
    cv::Point3f pt = pts[i] - origin;
    // Rotate into outlet frame and convert meters->mm
    hole_points(i, 0) = cv::Vec3f(pt.z * 1e-3f, pt.x * 1e-3f, pt.y * 1e-3f);
  }

  return visual_pose_estimation::PoseEstimator(hole_points);
}

} //namespace outlet_pose_estimation
