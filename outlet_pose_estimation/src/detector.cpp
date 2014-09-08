#include "outlet_pose_estimation/detector.h"

namespace outlet_pose_estimation {

Detector::Detector()
{
}

bool Detector::loadTemplate(const std::string& filename)
{
  return outlet_template_.load(filename.c_str());
}

bool Detector::detect(const cv::Mat& image, std::vector<cv::Point2f>& points) const
{
  IplImage ipl = (IplImage)image;

  // Detect the outlet holes
  // Current code path does not depend on the camera parameters, so we leave them NULL.
  if (!outlet_template_.get_one_way_descriptor_base() ||
      !detect_outlet_tuple(&ipl, NULL, NULL, outlets_, outlet_template_, NULL, NULL))
    return false;

  // Retrieve the image coordinates of the holes
  std::vector<bool> is_detected;
  getImagePoints(outlets_, points, is_detected);
  return true;
}

void Detector::getDisplayImage(const cv::Mat& source,
                               const std::vector<cv::Point2f>& points,
                               bool success, cv::Mat& display) const
{
  /// @todo Currently assuming source is color
  display = source.clone();
  if (success) {
    IplImage display_ipl = (IplImage)display;
    /// @todo Cheating and reusing outlets_ from detect()
    draw_outlets(&display_ipl, outlets_);
  }
}

} //namespace outlet_pose_estimation
