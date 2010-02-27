#ifndef OUTLET_POSE_ESTIMATION_DETECTOR_H
#define OUTLET_POSE_ESTIMATION_DETECTOR_H

#include <visual_pose_estimation/object_detector.h>
#include "outlet_pose_estimation/detail/outlet_detector.h"

namespace outlet_pose_estimation {

class Detector
{
public:
  Detector();

  bool loadTemplate(const std::string& filename);

  const outlet_template_t& getTemplate() const { return outlet_template_; }

  virtual bool detect(const cv::Mat& image, std::vector<cv::Point2f>& points) const;

  virtual void getDisplayImage(const cv::Mat& source,
                               const std::vector<cv::Point2f>& points,
                               bool success, cv::Mat& display) const;

protected:
  outlet_template_t outlet_template_;
  mutable std::vector<outlet_t> outlets_; // HACK: cheat and reuse for getDisplayImage
};

} //namespace outlet_pose_estimation

#endif
