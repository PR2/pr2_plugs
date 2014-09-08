#ifndef VISUAL_POSE_ESTIMATION_OBJECT_DETECTOR_H
#define VISUAL_POSE_ESTIMATION_OBJECT_DETECTOR_H

#include <opencv/cv.h>
#include <vector>

namespace visual_pose_estimation {

class ObjectDetector
{
public:
  virtual bool detect(const cv::Mat& image, std::vector<cv::Point2f>& points) const = 0;

  virtual void getDisplayImage(const cv::Mat& source,
                               const std::vector<cv::Point2f>& points,
                               bool success, cv::Mat& display) const = 0;
};

} //namespace visual_pose_estimation

#endif
