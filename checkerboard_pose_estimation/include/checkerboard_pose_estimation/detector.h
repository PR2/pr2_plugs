#ifndef CHECKERBOARD_POSE_ESTIMATION_DETECTOR_H
#define CHECKERBOARD_POSE_ESTIMATION_DETECTOR_H

#include <visual_pose_estimation/object_detector.h>

namespace checkerboard_pose_estimation {

class Detector
{
public:
  enum Side { LEFT, RIGHT };
  
  Detector();

  Detector(int width, int height, bool do_subpixel = true, Side origin_side = LEFT);

  void setDimensions(int width, int height);

  int width() const;
  int height() const;
  
  bool getSubpixel() const;
  void setSubpixel(bool is_on);

  Side getOriginSide() const;
  void setOriginSide(Side side);
  
  virtual bool detect(const cv::Mat& image, std::vector<cv::Point2f>& points) const;

  virtual void getDisplayImage(const cv::Mat& source,
                               const std::vector<cv::Point2f>& points,
                               bool success, cv::Mat& display) const;

protected:
  int width_;
  int height_;
  bool do_subpixel_refinement_;
  Side origin_side_;
};


inline Detector::Detector()
  : width_(0), height_(0), do_subpixel_refinement_(true), origin_side_(LEFT)
{
}

inline Detector::Detector(int width, int height, bool do_subpixel, Side origin_side)
  : width_(width), height_(height),
    do_subpixel_refinement_(do_subpixel),
    origin_side_(origin_side)
{
}

inline void Detector::setDimensions(int width, int height)
{
  width_ = width;
  height_ = height;
}

inline int Detector::width() const { return width_; }
inline int Detector::height() const { return height_; }
  
inline bool Detector::getSubpixel() const { return do_subpixel_refinement_; }
inline void Detector::setSubpixel(bool is_on) { do_subpixel_refinement_ = is_on; }

inline Detector::Side Detector::getOriginSide() const { return origin_side_; }
inline void Detector::setOriginSide(Side side) { origin_side_ = side; }

} //namespace checkerboard_pose_estimation

#endif
