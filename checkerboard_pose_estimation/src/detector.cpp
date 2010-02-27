#include "checkerboard_pose_estimation/detector.h"
#include "checkerboard_pose_estimation/cvcalibinit_lowres.h"

#include <outlet_pose_estimation/detail/features.h> // ApplyGamma
#include <limits>

namespace checkerboard_pose_estimation {

bool Detector::detect(const cv::Mat& image, std::vector<cv::Point2f>& points) const
{
  IplImage ipl = (IplImage)image;

  // Do gamma correction
  ApplyGamma(&ipl, 1.5f); /// @todo Why do we need gamma correction here?

  // Find the checkerboard corners. First we try a specialized low-resolution detector,
  // falling back to the standard OpenCV detector.
  points.resize(width_ * height_);
  int corners_found = 0;
  int ret = cvFindChessboardCornersLowres(&ipl, cvSize(width_, height_),
                                          (CvPoint2D32f*)&points[0], &corners_found);
  if (!ret) {
    ret = cvFindChessboardCorners(&ipl, cvSize(width_, height_),
                                  (CvPoint2D32f*)&points[0], &corners_found,
                                  CV_CALIB_CB_ADAPTIVE_THRESH);
    if(!ret) {
      points.resize(corners_found);
      return false;
    }
  }

  // We enforce a unique ordering on the checkerboard corners. The lowres detector now
  // guarantees a unique ordering, but the fallback OpenCV detector does not yet.
  /// @todo Remove unique ordering code when OpenCV is updated.
  
  // Force consistent clockwise ordering on the four outside corners.
  cv::Point2f c0 = points[0];
  cv::Point2f c1 = points[width_ - 1];
  cv::Point2f c2 = points[(height_ - 1) * width_];
  if ((c1.x - c0.x)*(c2.y - c0.y) - (c2.x - c0.x)*(c1.y - c0.y) < 0) {
    for (int i = 0; i < height_; ++i)
      std::reverse(points.begin() + width_*i, points.begin() + width_*(i+1));
  }

  // Reverse the corners if the origin (corner 0) is on the wrong side.
  Side detected_side = (c0.x > c2.x) ? RIGHT : LEFT;
  if (detected_side != origin_side_)
    std::reverse(points.begin(), points.end());

  if (do_subpixel_refinement_) {
    // Make sure we use a conservative radius, less than the minimum distance between corners.
    // Otherwise subpixel "refinement" may move detected points to the wrong corners.
    float min_distance_sq = std::numeric_limits<float>::max();
    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_ - 1; ++col) {
        int index = row*width_ + col;
        cv::Point2f a = points[index], b = points[index+1];
        cv::Point2f diff = a - b;
        min_distance_sq = std::min(min_distance_sq, diff.x*diff.x + diff.y*diff.y);
      }
    }
    for (int row = 0; row < height_ - 1; ++row) {
      for (int col = 0; col < width_; ++col) {
        int index = row*width_ + col;
        cv::Point2f a = points[index], b = points[index+width_];
        cv::Point2f diff = a - b;
        min_distance_sq = std::min(min_distance_sq, diff.x*diff.x + diff.y*diff.y);
      }
    }
    int radius = std::sqrt(min_distance_sq) * 0.5f + 0.5f;

    cvFindCornerSubPix(&ipl, (CvPoint2D32f*)&points[0], corners_found,
                       cvSize(radius,radius), cvSize(-1,-1),
                       cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
  }

  return true;
}

void Detector::getDisplayImage(const cv::Mat& source,
                               const std::vector<cv::Point2f>& points,
                               bool success, cv::Mat& display) const
{
  static const int MAGNIFICATION = 4;

  /// @todo Currently assuming source is grayscale
  cv::Mat color;
  cv::cvtColor(source, color, CV_GRAY2BGR);  
  cv::resize(color, display, cv::Size(), MAGNIFICATION, MAGNIFICATION);

  if (points.empty()) return;
  
  std::vector<cv::Point2f> scaled_points(points.size());
  for (size_t i = 0; i < points.size(); ++i)
    scaled_points[i] = cv::Point2f(points[i].x * MAGNIFICATION, points[i].y * MAGNIFICATION);
  cv::drawChessboardCorners(display, cv::Size(width_, height_), cv::Mat(scaled_points), success);
}

} //namespace checkerboard_pose_estimation
