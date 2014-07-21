#ifndef VISUAL_POSE_ESTIMATION_PLANAR_H
#define VISUAL_POSE_ESTIMATION_PLANAR_H

//#include <cv.h>
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/legacy/compat.hpp"
#include <opencv2/nonfree/nonfree.hpp>

namespace cv {

void findPlanarObjectPose(const Mat& object_points, const Mat& image_points, const Point3f& normal, 
                          const Mat& intrinsic_matrix, const Mat& distortion_coeffs, std::vector<Point3f>& object_points_crf);
    
void solvePlanarPnP(const Mat& objectPoints, const Mat& imagePoints, const Mat& cameraMatrix, const Mat& distCoeffs, 
                    Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false);

} //namespace cv

#endif
