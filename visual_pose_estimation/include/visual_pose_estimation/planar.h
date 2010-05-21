#ifndef VISUAL_POSE_ESTIMATION_PLANAR_H
#define VISUAL_POSE_ESTIMATION_PLANAR_H

#include <cv.h>

namespace cv {

void findPlanarObjectPose(const Mat& object_points, const Mat& image_points, const Point3f& normal, 
                          const Mat& intrinsic_matrix, const Mat& distortion_coeffs, std::vector<Point3f>& object_points_crf);
    
void solvePlanarPnP(const Mat& objectPoints, const Mat& imagePoints, const Mat& cameraMatrix, const Mat& distCoeffs, 
                    Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false);

} //namespace cv

#endif
