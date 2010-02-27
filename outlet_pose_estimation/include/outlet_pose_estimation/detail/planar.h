/*
 *  planar.h
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 1/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#if !defined(_PLANAR_H)
#define _PLANAR_H

#include <cv.h>

const float pi = 3.1415926;

// mat is 2x2 matrix, the function maps 4 corners of an image of given size
// and calculates a bounding rectangle around the resulting quadrangle
CvRect calc_mapped_rectangle(CvRect roi, CvMat* mat);

// generating random affine transform. mat should be allocated as flt 2x3.
CvSize gen_random_homog_transform(CvRect roi, CvMat* mat);

// the function gets an src image (possibly with ROI set) and transforms it randomly.
// The transform matrix is T = R(t)R^{-1}(p)SR(p), where R is rotation matrix, S is scaling matrix.
// Scaling parameters (two -- along x and y) are sampled from [0.6,1.5], 
// angles t and p -- [-pi,pi].

void gen_random_homog_patches(IplImage* src, int count, IplImage** dst);
void test_homog_transform(IplImage* src);

void save_image_array(const char* folder, const char* filename, int count, IplImage** images);

namespace cv{
void findPlanarObjectPose(const Mat& object_points, const Mat& image_points, const Point3f& normal, 
                          const Mat& intrinsic_matrix, const Mat& distortion_coeffs, std::vector<Point3f>& object_points_crf);
}


#endif //_PLANAR_H
