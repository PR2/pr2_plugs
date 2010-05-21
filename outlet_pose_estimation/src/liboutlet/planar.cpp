/*
 *  planar.cpp
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 1/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <time.h>

#include "outlet_pose_estimation/detail/planar.h"
#include <highgui.h>
#include <stdio.h>

#include <limits>

void gen_3x3_matrix(CvMat* in, CvMat* out)
{
	for(int r = 0; r < 2; r++)
	{
		for(int c = 0; c < 3; c++)
		{
			float value = cvmGet(in, r, c);
			cvmSet(out, r, c, value);
		}
	}

	cvmSet(out, 2, 0, 0.0f);
	cvmSet(out, 2, 1, 0.0f);
	cvmSet(out, 2, 2, 1.0f);
}

CvPoint2D32f apply_mat(CvMat* mat, CvPoint2D32f vec)
{
	CvPoint2D32f res = cvPoint2D32f(cvmGet(mat, 0, 0)*vec.x + cvmGet(mat, 0, 1)*vec.y,
									cvmGet(mat, 1, 0)*vec.x + cvmGet(mat, 1, 1)*vec.y);
	return(res);
}

CvRect calc_mapped_rectangle(CvMat* mat, CvRect roi)
{
	CvPoint2D32f p[4];
	p[0] = cvPoint2D32f(roi.x, roi.y);
	p[1] = cvPoint2D32f(roi.x + roi.width, roi.y);
	p[2] = cvPoint2D32f(roi.x + roi.width, roi.y + roi.height);
	p[3] = cvPoint2D32f(roi.x, roi.y + roi.height);
	
	CvPoint2D32f r[4];
	for(int i = 0; i < 4; i++)
	{
		r[i] = apply_mat(mat, p[i]);
	}
	
	float xmin = FLT_MAX, xmax = -FLT_MAX, ymin = FLT_MAX, ymax = -FLT_MAX;
	for(int i = 0; i < 4; i++)
	{
		xmin = MIN(xmin, r[i].x);
		xmax = MAX(xmax, r[i].x);
		ymin = MIN(ymin, r[i].y);
		ymax = MAX(ymax, r[i].y);
	}
	
	return(cvRect(xmin, ymin, xmax - xmin, ymax - ymin));
}

void copy_cols(CvMat* src, CvMat* dst, int start_col, int end_col)
{
	for(int r = 0; r < src->rows; r++)
	{
		for(int c = start_col; c < end_col; c++)
		{
			float val = cvmGet(src, r, c);
			cvmSet(dst, r, c, val);
		}
	}
}

// generates a 2x3 affine transform 
CvSize gen_random_homog_transform(CvRect roi, CvMat* mat)
{
	CvMat* rt = cvCreateMat(2, 2, CV_32FC1);
	CvMat* rp = cvCreateMat(2, 2, CV_32FC1);
	CvMat* rpi = cvCreateMat(2, 2, CV_32FC1);
	CvMat* s = cvCreateMat(2, 2, CV_32FC1);
	CvMat* final = cvCreateMat(2, 2, CV_32FC1);
	
	CvMat* temp = cvCreateMat(2, 3, CV_32FC1);
		
	float phi = (float(rand())/RAND_MAX*2 - 1)*60;
	cv2DRotationMatrix(cvPoint2D32f(0, 0), phi, 1, temp);
	copy_cols(temp, rp, 0, 2);

	cv2DRotationMatrix(cvPoint2D32f(0, 0), -phi, 1, temp);
	copy_cols(temp, rpi, 0, 2);
	
	const float max_angle = 30;
	float theta = (float(rand())/RAND_MAX*2 - 1)*max_angle;
	cv2DRotationMatrix(cvPoint2D32f(0, 0), theta, 1, temp);
	copy_cols(temp, rt, 0, 2);
	
	const float smin = 0.8f;
	const float smax = 1.3f;
	float l1 = float(rand())/RAND_MAX*(smax - smin) + smin;
	float l2 = float(rand())/RAND_MAX*(smax - smin) + smin;
	cvmSet(s, 0, 0, l1);
	cvmSet(s, 0, 1, 0.0f);
	cvmSet(s, 1, 0, 0.0f);
	cvmSet(s, 1, 1, l2);
	
	cvMatMul(rt, rpi, final);
	cvMatMul(final, s, final);
	cvMatMul(final, rp, final);
	
	CvRect bound = calc_mapped_rectangle(final, roi);
	
	for(int r = 0; r < 2; r++)
	{
		for(int c = 0; c < 2; c++)
		{
			float value = cvmGet(final, r, c);
			cvmSet(mat, r, c, value);
		}
	}
	
	int length = MAX(bound.width, bound.height);
	bound.x -= (length - bound.width)/2;
	bound.y -= (length - bound.height)/2;
	
	cvmSet(mat, 0, 2, -bound.x);
	cvmSet(mat, 1, 2, -bound.y);
	
	cvReleaseMat(&rt);
	cvReleaseMat(&rp);
	cvReleaseMat(&s);
	cvReleaseMat(&rpi);
	cvReleaseMat(&temp);
	cvReleaseMat(&final);
	
	
	return cvSize(length, length);
}

void gen_random_homog_patches(IplImage* src, int count, IplImage** dst)
{
	srand(clock());
	
	CvRect roi = cvGetImageROI(src);
	cvResetImageROI(src);
	
	CvMat* transform = cvCreateMat(2, 3, CV_32FC1);
	for(int i = 0; i < count; i++)
	{
		CvSize size = gen_random_homog_transform(roi, transform);
		dst[i] = cvCreateImage(size, IPL_DEPTH_8U, 1);
		cvWarpAffine(src, dst[i], transform);
		cvEqualizeHist(dst[i], dst[i]);
	}
	cvSetImageROI(src, roi);
	
	cvReleaseMat(&transform);
}

void save_image_array(const char* folder, const char* filename, int count, IplImage** images)
{
	for(int i = 0; i < count; i++)
	{
		char buf[1024];
		sprintf(buf, "%s/%s_%d.jpg", folder, filename, i);
		cvSaveImage(buf, images[i]);
	}
}

void release_image_array(int count, IplImage** images)
{
	for(int i = 0; i < count; i++)
	{
		cvReleaseImage(&images[i]);
	}
}
void test_homog_transform(IplImage* src)
{
	const int count = 30;
	IplImage* images[count];
	gen_random_homog_patches(src, count, images);
	save_image_array("../../patches", "", count, images);
	release_image_array(count, images);
}

namespace cv{
    
    
// Algorithm:
// plane equation: P*N + c = 0
// we find point rays in 3D from image points and camera parameters
// then we fit c by minimizing average L2 distance between rotated and translated object points
// and points found by crossing point rays with plane. We use the fact that center of mass 
// of object points and fitted points should coincide.
void findPlanarObjectPose(const Mat& object_points, const Mat& image_points, const Point3f& normal, 
                              const Mat& intrinsic_matrix, const Mat& distortion_coeffs, std::vector<Point3f>& object_points_crf)
{
    vector<Point2f> _rays;
    undistortPoints(image_points, _rays, intrinsic_matrix, distortion_coeffs);
    
    // filter rays that are parallel to the plane
    vector<Point3f> rays;
    for(size_t i = 0; i < _rays.size(); i++)
    {
        double proj = rays[i].dot(normal);
        if(fabs(proj) > std::numeric_limits<double>::epsilon())
        {
            rays.push_back(Point3f(_rays[i].x, _rays[i].y, 1.0f));
        }
    }
    
    Point3f pm(0.0f, 0.0f, 0.0f);
    for(size_t i = 0; i < rays.size(); i++)
    {
        pm = pm + rays[i]*(1.0/rays[i].dot(normal));
    }
    
    pm = -pm*(1.0/rays.size());
    
    double dev = 0.0;
    for(size_t i = 0; i < rays.size(); i++)
    {
        Point3f pdev = rays[i]*(1.0/rays[i].dot(normal)) - pm;
        dev += pdev.dot(pdev);
    }
    
    double C = -1.0/sqrt(dev);
    
    object_points_crf.resize(rays.size());
    for(size_t i = 0; i < rays.size(); i++)
    {
        object_points_crf[i] = -rays[i]*(C/rays[i].dot(normal));
    }
}

};
