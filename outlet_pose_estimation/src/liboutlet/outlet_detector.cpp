/*
 *  outlet_detector.cpp
 *  outlet_sample
 *
 *  Created by Victor  Eruhimov on 2/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <stdio.h>
#include <stdarg.h>

#include "outlet_pose_estimation/detail/one_way_outlets.h"

#if defined(_VERBOSE)
static int PRINTF( const char* fmt, ... )
{
    va_list args;
    va_start(args, fmt);
    int ret = vprintf(fmt, args);
	return ret;
}
#else
static int PRINTF( const char*, ... )
{
    return 0;
}
#endif // _VERBOSE


#include "outlet_pose_estimation/detail/outlet_detector.h"
#include "outlet_pose_estimation/detail/planar.h"
#include "outlet_pose_estimation/detail/outlet_tuple.h"
#include "outlet_pose_estimation/detail/gh_outlets.h"

#include "highgui.h"

using namespace std;

int detect_outlet_tuple(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
	vector<outlet_t>& outlets, const outlet_template_t& outlet_templ,
	const char* output_path, const char* filename, float* scale_ranges)
{
#if 1
    if (distortion_params) {
        // correcting for distortion
        IplImage* _img = cvCloneImage(src);
        //		int64 _t1 = cvGetTickCount();
        cvUndistort2(_img, src, intrinsic_matrix, distortion_params);
        //		int64 _t2 = cvGetTickCount();
        //		printf("Undistort time elapsed: %f", double(_t2 - _t1)/cvGetTickFrequency()*1e-6);
        cvReleaseImage(&_img);
    }
#endif

    int ret = 1;

#if !defined(_GHT)
    if(outlet_templ.get_color() == outletOrange && outlet_templ.get_count() == 4)
    {
        ret = detect_outlet_tuple_2x2_orange(src, intrinsic_matrix, distortion_params, outlets, outlet_templ, output_path, filename);
    }
    else if(outlet_templ.get_count() == 2)
    {
        ret = detect_outlet_tuple_2x1(src, intrinsic_matrix, distortion_params, outlets, outlet_templ, output_path, filename);
    }
#else
    CvRect roi = cvGetImageROI(src);
    IplImage* red = cvCreateImage(cvSize(roi.width, roi.height), IPL_DEPTH_8U, 1);
#if defined(_RED)
    cvSetImageCOI(src, 3);
    cvCopy(src, red);
    cvSetImageCOI(src, 0);
#else
    cvSetZero(red);
    cvSetImageCOI(src, 3);
    cvCopy(src, red);
    cvConvertScale(red, red, 0.5);
    IplImage* red1 = cvCloneImage(red);
    cvSetImageCOI(src, 2);
    cvCopy(src, red1);
    cvConvertScale(red1, red1, 0.5);
    cvAdd(red, red1, red);
    cvReleaseImage(&red1);
    cvSetImageCOI(src, 0);
#endif //_RED

#if 1
    detect_outlets_one_way(red, outlet_templ, outlets, src,output_path, filename, scale_ranges);
#else
//    detect_outlets_gh(red, outlet_templ, outlets, src, output_path, filename);
#endif
    
    //printf("Returned %d outlets\n", outlets.size());

    // take into account image roi
    for(size_t i = 0; i < outlets.size(); i++)
    {
        outlets[i].hole1 = cv::Point2d(outlets[i].hole1) + cv::Point2d(roi.x, roi.y);
        outlets[i].hole2 = cv::Point2d(outlets[i].hole2) + cv::Point2d(roi.x, roi.y);
        outlets[i].ground_hole = cv::Point2d(outlets[i].ground_hole) + cv::Point2d(roi.x, roi.y);
    }


#endif
    cvReleaseImage(&red);

    if(outlets.size() != (size_t)outlet_templ.get_count())
    {
        // template not found
        return 0;
    }
    else
    {
        ret = 1;
    }

#if 0
    calc_outlet_3d_coord_2x2(intrinsic_matrix, outlet_templ, outlets);
#else
    // now find 3d coordinates of the outlet in the camera reference frame
    // first, calculate homography
#ifdef _DO_POSE_ESTIMATE_IN_DETECTOR
	if (intrinsic_matrix)
	{
#if 0
		CvPoint3D32f* object_holes_3d = new CvPoint3D32f[outlet_templ.get_count()*3];
		CvPoint2D32f* object_holes_2d = new CvPoint2D32f[outlet_templ.get_count()*3];
		CvPoint2D32f* image_holes = new CvPoint2D32f[outlet_templ.get_count()*3];

		outlet_templ.get_holes_2d(object_holes_2d);
		outlet_templ.get_holes_3d(object_holes_3d);
		for(size_t i = 0; i < outlets.size(); i++)
		{
			image_holes[3*i] = cvPoint2D32f(outlets[i].hole1.x, outlets[i].hole1.y); // power left
			image_holes[3*i + 1] = cvPoint2D32f(outlets[i].hole2.x, outlets[i].hole2.y); // power right
			image_holes[3*i + 2] = cvPoint2D32f(outlets[i].ground_hole.x, outlets[i].ground_hole.y); // ground hole
		}

		CvMat* homography = cvCreateMat(3, 3, CV_32FC1);
		CvMat* inv_homography = cvCreateMat(3, 3, CV_32FC1);

		CvMat* image_holes_mat = cvCreateMat(outlet_templ.get_count()*3, 2, CV_32FC1);
		CvMat* object_holes_mat = cvCreateMat(outlet_templ.get_count()*3, 2, CV_32FC1);
		for(int i = 0; i < outlet_templ.get_count()*3; i++)
		{
			cvmSet(image_holes_mat, i, 0, image_holes[i].x);
			cvmSet(image_holes_mat, i, 1, image_holes[i].y);
			cvmSet(object_holes_mat, i, 0, object_holes_2d[i].x);
			cvmSet(object_holes_mat, i, 1, object_holes_2d[i].y);
		}
		cvFindHomography(image_holes_mat, object_holes_mat, homography);
		cvFindHomography(object_holes_mat, image_holes_mat, inv_homography);
		cvReleaseMat(&image_holes_mat);
		cvReleaseMat(&object_holes_mat);

		CvPoint3D32f origin;
		CvPoint2D32f origin_2d;
		map_point_homography(image_holes[0], homography, origin_2d);
		origin = cvPoint3D32f(origin_2d.x, origin_2d.y, 0.0);
		CvPoint2D32f scale = cvPoint2D32f(1.0, 1.0);

		CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
		CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
		calc_camera_pose(intrinsic_matrix, 0, outlet_templ.get_count()*3, object_holes_3d, image_holes,
						 rotation_vector, translation_vector);
		calc_outlet_coords(outlets, homography, origin, scale, rotation_vector, translation_vector, inv_homography);

		delete object_holes_2d;
		delete object_holes_3d;
		delete image_holes;
		cvReleaseMat(&rotation_vector);
		cvReleaseMat(&translation_vector);
		cvReleaseMat(&homography);
		cvReleaseMat(&inv_homography);
#else
        if(outlets.size() >= 4)
        {
            calc_outlet_coords_ground(outlets, outlet_templ, intrinsic_matrix, 0);
        }
        else
        {
            calc_outlet_coords(outlets, outlet_templ, intrinsic_matrix, 0);
//            printf("Outlet 0 ground hole: %f,%f,%f\n", outlets[0].coord_hole_ground.x, outlets[0].coord_hole_ground.y, outlets[0].coord_hole_ground.z);
        }

#endif
	}
#endif // Whether to do pose estimation at all
#endif

    return ret;
}

void calc_outlet_3d_coord_2x2(CvMat* intrinsic_matrix, const outlet_template_t& outlet_templ, vector<outlet_t>& outlets)
{
	// filter outlets using template match
	CvMat* homography = 0;
	CvPoint3D32f origin;
	CvPoint2D32f scale;

	homography = cvCreateMat(3, 3, CV_32FC1);
	CvMat* inv_homography = cvCreateMat(3, 3, CV_32FC1);

    CvPoint2D32f centers[4];
    for(int i = 0; i < 4; i++)
    {
        centers[i] = cvPoint2D32f((outlets[i].hole1.x + outlets[i].hole2.x)*0.5,
                                  (outlets[i].hole1.y + outlets[i].hole2.y)*0.5);
    }

    calc_outlet_homography(centers, homography,
                           outlet_templ, inv_homography);


	calc_origin_scale(centers, homography, &origin, &scale);

	CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
	CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
	calc_camera_outlet_pose(intrinsic_matrix, 0, outlet_templ, centers, rotation_vector, translation_vector);
	calc_outlet_coords(outlets, homography, origin, scale, rotation_vector, translation_vector, inv_homography);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&inv_homography);
}

int detect_outlet_tuple_2x2_orange(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
                            vector<outlet_t>& outlets, const outlet_template_t& outlet_templ,
                            const char* output_path, const char* filename)
{
	outlet_tuple_t outlet_tuple;

	outlet_tuple.tuple_mask = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
	int ret = find_outlet_centroids(src, outlet_tuple, output_path, filename);
	if(!ret)
	{
		PRINTF("find_outlet_centroids did not find a tuple\n");
		return 0;
	}

	vector<outlet_feature_t> features;
	detect_outlets(src, features, outlets, &outlet_tuple, output_path, filename);

	CvPoint2D32f hor_dir = outlet_tuple.centers[1] - outlet_tuple.centers[0];
	//	select_orient_outlets(hor_dir, outlets, 4);

	// filter outlets using template match
	CvMat* homography = 0;
	CvPoint3D32f origin;
	CvPoint2D32f scale;

	homography = cvCreateMat(3, 3, CV_32FC1);
	CvMat* inv_homography = cvCreateMat(3, 3, CV_32FC1);

	// test the distance
	const int iter_count = 1;
	for(int j = 0; j < iter_count; j++)
	{
		calc_outlet_homography(outlet_tuple.centers, homography,
							   outlet_templ, inv_homography);

/*
		float sum_dist = 0;
		for(int i = 0; i < 4; i++)
		{
			vector<CvPoint2D32f> borders;
			map_vector(outlet_tuple.borders[i], homography, borders);
			CvPoint2D32f center = calc_center(borders);
			vector<CvPoint2D32f> temp;
			temp.push_back(outlet_tuple.centers[i]);
			map_vector(temp, homography, temp);
			float dist = length(temp[0] - center);
			sum_dist += dist;

			temp.clear();
			temp.push_back(center);
			map_vector(temp, inv_homography, temp);
			outlet_tuple.centers[i] = temp[0];
		}


#if defined(_VERBOSE)
		printf("Iteration %d: error %f pixels\n", j, sum_dist/4);
#endif //_VERBOSE
*/
	}

	calc_origin_scale(outlet_tuple.centers, homography, &origin, &scale);

	CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
	CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
	calc_camera_outlet_pose(intrinsic_matrix, 0, outlet_templ, outlet_tuple.centers, rotation_vector, translation_vector);
	calc_outlet_coords(outlets, homography, origin, scale, rotation_vector, translation_vector, inv_homography);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&inv_homography);

	filter_outlets_size(outlets);

	filter_outlets_tuple(outlets, outlet_tuple.tuple_mask, hor_dir);

#if defined(_VERBOSE)
	if(output_path && filename)
	{
		IplImage* temp = cvCloneImage(src);
		draw_outlets(temp, outlets);

		char buf[1024];
		sprintf(buf, "%s/output_filt/%s", output_path, filename);
		strcpy(buf + strlen(buf) - 3, "jpg");
		cvSaveImage(buf, temp);

		cvReleaseImage(&temp);
	}
#endif //_VERBOSE

	PRINTF(" found %d holes, %d outlets\n", features.size(), outlets.size());

	if(homography == 0)
	{
		PRINTF("Homography mask not found.\n");
		return 0;
	}
	else if(outlets.size() != 4)
	{
        cvReleaseMat(&homography);
		PRINTF("Outlet tuple not found!\n");
		return 0;
	}

    cvReleaseMat(&homography);

	return 1;
}

int detect_outlet_tuple_2x1(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
                                   vector<outlet_t>& outlets, const outlet_template_t& outlet_templ,
                                   const char* output_path, const char* filename)
{
    vector<feature_t> holes;

    IplImage* img_small = cvCreateImage(cvSize(src->width/2, src->height/2), IPL_DEPTH_8U, 3);
    cvResize(src, img_small);

    IplImage* red = cvCreateImage(cvSize(img_small->width, img_small->height), IPL_DEPTH_8U, 1);
    cvSetImageCOI(img_small, 3);
    cvCopy(img_small, red);
    cvSetImageCOI(img_small, 0);

    detect_outlets_2x1_one_way(red, outlet_templ.get_one_way_descriptor_base(), holes, img_small, output_path, filename);
    cvReleaseImage(&red);
    cvReleaseImage(&img_small);

    if(holes.size() == 6)
    {
        features2outlets_2x1(holes, outlets);

        CvPoint2D32f centers[6];
        for(int i = 0; i < 6; i++)
        {
            centers[i] = cvPoint2D32f(holes[i].pt.x*2, holes[i].pt.y*2);
        }

        CvPoint2D32f object_points[6];
        generate_object_points_2x1(object_points);

        CvMat* homography = cvCreateMat(3, 3, CV_32FC1);
        cvGetPerspectiveTransform(centers, object_points, homography);

        CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
        CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);

        calc_camera_outlet_pose(intrinsic_matrix, distortion_params, outlet_templ, centers, rotation_vector, translation_vector);

        calc_outlet_coords(outlets, homography, cvPoint3D32f(0.0f, 0.0f, 0.0f), cvPoint2D32f(1.0f, 1.0f),
            rotation_vector, translation_vector);

        return 1;
    }
    else
    {
        return 0;
    }
}

void features2outlets_2x1(const vector<feature_t>& features, vector<outlet_t>& outlets)
{
    outlet_t outlet;
    outlet.hole1 = features[0].pt*2;
    outlet.hole2 = features[1].pt*2;
    outlet.ground_hole = features[4].pt*2;
    outlets.push_back(outlet);

    outlet.hole1 = features[2].pt*2;
    outlet.hole2 = features[3].pt*2;
    outlet.ground_hole = features[5].pt*2;
    outlets.push_back(outlet);
}
