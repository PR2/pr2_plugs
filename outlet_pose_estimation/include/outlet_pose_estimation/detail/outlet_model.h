/*
 *  outlet_model.h
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 12/22/08.
 *  Copyright 2008 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#if !defined(_OUTLET_MODEL_H)
#define _OUTLET_MODEL_H

#include <vector>
#include <map>

#include <cv.h>
#include <ml.h>

#include "outlet_pose_estimation/detail/outlet_tuple.h"

typedef struct
{
	CvRect bbox;
	float weight;
} outlet_feature_t;

inline CvPoint feature_center(outlet_feature_t feature)
{
	return cvPoint(feature.bbox.x + feature.bbox.width/2, feature.bbox.y + feature.bbox.height/2);
}

struct outlet_t
{
	CvSeq* outlet;
	CvPoint hole1;
	CvPoint hole2;
    CvPoint ground_hole;
	outlet_feature_t feature1;
	outlet_feature_t feature2;
	CvPoint3D32f coord_hole1;
	CvPoint3D32f coord_hole2;
	CvPoint3D32f coord_hole_ground;
	float weight;
	float weight_orient;
	bool hole1_detected;
	bool hole2_detected;
	bool ground_hole_detected;

    cv::Point2f hole1f;
    cv::Point2f hole2f;
    cv::Point2f hole_groundf;
    bool is_subpixel;

    outlet_t() : is_subpixel(false) {};
};

inline float outlet_size(outlet_t outlet);
CvRect outlet_rect(outlet_t outlet);

#define __max MAX
#define __min MIN

// detect_outlet: detects outlets in an image.
// Input parameters:
//	src: input color image
//	features: output array of features detected in the image
//	outlets: output array of outlets detected in the image
//	outlet_tuple: the optional tuple found in the image. It is used to filter out
//		features outside of the tuple mask or close to its boundary.
//	output_path: path for logging the results
//	filename: filename for logging the results
void detect_outlets(IplImage* src, std::vector<outlet_feature_t>& features, std::vector<outlet_t>& outlets,
					outlet_tuple_t* outlet_tuple, const char* output_path = 0, const char* filename = 0);

inline void cvRectangle(IplImage* img, CvRect rect, CvScalar color, int thickness)
{
	cvRectangle(img, cvPoint(rect.x, rect.y), cvPoint(rect.x + rect.width, rect.y + rect.height),
				color, thickness);
}

inline CvRect fit_rect_roi(CvRect rect, CvRect roi)
{
	CvRect fit = rect;
	fit.x = MAX(fit.x, roi.x);
	fit.y = MAX(fit.y, roi.y);
	fit.width = MIN(fit.width, roi.x + roi.width - fit.x - 1);
	fit.height = MIN(fit.height, roi.y + roi.height - fit.y - 1);
	assert(fit.width > 0);
	assert(fit.height > 0);
	return(fit);
}

inline CvRect fit_rect(CvRect rect, IplImage* img)
{
	CvRect roi = cvGetImageROI(img);
	return fit_rect_roi(rect, roi);
}

inline CvRect double_rect(CvRect small_rect)
{
	return cvRect(small_rect.x - small_rect.width/2, small_rect.y - small_rect.height/2,
				  small_rect.width*2, small_rect.height*2);
}

inline int is_point_inside_rect(CvRect rect, CvPoint point)
{
	if(point.x >= rect.x && point.y >= rect.y &&
	   point.x <= rect.x + rect.width && point.y <= rect.y + rect.height)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

CvRect getOutletROI(const std::vector<outlet_t>& outlets);

int is_point_incenter_roi(const std::vector<CvRect>& rects, CvPoint point);

//void DrawKeypoints(IplImage* img, std::vector<Keypoint> keypts);
void DrawKeypoints(IplImage* img, std::vector<outlet_feature_t> features);

typedef std::map<std::string, std::vector<CvRect> > outlet_roi_t;
CvMat* vector2mat(const std::vector<int>& vec);
void read_outlet_roi(const char* filename, outlet_roi_t& outlet_roi);
void extract_intensity_features(IplImage* grey, const std::vector<outlet_feature_t>& keypts, CvMat** mat,
								int equalize_hist = 0,
								const std::vector<int>& labels = std::vector<int>(), const char* buf = 0);
//void calc_labels(const std::vector<CvRect>& rects, const std::vector<Keypoint>& keypts, std::vector<int>& labels);
void calc_labels(const std::vector<CvRect>& rects, const std::vector<outlet_feature_t>& keypts, std::vector<int>& labels);
void FilterPoints(IplImage* grey, std::vector<outlet_feature_t>& keypts, const CvRTrees* rtrees);
void filter_outlets(IplImage* grey, std::vector<outlet_t>& outlets, CvRTrees* rtrees);

//void outletfarr2keypointarr(const std::vector<outlet_feature_t>& features, std::vector<Keypoint>& keypoints);
//void keypointarr2outletfarr(const std::vector<Keypoint>& keypoints, std::vector<outlet_feature_t>& features);

void find_outlet_features(IplImage* src, std::vector<outlet_feature_t>& features, const char* filename);
void find_outlet_features_fast(IplImage* src, std::vector<outlet_feature_t>& features, float hole_contrast,
                               const char* output_path, const char* filename);

// generates several perspective distortions of the original outlet and
// extracts intensity values into CvMat format for subsequent learning of a classifier
int generate_outlet_samples(IplImage* grey, outlet_t outlet, int count, CvMat** predictors, const char* filename = 0);

// train an outlet model by generating perspective distorted outlets
void train_outlet_model(const char* path, const char* config_filename,
						const char* roi_filename, const char* forest_filename);

void write_pr(const char* pr_filename, const char* image_filename, const outlet_roi_t& outlet_roi,
			  const std::vector<outlet_t>& outlets);

void filter_negative_samples(const std::vector<CvRect>& rects, std::vector<outlet_feature_t>& keypts, float fraction);
void calc_contrast_factor(IplImage* grey, CvRect rect, float& contrast, float& variation);

void select_central_outlets(std::vector<outlet_t>& outlets, int count);
void select_orient_outlets(CvPoint2D32f orientation, std::vector<outlet_t>& outlets, int count = 0);
int select_orient_outlets_ex(IplImage* grey, std::vector<outlet_t>& outlets, const char* filename = 0);

void draw_outlets(IplImage* temp, const std::vector<outlet_t>& outlets);
void filter_canny(IplImage* grey, std::vector<outlet_feature_t>& features);

// a temporary solution for finding image homography
int load_homography_map(const char* filename, CvMat** map_matrix);
IplImage* load_match_template_mask(const char* filename);

CvMat* calc_image_homography(IplImage* src);


// uses template matching, currently done offline
void filter_outlets_templ_ex(std::vector<outlet_t>& outlets, CvMat* map_matrix, IplImage* mask);
int filter_outlets_templ(std::vector<outlet_t>& outlets, const char* filename);
int filter_outlets_templmatch(IplImage* src, std::vector<outlet_t>& outlets, IplImage* templ, const char* output_path,
							  const char* filename = 0, CvMat** homography = 0, CvPoint3D32f* origin = 0, CvPoint2D32f* scale = 0);

IplImage* calc_tuple_distance_map(IplImage* tuple_mask);


// calc_outlet_coords: calculate outlets 3D coordinates in camera coordinate system. The result is stored
// in the input vector of outlet_t objects and can be retrieved by looking into outlet_t::coord_hole* fields or
// by calling get_outlet_coordinates(...).
// Input parameters:
//	outlets: input vector of outlets.
//	map_matrix: homography matrix that maps a camera image into a rectified image.
//	origin, scale: parameters for calculating 3D coordinates of a point in a rectified image.
//		Are calculated by calc_origin_scale(...) function.
//	rotation_vector, translation_vector: vectors for mapping from an outlet coordinate system into
//		a camera coordinate system. Are calculated by calc_camera_pose(...).
int calc_outlet_coords(std::vector<outlet_t>& outlets, CvMat* map_matrix, CvPoint3D32f origin, CvPoint2D32f scale,
	CvMat* rotation_vector, CvMat* translation_vector, CvMat* inv_map_matrix = 0);

void calc_outlet_coords(std::vector<outlet_t>& outlets, const outlet_template_t& outlet_template,
                        CvMat* intrinsic_matrix, CvMat* distortion_params);
void calc_outlet_coords_ground(std::vector<outlet_t>& outlets, const outlet_template_t& outlet_template,
                        CvMat* intrinsic_matrix, CvMat* distortion_params);


void calc_outlet_dist_stat(const std::vector<outlet_t>& outlets, float& mean, float& stddev);
void calc_outlet_tuple_dist_stat(const std::vector<outlet_t>& outlets, float& ground_dist_x1,
								 float& ground_dist_x2, float& ground_dist_y);
void calc_outlet_coords(CvMat* rotation_vector, CvMat* translation_vector, const std::vector<cv::Point3f>& object_points, std::vector<outlet_t>& outlets);

void filter_features_mask(std::vector<outlet_feature_t>& features, IplImage* mask);
void filter_outlets_mask(std::vector<outlet_t>& outlets, IplImage* mask);
void filter_outlets_size(std::vector<outlet_t>& outlets);

void filter_features_distance_mask(std::vector<outlet_feature_t>& features, IplImage* distance_map);

int find_origin_chessboard(IplImage* src, CvMat* map_matrix, CvPoint3D32f& origin, float bar_length);

// filter_outlet_tuple: enforces one outlet per orange connected component.
// Input parameters:
//	outlets: input/output array of outlets
//	tuple_mask: a tuple mask
//	hor_dir: the horizontal direction (can be computed as teh difference between two outlet centers)
void filter_outlets_tuple(std::vector<outlet_t>& outlets, IplImage* tuple_mask, CvPoint2D32f hor_dir);

// retrieves coordinates of outlet holes in the following order: ground hole, left hole, right hole.
// the size of points array should be at least 3
void get_outlet_coordinates(const outlet_t& outlet, CvPoint3D32f* points);

void move_features(std::vector<outlet_feature_t>& features, CvPoint origin);

void estimateCameraPosition(const std::vector<KeyPointEx>& image_points, const std::vector<cv::Point3f>& object_points,
                        CvMat* intrinsic_matrix, CvMat* distortion_params, CvMat* rotation_vector, CvMat* translation_vector);

void getImagePoints(const std::vector<outlet_t>& outlets, std::vector<cv::Point2f>& image_points, std::vector<bool>& is_detected);


void findPrecisePowerHoleLocation(IplImage* grey, cv::Point2f center, cv::Point2f dir, cv::Point2f dir_perp, cv::Point2f& hole);
void findPreciseGroundHoleLocation(IplImage* grey, cv::Point2f center, cv::Point2f& hole);
void findPreciseOutletLocations(IplImage* grey, const outlet_template_t& outlet_template, std::vector<outlet_t>& outlets);
void findPreciseOutletLocationsAvg(IplImage* grey, const outlet_template_t& outlet_template, std::vector<outlet_t>& outlets);

void flipOutlet(std::vector<outlet_t>& outlets);

#endif //_OUTLET_MODEL_H
