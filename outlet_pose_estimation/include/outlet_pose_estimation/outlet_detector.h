/*
 *  outlet_detector.h
 *  outlet_sample
 *
 *  Created by Victor  Eruhimov on 2/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_OUTLET_DETECTOR_H)
#define _OUTLET_DETECTOR_H

#include <vector>

#include "outlet_pose_estimation/detail/outlet_model.h"


// detect_outlet_tuple: high-level function for detecting 4 orange outlets
// Input parameters:
//	src: input color image
//	intrinsic_matrix: camera matrix of intrinsic parameters
//	distortion_params: vector of distortion parameters
//	outlets: output vector of outlets
//  outlet_templ: a geometrical template for an array of outlets
//	output_path, filename: optional path and filename for logging the results
// Return value: 1 in case of success (found 4 outlets), 0 otherwise.
int detect_outlet_tuple(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
	std::vector<outlet_t>& outlets, const outlet_template_t& outlet_templ = outlet_template_t(),
	const char* output_path = 0, const char* filename = 0, float* scale_ranges = 0);

int detect_outlet_tuple_2x2_orange(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
                                   std::vector<outlet_t>& outlets, const outlet_template_t& outlet_templ,
                                   const char* output_path, const char* filename);

int detect_outlet_tuple_2x1(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
                                   std::vector<outlet_t>& outlets, const outlet_template_t& outlet_templ,
                                   const char* output_path, const char* filename);

int detect_outlet_tuple_2x1_orange(IplImage* src, CvMat* intrinsic_matrix, CvMat* distortion_params,
                                   std::vector<outlet_t>& outlets, const outlet_template_t& outlet_templ,
                                   const char* output_path, const char* filename);

void features2outlets_2x1(const std::vector<feature_t>& features, std::vector<outlet_t>& outlets);

void calc_outlet_3d_coord_2x2(CvMat* intrinsic_matrix, const outlet_template_t& outlet_templ, std::vector<outlet_t>& outlets);


#endif //_OUTLET_DETECTOR_H
