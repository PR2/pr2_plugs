/*
 *  one_way_outlets.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_ONE_WAY_OUTLET)
#define _ONE_WAY_OUTLET

#include <vector>
#include <cv.h>
#include "features.h"
#include "one_way_descriptor.h"
#include "one_way_descriptor_base.h"
#include "outlet_model.h"


void detect_outlets_2x1_one_way(IplImage* img, const CvOneWayDescriptorObject* descriptors,
                                std::vector<feature_t>& features, IplImage* color,
                                const char* output_path = 0, const char* output_filename = 0);


void detect_outlets_one_way(IplImage* test_image, const outlet_template_t& outlet_template,
                            std::vector<outlet_t>& holes, IplImage* color_image,
                            const char* output_path, const char* output_filename,float* scale_ranges = 0);

float matchOutlets(const std::vector<KeyPointEx>& test_points, const outlet_template_t& outlet_template,
                  const std::vector<KeyPointEx>& template_points, std::vector<outlet_t>& outlets);

//Calculates more accurate outlet position by following algorithm: for each outlet point we select the closest feature point (with distance less than accuracy parameter otherwise we say that there is no the closest feature)
// Then we calculates affine transform between train features and selected features and apply found transform to the train outlet
// If we cannot do this then we clear outlet that means there is no outlet on the image
// INPUT: features - all found features on the image
//		  train_features - train outlet
//		  src_outlet - test outlet for which we try to calculate exact location
//		  accuracy - max distance for feature selecting
//		  useSecondAttraction - try to set correspondence between image features and outlet after mapping
// OUTPUT: dst_outlet
//		   reprojectionError - reprojection error for affine transform
void calcExactLocation(std::vector<feature_t>& features,const std::vector<feature_t>& train_features, std::vector<feature_t>& src_outlet, std::vector<feature_t>& dst_outlet, float& reprojectionError, int accuracy = 10, bool useSecondAttraction = true);

void convertFeaturesToOutlet(const std::vector<feature_t>& res_features, std::vector<outlet_t>& holes, IplImage* resImage=0);
void convertFeaturesToOutlet(const std::vector<feature_t>& res_features, const std::vector<bool>& is_detected, std::vector<outlet_t>& holes);

#endif //_ONE_WAY_OUTLET
