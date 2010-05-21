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

#endif //_ONE_WAY_OUTLET
