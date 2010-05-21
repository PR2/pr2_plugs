/*
 *  gh_outlets.h
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 2/10/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include <cv.h>
#include <outlet_pose_estimation/detail/outlet_model.h>
#include <outlet_pose_estimation/detail/outlet_tuple.h>

void detect_outlets_gh(IplImage* img, const outlet_template_t& outlet_template, std::vector<outlet_t>& outlets, IplImage* color_image, 
                       const char* output_path, const char* output_filename);
