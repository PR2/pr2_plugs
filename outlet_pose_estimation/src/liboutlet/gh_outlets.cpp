/*
 *  gh_outlets.cpp
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 2/10/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include <highgui.h>
#include "outlet_pose_estimation/detail/gh_outlets.h"
#include "outlet_pose_estimation/detail/outlet_model.h"
#include "outlet_pose_estimation/detail/features.h"
#include "outlet_pose_estimation/detail/one_way_outlets.h"

using namespace std;

void detect_outlets_gh(IplImage* img, const outlet_template_t& outlet_template, vector<outlet_t>& outlets, IplImage* color_image, 
    const char* output_path, const char* output_filename)
{
    vector<feature_t> test_points;
    GetHoleFeatures(img, test_points);
    
#if defined(_VERBOSE)
    IplImage* temp = cvCloneImage(color_image);
    DrawFeatures(temp, test_points);
    
#if 0
    cvNamedWindow("1", 1);
    cvShowImage("1", temp);
    cvWaitKey(0);
#endif
    
#if defined(_SAVE_VERBOSE)
    cvSaveImage("features.jpg", temp);
#endif //_SAVE_VERBOSE
    cvReleaseImage(&temp);
#endif //_VERBOSE
    
    vector<feature_t> template_points = outlet_template.get_one_way_descriptor_base()->_GetLabeledFeatures();
    // reset class_id labels for both template and test points
    for(size_t i = 0; i < test_points.size(); i++) test_points[i].class_id = 0;
    matchOutlets(test_points, outlet_template, template_points, outlets);
    
#if defined(_SAVE_VERBOSE)
    IplImage* img1 = cvCloneImage(color_image);
    draw_outlets(img1, outlets);
    char test_image_filename[1024];
    sprintf(test_image_filename, "%s/outlets/%s", output_path, output_filename);
    cvSaveImage(test_image_filename, img1);
#endif //_SAVE_VERBOSE
    
}
