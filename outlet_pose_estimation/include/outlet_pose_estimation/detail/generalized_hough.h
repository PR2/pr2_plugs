//Created by Alexey Latyshev
// Set of functions for Generic Hough Transform (GHT) for outlets detection
#ifndef _G_HOUGH_H
#define _G_HOUGH_H

#include <cv.h>
#include <highgui.h>
#include <vector>
#include "outlet_pose_estimation/detail/features.h"
#include "outlet_pose_estimation/detail/affine_transform.h"
#include "outlet_pose_estimation/detail/outlet_model.h"

// Builds 6-dimension histogram [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
CvSparseMat* buildHoughHist(std::vector<feature_t>& input, const std::vector<feature_t>& train_features, int* hist_size, float** ranges);

//// Calculates maximums of histogram.
//// Returns one of the maximums
//float** getMaxHistValues(const CvSparseMat* hist, int* hist_size);
////Returns all maximums with >= MIN_VOTES
void getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float**& maxs, int& count, int MIN_VOTES);
////Returns all maximums (count number)
// Return value: max votes
int getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float**& maxs, int& count);

// Calculates maximums of histogram.


// Calculates outlet features from given train outlet and affine transform
// Affine transform is array [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
void calcOutletPosition(const std::vector<feature_t>& train_features, float* affine_transform, std::vector<feature_t>& features);

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

// Main function for GeneralizedHough transform
// Output: holes
float generalizedHoughTransform(std::vector<feature_t>& hole_candidates, const std::vector<feature_t>& train_features, int* hist_size, float** ranges, std::vector<outlet_t>& holes, IplImage* ghtImage = NULL, IplImage* resImage = NULL, char* output_path = 0, char* base_filename = 0);

void convertFeaturesToOutlet(const std::vector<feature_t>& res_features, std::vector<outlet_t>& holes, IplImage* resImage=0);
void convertFeaturesToOutlet(const std::vector<feature_t>& res_features, const std::vector<bool>& is_detected, std::vector<outlet_t>& holes);

#endif
