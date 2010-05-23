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


// Main function for GeneralizedHough transform
// Output: holes
float generalizedHoughTransform(std::vector<feature_t>& hole_candidates, const std::vector<feature_t>& train_features, int* hist_size, float** ranges, std::vector<outlet_t>& holes, IplImage* ghtImage = NULL, IplImage* resImage = NULL, char* output_path = 0, char* base_filename = 0);


#endif
