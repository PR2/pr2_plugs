/*
 *  affine_transform.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_AFFINE_TRANSFORM_H)
#define _AFFINE_TRANSFORM_H

#include <vector>

#include <cv.h>

#include "outlet_pose_estimation/detail/features.h"
#include "outlet_pose_estimation/detail/one_way_descriptor_base.h"

void FindAffineTransform(const std::vector<CvPoint>& p1, const std::vector<CvPoint>& p2, CvMat* affine);
void MapVectorAffine(const std::vector<CvPoint>& p1, std::vector<CvPoint>& p2, CvMat* transform);
void MapFeaturesAffine(const std::vector<feature_t>& features, std::vector<feature_t>& mapped_features, CvMat* transform);
float CalcAffineReprojectionError(const std::vector<CvPoint>& p1, const std::vector<CvPoint>& p2, CvMat* transform);


#endif //_AFFINE_TRANSFORM_H
