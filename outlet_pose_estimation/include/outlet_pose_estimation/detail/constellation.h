/*
 *  constellation.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/3/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_CONSTELLATION_H)
#define _CONSTELLATION_H

#include <vector>
#include <cv.h>
#include "outlet_pose_estimation/detail/features.h"
#include "outlet_pose_estimation/detail/one_way_descriptor_base.h"

float calc_set_std(const std::vector<feature_t>& features, const std::vector<int>& indices = std::vector<int>());
void DetectObjectConstellation(const std::vector<feature_t>& train, const std::vector<feature_t>& input, CvMat* homography, std::vector<int>& indices);
void InferMissingObjects(const std::vector<feature_t>& train, const std::vector<feature_t>& input, CvMat* homography, const std::vector<int>& indices, 
                         std::vector<feature_t>& full);
void FilterOutletFeatures(const std::vector<feature_t>& src_features, std::vector<feature_t>& dst_features, float max_dist);
void FilterOutletFeatures(const std::vector<feature_t>& src_features, std::vector<feature_t>& dst_features, std::vector<int>& dst_indexes, float max_dist);

void ClusterOutletFeatures(const std::vector<feature_t>& src_features, std::vector<feature_t>& clusters, float max_dist);
void SelectNeighborFeatures(const std::vector<feature_t>& src_features, CvPoint center, std::vector<feature_t>& dst_features, float max_dist);



#endif // _CONSTELLATION_H
