/*
 *  constellation.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/3/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_pose_estimation/detail/constellation.h"
#include "outlet_pose_estimation/detail/outlet_tuple.h"
#include "outlet_pose_estimation/detail/affine_transform.h"

using namespace std;

int iterate_indices(vector<int>& indices, int max_index, int min_valid_indices, int* workspace)
{
    int indices_count = indices.size();
    while(1)
    {
        // increment indices
        for(int i = 0; i < indices_count; i++)
        {
            indices[i]++;
            if(indices[i] < max_index) 
            {
                break;
            }
            else
            {
                indices[i] = -1;
            }    
        }
        
        // now check indices
        int valid_indices = 0;
        for(int j = 0; j < max_index; j++) workspace[j] = 0;
        int repeat_flag = 0;
        
        for(int i = 0; i < indices_count; i++) 
        {
            if(indices[i] >= 0)
            {
                valid_indices++;

                if(++workspace[indices[i]] > 1)
                {
                    repeat_flag = 1;
                }
            }
        }
        
        if(repeat_flag == 0 && valid_indices >= min_valid_indices)
        {
            return 0;
        }
        
        if(valid_indices == 0)
        {
            return -1;
        }
    }
}

int validate_parts(const vector<feature_t>& train, const vector<feature_t>& test, const vector<int>& indices, int* min_part_count)
{
    int part_count[] = {0, 0};
    const int min_valid_parts = 5; // since we use homography matching, almost any 4 points will be matched with low error
    
    for(int i = 0; i < (int)indices.size(); i++)
    {
        if(indices[i] == -1) continue;
        
        if(test[i].class_id != train[indices[i]].class_id)
        {
            return -1;
        }
        
        part_count[train[indices[i]].class_id]++;
    }
    
    if(part_count[0] < min_part_count[0] || part_count[1] < min_part_count[1] || 
       part_count[0] + part_count[1] < min_valid_parts)
    {
        return -1;
    }
    
    return 0;
}

float min_idx(float v1, float v2, int i1, int i2)
{
    if(i1 && i2)
    {
        return MIN(v1, v2);
    }
    else if(i1)
    {
        return v1;
    }
    else return v2;        
}

float max_idx(float v1, float v2, int i1, int i2)
{
    if(i1 && i2)
    {
        return MAX(v1, v2);
    }
    else if(i1)
    {
        return v1;
    }
    else return v2;        
}

int validate_order(const vector<feature_t>& train, const vector<feature_t>& test, const vector<int>& indices)
{
    const int min_offset = 10;
    vector<int> parts_index;
    
    const int parts_number = 6;
    CvPoint centers[parts_number];
    parts_index.assign(parts_number, 0);
    
    for(int i = 0; i < (int)indices.size(); i++)
    {
        if(indices[i] == -1) continue;
        centers[indices[i]] = test[i].pt;
        parts_index[indices[i]] = 1;
    }

    if((centers[0].x > centers[4].x)*parts_index[0]*parts_index[4] || 
       (centers[4].x > centers[1].x)*parts_index[4]*parts_index[1] ||
       (centers[2].x > centers[5].x)*parts_index[2]*parts_index[5] || 
       (centers[5].x > centers[3].x)*parts_index[5]*parts_index[3])
    {
        return -1;
    }
    
    if((min_idx(centers[0].y, centers[1].y, parts_index[0], parts_index[1]) < centers[4].y)*
            (parts_index[0] | parts_index[1])*parts_index[4] || 
        (min_idx(centers[2].y, centers[3].y, parts_index[2], parts_index[3]) < centers[5].y)*
            (parts_index[2] | parts_index[3])*parts_index[5] || 
        (centers[5].y < max_idx(centers[0].y, centers[1].y, parts_index[0], parts_index[1]) + min_offset)*
            (parts_index[0] | parts_index[1])*parts_index[5] ||
       (max_idx(centers[0].y, centers[1].y, parts_index[0], parts_index[1]) + min_offset > 
        min_idx(centers[2].y, centers[3].y, parts_index[2], parts_index[3]))*
       (parts_index[0] | parts_index[1])*(parts_index[2] | parts_index[3]))
    {
        return -1;
    }
    
    return 0;
}

float calc_set_std(const vector<feature_t>& features, const vector<int>& indices)
{
    CvPoint2D32f sum = cvPoint2D32f(0, 0);
    CvPoint2D32f sum2 = cvPoint2D32f(0, 0);
    int count = 0;
    
    if(indices.size() > 0)
    {
        for(int i = 0; i < (int)indices.size(); i++)
        {
            if(indices[i] < 0) continue;
            
            sum.x += features[i].pt.x;
            sum.y += features[i].pt.y;
            sum2.x += features[i].pt.x*features[i].pt.x;
            sum2.y += features[i].pt.y*features[i].pt.y;
            
            count++;
        }
    }
    else
    {
        for(int i = 0; i < (int)features.size(); i++)
        {
            sum.x += features[i].pt.x;
            sum.y += features[i].pt.y;
            sum2.x += features[i].pt.x*features[i].pt.x;
            sum2.y += features[i].pt.y*features[i].pt.y;
        }
        
        count = features.size();
    }
    
    float sigmax = sqrt(sum2.x/count - sum.x*sum.x/(count*count));
    float sigmay = sqrt(sum2.y/count - sum.y*sum.y/(count*count));
    
    return MAX(sigmax, sigmay);
}

float CalcReprojectionError(CvMat* src_points, CvMat* dst_points, CvMat* src_proj_points, CvMat* homography)
{
    cvPerspectiveTransform(src_points, src_proj_points, homography);
    float error = 0.0f;
    for(int i = 0; i < dst_points->rows; i++)
    {
        float* dst_points_ptr = (float*)(dst_points->data.ptr + i*dst_points->step);
        float* src_proj_points_ptr = (float*)(src_proj_points->data.ptr + i*src_proj_points->step);
        error += sqrt((dst_points_ptr[0] - src_proj_points_ptr[0])*(dst_points_ptr[0] - src_proj_points_ptr[0]) + 
                      (dst_points_ptr[1] - src_proj_points_ptr[1])*(dst_points_ptr[1] - src_proj_points_ptr[1]));
    }
    
    error /= dst_points->rows;
    
    return error;
}

void count_parts(const vector<feature_t>& features, int* min_part_count, int parts_number)
{
    int* part_count = new int[parts_number];
    
    for(int i = 0; i < parts_number; i++) part_count[i] = 0;

    for(int i = 0; i < (int)features.size(); i++)
    {
        part_count[features[i].class_id]++;
    }
    
    for(int i = 0; i < parts_number; i++)
    {
        min_part_count[i] = MIN(min_part_count[i], part_count[i]);
    }
    
    delete []part_count;
}

void DetectObjectConstellation(const vector<feature_t>& train, const vector<feature_t>& input, CvMat* homography, vector<int>& indices)
{
    int parts_number = train.size();
    int test_feature_count = input.size();
    indices.resize(test_feature_count);
    
    for(int i = 0; i < test_feature_count; i++) indices[i] = -1;
    
    if(test_feature_count > 9)
    {
        printf("The number of features is %d, exiting...\n", test_feature_count);
        return;
    }
    
    const int min_valid_indices = 4;
    int* workspace = new int[parts_number];
    int ret_flag = 0;
    
    CvMat* _train_points = cvCreateMat(parts_number, 2, CV_32FC1);
    CvMat* _test_points = cvCreateMat(test_feature_count, 2, CV_32FC1);
    
    vector<int> min_indices = indices;
    float min_error = 1e10;
    int min_part_count[] = {3, 1};
    count_parts(input, min_part_count, 2);
    CvMat* min_homography = cvCloneMat(homography);
    
    float train_set_size = calc_set_std(train);
//    printf("Train set size = %f\n", train_set_size);
    const float set_size_factor = 2.0f;
    
//    printf("Starting the parts search...\n");
    while(1)
    {
        ret_flag = iterate_indices(indices, parts_number, min_valid_indices, workspace);
        if(ret_flag == -1)
        {
            break;
        }
        
        if(validate_parts(train, input, indices, min_part_count) == -1)
        {
            continue;
        }
        
        if(validate_order(train, input, indices) == -1)
        {
            continue;
        }
        
        if(calc_set_std(input, indices) > train_set_size*set_size_factor)
        {
            continue;
        }

//        printf("Tested indices %d%d%d%d%d%d\n", indices[0], indices[1], indices[2], indices[3], indices[4], indices[5]);
        
#if defined(_HOMOGRAPHY)
        // fill train and test points
        int point_count = 0;
        for(int i = 0; i < test_feature_count; i++)
        {
            if(indices[i] == -1) continue;
            
            cvmSet(_train_points, point_count, 0, train[indices[i]].center.x);
            cvmSet(_train_points, point_count, 1, train[indices[i]].center.y);
            cvmSet(_test_points, point_count, 0, input[i].center.x);
            cvmSet(_test_points, point_count, 1, input[i].center.y);
            point_count++;
        }
        
        CvMat train_points, test_points;
        CvRect subrect = cvRect(0, 0, 2, point_count);
        cvGetSubRect(_train_points, &train_points, subrect);
        cvGetSubRect(_test_points, &test_points, subrect);
        CvMat src_proj_points;
        cvGetSubRect(_test_points2, &src_proj_points, subrect);

        const double ransacReprojThreshold = 5.0;
        cvFindHomography(&train_points, &test_points, homography, CV_RANSAC, ransacReprojThreshold);
                            
        train_points.cols = 1;
        train_points.type = hack->type;
        test_points.cols = 1;
        test_points.type = hack->type;
        src_proj_points.cols = 1;
        src_proj_points.type = hack->type;
        
        float error = CalcReprojectionError(&train_points, &test_points, &src_proj_points, homography);
#else
        // fill the points
        vector<CvPoint> p1, p2;
        for(int i = 0; i < test_feature_count; i++)
        {
            if(indices[i] == -1) continue;
            
            p1.push_back(train[indices[i]].pt);
            p2.push_back(input[i].pt);
        }
        
        FindAffineTransform(p1, p2, homography);
        
        float error = CalcAffineReprojectionError(p1, p2, homography);
        
#endif //_HOMOGRAPHY
        if(error < min_error)
        {
            min_error = error;
            min_indices = indices;
            cvCopy(homography, min_homography);
        }
        
    }
//    printf("Parts search finished\n");
//    printf("Final set size = %f\n", calc_set_std(input, min_indices));
    
    indices = min_indices;
    cvCopy(min_homography, homography);
    
    delete []workspace;
    cvReleaseMat(&min_homography);
    
    cvReleaseMat(&_train_points);
    cvReleaseMat(&_test_points);
}

void features2points(const vector<feature_t>& features, vector<CvPoint2D32f>& points)
{
    for(int i = 0; i < (int)features.size(); i++)
    {
        points.push_back(cvPoint2D32f(features[i].pt.x, features[i].pt.y));
    }
}

void points2features(const vector<CvPoint2D32f>& points, vector<feature_t>& features)
{
    features.resize(points.size());
    for(int i = 0; i < (int)points.size(); i++)
    {
        features[i].pt = cvPoint(points[i].x, points[i].y);
    }
}

void map_features(const vector<feature_t>& src_features, CvMat* homography, vector<feature_t>& dst_features)
{
    vector<CvPoint2D32f> src_points, dst_points;
    
    features2points(src_features, src_points);
    map_vector_homography(src_points, homography, dst_points);
    dst_features = src_features;
    points2features(dst_points, dst_features);
}

void InferMissingObjects(const vector<feature_t>& train, const vector<feature_t>& input, CvMat* homography, const vector<int>& indices, 
                         vector<feature_t>& full)
{
    int parts_number = train.size();
    
    vector<feature_t> train_mapped;
#if defined(_HOMOGRAPHY)
    map_features(train, homography, train_mapped);
#else
    MapFeaturesAffine(train, train_mapped, homography); //TBD need to unify the transformation or get rid of homography
#endif //_HOMOGRAPHY
    
    // find missing indices
    vector<int> miss_indices;
    miss_indices.assign(parts_number, 0);
    
    full.resize(parts_number);
    for(int i = 0; i < (int)indices.size(); i++)
    {
        if(indices[i] >= 0)
        {
            miss_indices[indices[i]] = 1;
            full[indices[i]] = input[i];
        }
    }
    
    for(int i = 0; i < (int)miss_indices.size(); i++)
    {
        if(miss_indices[i] == 0)
        {
            full[i] = train_mapped[i];
        }
    }
}

void FilterOutletFeatures(const vector<feature_t>& src_features, vector<feature_t>& dst_features, float max_dist)
{
    vector<int> ground_idx;
    // find all ground hole candidates
    for(int i = 0; i < (int)src_features.size(); i++)
    {
        if(src_features[i].class_id == 1)
        {
            ground_idx.push_back(i);
        }
    }
    
    // find all pairs of ground holes that are close enough to each other and filter out one of them
    vector<int> ground_idx_filtered;
    for(int i = 0; i < (int)ground_idx.size(); i++)
    {
        int flag = 0;
        for(int j = 0; j < (int)ground_idx_filtered.size(); j++)
        {
            if(length(src_features[ground_idx[i]].pt - src_features[ground_idx_filtered[j]].pt) < max_dist)
            {
                flag = 1;
                break;
            }
        }
        
        if(flag == 0)
        {
            ground_idx_filtered.push_back(ground_idx[i]);
        }
    }
    
    ground_idx = ground_idx_filtered;
//    printf("Found %d clusters\n", (int)ground_idx.size());
    
    // now filter out all features that are far enough from ground holes
    vector<int> indices;
    indices.assign(src_features.size(), 0);
    for(int i = 0; i < (int)ground_idx.size(); i++)
    {
        for(int j = 0; j < (int)src_features.size(); j++)
        {
            if(length(src_features[j].pt - src_features[ground_idx[i]].pt) < max_dist)
            {
                indices[j] = 1;
            }
        }
    }
    
    // now copy the features
    for(int i = 0; i < (int)src_features.size(); i++)
    {
        if(indices[i])
        {
            dst_features.push_back(src_features[i]);
        }
    }
}

//-----------------------
void FilterOutletFeatures(const vector<feature_t>& src_features, vector<feature_t>& dst_features, vector<int>& dst_indexes, float max_dist)
{
    vector<int> ground_idx;
	dst_indexes.clear();
    // find all ground hole candidates
    for(int i = 0; i < (int)src_features.size(); i++)
    {
        if(src_features[i].class_id == 1)
        {
            ground_idx.push_back(i);
        }
    }
    
    // find all pairs of ground holes that are close enough to each other and filter out one of them
    vector<int> ground_idx_filtered;
    for(int i = 0; i < (int)ground_idx.size(); i++)
    {
        int flag = 0;
        for(int j = 0; j < (int)ground_idx_filtered.size(); j++)
        {
            if(length(src_features[ground_idx[i]].pt - src_features[ground_idx_filtered[j]].pt) < max_dist)
            {
                flag = 1;
                break;
            }
        }
        
        if(flag == 0)
        {
            ground_idx_filtered.push_back(ground_idx[i]);
        }
    }
    
    ground_idx = ground_idx_filtered;
    vector<int> indices;
    indices.assign(src_features.size(), 0);
    for(int i = 0; i < (int)ground_idx.size(); i++)
    {
        for(int j = 0; j < (int)src_features.size(); j++)
        {
            if(length(src_features[j].pt - src_features[ground_idx[i]].pt) < max_dist)
            {
                indices[j] = 1;
            }
        }
    }
    
    // now copy the features
    for(int i = 0; i < (int)src_features.size(); i++)
    {
        if(indices[i])
        {
            dst_features.push_back(src_features[i]);
			dst_indexes.push_back(i);
        }
    }
}
//----------------------------------
void ClusterOutletFeatures(const vector<feature_t>& src_features, vector<feature_t>& clusters, float max_dist)
{
    vector<int> ground_idx;
    // find all ground hole candidates
    for(int i = 0; i < (int)src_features.size(); i++)
    {
        if(src_features[i].class_id == 1)
        {
            ground_idx.push_back(i);
        }
    }
    
    // find all pairs of ground holes that are close enough to each other and filter out one of them
    vector<int> ground_idx_filtered;
    for(int i = 0; i < (int)ground_idx.size(); i++)
    {
        int flag = 0;
        for(int j = 0; j < (int)ground_idx_filtered.size(); j++)
        {
            if(length(src_features[ground_idx[i]].pt - src_features[ground_idx_filtered[j]].pt) < max_dist)
            {
                flag = 1;
                break;
            }
        }
        
        if(flag == 0)
        {
            ground_idx_filtered.push_back(ground_idx[i]);
        }
    }
    
    ground_idx = ground_idx_filtered;
//    printf("Found %d clusters\n", (int)ground_idx.size());
    
    // now copy the features
    for(int i = 0; i < (int)ground_idx.size(); i++)
    {
        clusters.push_back(src_features[ground_idx[i]]);
    }
}

void SelectNeighborFeatures(const vector<feature_t>& src_features, CvPoint center, vector<feature_t>& dst_features, float max_dist)
{
    for(int i = 0; i < (int)src_features.size(); i++)
    {
        if(length(src_features[i].pt - center) < max_dist)
        {
            dst_features.push_back(src_features[i]);
        }
    }
}
