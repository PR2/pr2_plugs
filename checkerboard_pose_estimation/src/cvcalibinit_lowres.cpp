/*
 *  plug.cpp
 *
 *
 *  Created by Victor  Eruhimov on 9/5/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "checkerboard_pose_estimation/cvcalibinit_lowres.h"

#include <stdio.h>
#include <highgui.h>

#include <outlet_pose_estimation/detail/features.h>

using namespace std;

//#define _DEBUG_WINDOWS

struct chessboard_feature_t : public feature_t
{
    int idx1, idx2;
};

void SelectNeighborFeatures(const vector<feature_t>& features, vector<feature_t>& neighbors, cv::Point2f point, float max_dist)
{
    neighbors.resize(0);
    for(int i = 0; i < (int)features.size(); i++)
    {
        if(length(features[i].pt - point) < max_dist)
        {
            neighbors.push_back(features[i]);
        }
    }
}

template <class T>
int Find1NN(const vector<T>& features, cv::Point2f point, int exclude_point = 0)
{
    int min_idx = -1;
    float min_dist = 1e10;
    const float min_flt_dist = 1e-5;
    for(size_t i = 0; i < features.size(); i++)
    {
        if(exclude_point && norm(features[i].pt - point) < min_flt_dist)
            continue;
        float dist = length(features[i].pt - point);
        if(dist < min_dist)
        {
            min_dist = dist;
            min_idx = i;
        }
    }

    return min_idx;
}

int find(const vector<int>& indices, int idx)
{
    for(size_t j = 0; j < indices.size(); j++)
    {
        if(indices[j] == idx)
        {
            return j;
        }
    }

    return -1;
}

int Find1NNEx(const vector<feature_t>& features, cv::Point2f point, const vector<int>& exclude_points)
{
    int min_idx = -1;
    float min_dist = 1e10;
    const float min_flt_dist = 1e-5;
    for(size_t i = 0; i < features.size(); i++)
    {
        if(norm(features[i].pt - point) < min_flt_dist)
            continue;
        if(find(exclude_points, i) >= 0) continue;

        float dist = length(features[i].pt - point);
        if(dist < min_dist)
        {
            min_dist = dist;
            min_idx = i;
        }
    }

    return min_idx;
}

int Find2NNPerp(const vector<feature_t>& features, cv::Point2f point, cv::Point2f dir)
{
    int idx = -1;
    float min_dist = 1e10;
    float dir_norm = norm(dir);
    const float min_perp_dist = dir_norm*0.5f;

    for(size_t i = 0; i < features.size(); i++)
    {
        if(norm(features[i].pt - point) < 1e-5)
            continue;
        cv::Point2f pdir = features[i].pt - point;
        float perp_dist = norm(pdir - dir*pdir.dot(dir)*(1.0f/(dir_norm*dir_norm)));
        if(perp_dist < min_perp_dist) continue; // filter out points on the dir line

        float dist = norm(pdir);
        if(dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }

    return idx;
}

int CountPoints(const vector<feature_t>& features, cv::Point2f point, cv::Point2f dir,
                int dir_sign = 0, int* border_point_idx = 0)
{
    if(dir_sign == 0)
    {
        int count1 = CountPoints(features, point, dir, 1);
        int count2 = CountPoints(features, point, dir, -1);
        return count1 + count2;
    }

    int count = 1;
    float dir_norm = norm(dir);
    const float min_dist = dir_norm*0.2f;
    for(;;count++)
    {
        cv::Point2f new_point = point + dir*float(count*dir_sign);
        int idx = Find1NN(features, new_point);
        cv::Point2f offset = new_point - features[idx].pt;

        if(norm(offset) > min_dist)
        {
            break;
        }
        else
        {
            if(border_point_idx) *border_point_idx = idx;
        }
    }

    return count - 1;
}

int IsBorderPoint(const vector<feature_t>& features, cv::Point2f point, cv::Point2f dir_border, cv::Point2f dir_second)
{
    cv::Point2f dir1 = (dir_border - dir_second)*0.5f;
    cv::Point2f dir2 = (dir_border + dir_second)*0.5f;
    float min_dist = 0.2f*MAX(norm(dir1), norm(dir2));

    int idx1 = Find1NN(features, point + dir1);
    int idx2 = Find1NN(features, point + dir2);
    cv::Point2f offset1 = features[idx1].pt - point - dir1;
    cv::Point2f offset2 = features[idx2].pt - point - dir2;
    if(norm(offset1) > min_dist && norm(offset2) > min_dist)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

bool helper_corner_less(chessboard_feature_t f1, chessboard_feature_t f2)
{
    return f1.angle < f2.angle;
}

float calc_corner_weight(const cv::Point2f& point, const cv::Point2f& origin, const cv::Point2f& dir1, const cv::Point2f& dir2,
            float weight_coeff)
{
    cv::Point2f cp = point - origin;
    cv::Point2f corner_pointm = cv::Point2f(-cp.y, cp.x);
    float sprod1 = corner_pointm.dot(dir2)/(- dir2.x*dir1.y + dir2.y*dir1.x)*2;//*norm(dir1);
    float sprod2 = corner_pointm.dot(dir1)/(dir2.x*dir1.y - dir2.y*dir1.x)*2;//*norm(dir2);
    //                            printf("cpm = %f,%f, sprod1 = %f, sprod2 = %f\n",
    //                                   corner_point.pt.x*2, corner_point.pt.y*2, sprod1, sprod2);
    sprod1 = round(sprod1);
    sprod2 = round(sprod2);

    float weight = sprod2*weight_coeff + sprod1;
    return weight;
}

void ShowFeatures(IplImage* img, const vector<feature_t>& features)
{
    IplImage* test = cvCloneImage(img);

    for(size_t i = 0; i < features.size(); i++)
    {
        cvCircle(test, features[i].pt, features[i].size, cvScalar(255));
        printf("feature %d: %f,%f\n", (int)i, features[i].pt.x, features[i].pt.y);
    }
    cvNamedWindow("1", 1);
    cvShowImage("1", test);
    cvWaitKey(0);

    cvSaveImage("features.jpg", test);

    cvReleaseImage(&test);
}

void FilterOutliers(vector<chessboard_feature_t>& corners, cv::Point2f dir1, cv::Point2f dir2, float min_dist)
{
    vector<chessboard_feature_t> corners_filtered;
    for(size_t i = 0; i < corners.size(); i++)
    {
        cv::Point2f p = corners[i].pt;
        int count_neighbors = 0;

        for(int sign2 = -1; sign2 <= 1; sign2 += 2)
        {
            for(int sign1 = -1; sign1 <= 1; sign1 += 2)
            {
                int _sign1 = (sign1 + sign2)/2;
                int _sign2 = (sign1 - sign2)/2;
                cv::Point2f pnn = p + dir1*float(_sign1) + dir2*float(_sign2);
                int idx = Find1NN(corners, pnn);
                cv::Point2f offset = corners[idx].pt - pnn;
                if(norm(offset) < min_dist)
                {
                    count_neighbors++;
                }
            }
        }

        if(count_neighbors > 1)
        {
            corners_filtered.push_back(corners[i]);
        }
    }

    corners = corners_filtered;
}

void updateSpanVector(vector<chessboard_feature_t>::const_iterator it_begin, vector<chessboard_feature_t>::const_iterator it_end,
                    vector<chessboard_feature_t>::const_iterator& it_origin, cv::Point2f& dir)
{
    vector<chessboard_feature_t>::const_iterator it_min = it_end, it_max = it_end;
    float min_weight = 1e10;
    float max_weight = -1e10;
    cv::Point2f origin = it_begin->pt;
    for(vector<chessboard_feature_t>::const_iterator it = it_begin; it != it_end; it++)
    {
        cv::Point2f offset = it->pt - origin;
        float weight = dir.dot(offset);
        if(weight < min_weight)
        {
            min_weight = weight;
            it_min = it;
        }
        if(weight > max_weight)
        {
            max_weight = weight;
            it_max = it;
        }
    }

    dir = it_max->pt - it_min->pt;
    it_origin = it_min;
}

float sortSpannedFeatures(vector<chessboard_feature_t>::iterator it_begin, vector<chessboard_feature_t>::iterator it_end,
    cv::Point2f origin, cv::Point2f dir)
{
    float max_dist = 0;
    for(vector<chessboard_feature_t>::iterator it = it_begin; it != it_end; it++)
    {
        cv::Point2f p = it->pt - origin;
        cv::Point2f p_dir = dir*p.dot(dir)*(1.0f/(dir.dot(dir)));
        it->angle = p.dot(dir)/sqrt(dir.dot(dir));
        float dist = sqrt((p - p_dir).dot(p - p_dir));
        max_dist = MAX(max_dist, dist);
    }

    sort(it_begin, it_end, helper_corner_less);

    return max_dist;
}

int CountBorderPoints(const vector<feature_t>& features, cv::Point2f origin, cv::Point2f dir)
{
    int counts[2] = {0, 0};
    for(size_t i = 0; i < features.size(); i++)
    {
        cv::Point2f offset = features[i].pt - origin;
        float prod = offset.x*dir.y - offset.y*dir.x;
        counts[prod > 0]++;
    }

    return MIN(counts[0], counts[1]);
}

int cvFindChessboardCornersLowres(IplImage* img, CvSize size, CvPoint2D32f* corners, int* corner_count)
{
    vector<feature_t> features;
    const float contrast = 1.2f;
    IplImage* smoothed = cvCloneImage(img);
    cvSmooth(img, smoothed);
    GetHoleFeatures(smoothed, features, contrast);
    cvReleaseImage(&smoothed);

#if defined(_DEBUG_WINDOWS)
    ShowFeatures(img, features);
#endif
    int board_length = MAX(size.width, size.height);
    float weight_coeff = board_length*1.0f;

    const float max_square_size = 20.0;

    for(size_t i = 0; i < features.size(); i++)
    {
        vector<feature_t> neighbors;
        SelectNeighborFeatures(features, neighbors, features[i].pt, max_square_size*board_length*sqrt(2.0f));
//        if(neighbors.size() < size.width*size.height) continue;
//        printf("Neighbors number %d\n", neighbors.size());
        if(neighbors.size() < (size_t)(size.width - 1)*(size.height - 1))
        {
            // this is not a chessboard point
            continue;
        }
        
        if(abs(features[i].pt.x - 413) < 5 && abs(features[i].pt.y - 214) < 5)
        {
            int w = 1;
        }

        int idx1 = Find1NN(neighbors, features[i].pt, 1);
        assert(idx1 >= 0);
        cv::Point2f dir1_diag = neighbors[idx1].pt - features[i].pt;

        vector<int> exclude_points;
        exclude_points.push_back(idx1);
        int idx1m = Find1NN(neighbors, features[i].pt - dir1_diag, 1);
        if(idx1m >= 0)
        {
            exclude_points.push_back(idx1m);
        }
        for(int k = 0; k < 3; k++)
        {
            int idx2 = Find1NNEx(neighbors, features[i].pt, exclude_points);

            //        int idx2 = Find2NNPerp(neighbors, features[i].pt, dir1_diag);
            if(idx2 < 0) continue;
            exclude_points.push_back(idx2);
            cv::Point2f dir2_diag = neighbors[idx2].pt - features[i].pt;

            // we have found diagonal directions, now let's convert them to board axes
            cv::Point2f dir1 = dir1_diag + dir2_diag;
            cv::Point2f dir2 = dir1_diag - dir2_diag;

            if(length(dir1) < 1.0f || length(dir2) < 1.0f) continue;

            int border1plus = -1, border1minus = -1, border2plus = -1, border2minus = -1;
            int count1plus = CountPoints(neighbors, features[i].pt, dir1, 1, &border1plus);
            int count1minus = CountPoints(neighbors, features[i].pt, dir1, -1, &border1minus);
            int count2plus = CountPoints(neighbors, features[i].pt, dir2, 1, &border2plus);
            int count2minus = CountPoints(neighbors, features[i].pt, dir2, -1, &border2minus);
            //        printf("%d,%d,%d,%d\n", border1plus, border1minus, border2plus, border2minus);
            int count1 = count1plus + count1minus + 1;
            int count2 = count2plus + count2minus + 1;

            int count_corner1 = 2*count1;
            // test the borders
            cv::Point2f point1plus = features[i].pt + dir1*float(count1plus);
            if(IsBorderPoint(neighbors, point1plus, dir1, dir2))
            {
                count_corner1--;
            }
            cv::Point2f point1minus = features[i].pt - dir1*float(count1minus);
            if(IsBorderPoint(neighbors, point1minus, -dir1, dir2))
            {
                count_corner1--;
            }

            int count_corner2 = 2*count2;
            cv::Point2f point2plus = features[i].pt + dir2*float(count2plus);
            if(IsBorderPoint(neighbors, point2plus, dir2, dir1))
            {
                count_corner2--;
            }
            cv::Point2f point2minus = features[i].pt - dir2*float(count2minus);
            if(IsBorderPoint(neighbors, point2minus, -dir2, dir1))
            {
                count_corner2--;
            }


            if(border1plus >= 0 && border1minus >= 0)
            {
                dir1 = (neighbors[border1plus].pt - neighbors[border1minus].pt)*(1.0f/(count1 - 1));
            }
            if(border2plus >= 0 && border2minus >= 0)
            {
                dir2 = (neighbors[border2plus].pt - neighbors[border2minus].pt)*(1.0f/(count2 - 1));
            }
            if(count_corner1 == size.height && count_corner2 == size.width)
            {
                count_corner1 = size.width;
                count_corner2 = size.height;
                cv::Point2f _dir = dir1;
                dir1 = dir2;
                dir2 = _dir;
            }
            else if(count_corner1 != size.width || count_corner2 != size.height)
            {
                continue;
            }

            if(dir1.x < 0) dir1 = -dir1;
            if(dir2.y < 0) dir2 = -dir2;
            float min_dist = 0.3*MAX(norm(dir1), norm(dir2));

            //printf("dir1 = %f,%f, dir2 = %f,%f\n", dir1.x, dir1.y, dir2.x, dir2.y);

            // find the corners
            vector<chessboard_feature_t> corner_points;
            for(size_t j1 = 0; j1 < neighbors.size(); j1++)
            {
                cv::Point2f p = neighbors[j1].pt;

                for(int sign2 = -1; sign2 <= 1; sign2 += 2)
                {
                    for(int sign1 = -1; sign1 <= 1; sign1 += 2)
                    {
                        cv::Point2f pnn = p + (dir1*float(sign1) + dir2*float(sign2))*0.5f;
                        int idx = Find1NN(neighbors, pnn);
                        cv::Point2f offset = neighbors[idx].pt - pnn;
                        if(norm(offset) < min_dist)
                        {
                            chessboard_feature_t corner_point;
                            corner_point.pt = (p + pnn)*0.5f;
                            if(corner_points.size() == 0)
                            {
                                corner_point.angle = 0;
                            }
                            else
                            {
                                corner_point.angle = calc_corner_weight(corner_point.pt, corner_points[0].pt,
                                                                        dir1, dir2, weight_coeff);
                            }
                            // check if this corner is already present
                            int corner_idx = Find1NN(corner_points, corner_point.pt);
                            if(corner_idx >= 0)
                            {
                                cv::Point2f offset = corner_point.pt - corner_points[corner_idx].pt;
                                if(norm(offset) < min_dist)
                                {
                                    continue;
                                }
                            }

                            corner_point.idx1 = j1;
                            corner_point.idx2 = idx;
                            corner_points.push_back(corner_point);
                            //                        printf("Added point %f,%f, weight %f, sprod1 = %f, sprod2 = %f\n",
                            //                               corner_point.pt.x*2, corner_point.pt.y*2, corner_point.angle,
                            //                                sprod1, sprod2);
                        }
                    }
                }
            }

#if defined(_DEBUG_WINDOWS)
            IplImage* test = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
            cvCvtColor(img, test, CV_GRAY2BGR);

            for(size_t i = 0; i < corner_points.size(); i++)
            {
                cvCircle(test, corner_points[i].pt, 3, cvScalar(255, 0, 0));
            }
            cvNamedWindow("1", 1);
            cvShowImage("1", test);
            cvWaitKey(0);
            
            cvSaveImage("test.png", test);

            cvReleaseImage(&test);
#endif //_DEBUG_WINDOWS

            FilterOutliers(corner_points, dir1*0.5f, dir2*0.5f, min_dist);

#if defined(_DEBUG_WINDOWS)
            test = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
            cvCvtColor(img, test, CV_GRAY2BGR);

            for(size_t i = 0; i < corner_points.size(); i++)
            {
                cvCircle(test, corner_points[i].pt, 3, cvScalar(0, 255, 0));
            }
            cvNamedWindow("1", 1);
            cvShowImage("1", test);
            cvWaitKey(0);

            cvReleaseImage(&test);
#endif //_DEBUG_WINDOWS


            if(corner_points.size() != size.width*size.height)
            {
                continue;
            }

            sort(corner_points.begin(), corner_points.end(), helper_corner_less);

#if 1
            vector<chessboard_feature_t>::const_iterator it_origin;
            updateSpanVector(corner_points.begin(), corner_points.begin() + 4, it_origin, dir1);
            int origin_idx = it_origin - corner_points.begin();

            // now recalculate the weights and sort again

            //        dir1 = (corner_points[size.width - 1].pt - corner_points[0].pt);
            //        dir2 = (corner_points[(size.height - 1)*size.width].pt - corner_points[0].pt);
            dir1 = dir1*(1.0f/(size.width - 1));
            //        dir2 = dir2*(1.0f/(size.height - 1));
            dir2 = dir2*0.5f;
//            if(dir1.y < 0) dir1 = -dir1;
//            if(dir2.x > 0) dir2 = -dir2;

            if(size.height % 2 == 0)
            {
                if(dir1.x < 0) dir1 = -dir1;
                //if(dir2.y < 0) dir2 = -dir2;
                if(dir1.x*dir2.y - dir1.y*dir2.x < 0) dir2 = -dir2;
            }
            else
            {
                cv::Point2f black_diag = neighbors[corner_points[0].idx1].pt -
                    neighbors[corner_points[0].idx2].pt;
                if(black_diag.dot(dir1) < 0) black_diag = -black_diag;
                if(black_diag.dot(dir2) < 0) dir2 = -dir2;
                if(dir1.x*dir2.y - dir1.y*dir2.x < 0) dir1 = -dir1;
//                printf("oirgin = %f,%f, black_diag = %f,%f, dir1 = %f,%f, dir2 = %f,%f\n",
//                    corner_points[0].pt.x, corner_points[0].pt.y, black_diag.x, black_diag.y,
//                    dir1.x, dir1.y, dir2.x, dir2.y);
            }


            corner_points[origin_idx].angle = 0.0f;
            for(size_t j1 = 0; j1 < corner_points.size(); j1++)
            {
                if(j1 == origin_idx) continue;
                corner_points[j1].angle = calc_corner_weight(corner_points[j1].pt,
                                                             corner_points[origin_idx].pt, dir1, dir2, weight_coeff);
            }

            sort(corner_points.begin(), corner_points.end(), helper_corner_less);

            // now reorder inside each line
            int valid_flag = 1;
            cv::Point2f origin = it_origin->pt;
            for(int jh = 0; jh < size.height; jh++)
            {
                float offset = sortSpannedFeatures(corner_points.begin() + size.width*jh,
                                    corner_points.begin() + size.width*(jh + 1), origin, dir1);
                if(offset > min_dist)
                {
                    valid_flag = 0;
                    break;
                }

                // looking for the next line
                cv::Point2f new_origin = origin + dir2;
                int idx = Find1NN(corner_points, new_origin);
                if(length(new_origin - corner_points[idx].pt) > min_dist && jh < size.height - 1)
                {
                    valid_flag = 0;
                    break;
                }

                // found the new line
                origin = corner_points[idx].pt;
            }

            if(!valid_flag)
            {
                //printf("invalid\n");
                continue;
            }

            // test the new origin for ordering
            if(size.height % 2 != 0)
            {
                cv::Point2f black_diag = neighbors[corner_points[0].idx1].pt -
                    neighbors[corner_points[0].idx2].pt;
                if(black_diag.dot(dir1) < 0) black_diag = -black_diag;
                assert(dir1.x*dir2.y - dir1.y*dir2.x > 0);
                if(black_diag.dot(dir2) < 0)
                {
                    continue; // this is not right!
                }

            }
#endif

            for(int jh = 0; jh < size.height; jh++)
            {
                for(int jw = 0; jw < size.width; jw++)
                {
                    corners[jh*size.width + jw] = corner_points[jh*size.width + jw].pt;
                }
            }

            if(corner_count) *corner_count = size.width*size.height;
            //printf("dir1 = %f,%f, dir2 = %f,%f\n", dir1.x, dir1.y, dir2.x, dir2.y);
            //        printf("Neighbors number: %d\n", neighbors.size());

            return 1;
        }
    }

    if(corner_count) *corner_count = 0;
    return 0;
}

