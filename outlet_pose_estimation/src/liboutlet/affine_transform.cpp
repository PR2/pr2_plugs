/*
 *  affine_transform.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_pose_estimation/detail/affine_transform.h"
#include "outlet_pose_estimation/detail/one_way_descriptor_base.h"

using namespace std;

void cvmSet6(CvMat* m, int row, int col, float val1, float val2, float val3, float val4, float val5, float val6)
{
    cvmSet(m, row, col, val1);
    cvmSet(m, row, col + 1, val2);
    cvmSet(m, row, col + 2, val3);
    cvmSet(m, row, col + 3, val4);
    cvmSet(m, row, col + 4, val5);
    cvmSet(m, row, col + 5, val6);
}

void FindAffineTransform(const std::vector<CvPoint>& p1, const std::vector<CvPoint>& p2, CvMat* affine)
{
    int eq_num = 2*(int)p1.size();
    CvMat* A = cvCreateMat(eq_num, 6, CV_32FC1);
    CvMat* B = cvCreateMat(eq_num, 1, CV_32FC1);
    CvMat* X = cvCreateMat(6, 1, CV_32FC1);

    for(int i = 0; i < (int)p1.size(); i++)
    {
        cvmSet6(A, 2*i, 0, p1[i].x, p1[i].y, 1, 0, 0, 0);
        cvmSet6(A, 2*i + 1, 0, 0, 0, 0, p1[i].x, p1[i].y, 1);
        cvmSet(B, 2*i, 0, p2[i].x);
        cvmSet(B, 2*i + 1, 0, p2[i].y);
    }

    cvSolve(A, B, X, CV_SVD);

    cvmSet(affine, 0, 0, cvmGet(X, 0, 0));
    cvmSet(affine, 0, 1, cvmGet(X, 1, 0));
    cvmSet(affine, 0, 2, cvmGet(X, 2, 0));
    cvmSet(affine, 1, 0, cvmGet(X, 3, 0));
    cvmSet(affine, 1, 1, cvmGet(X, 4, 0));
    cvmSet(affine, 1, 2, cvmGet(X, 5, 0));

    cvReleaseMat(&A);
    cvReleaseMat(&B);
    cvReleaseMat(&X);
}

void MapVectorAffine(const std::vector<CvPoint>& p1, std::vector<CvPoint>& p2, CvMat* transform)
{
    float a = cvmGet(transform, 0, 0);
    float b = cvmGet(transform, 0, 1);
    float c = cvmGet(transform, 0, 2);
    float d = cvmGet(transform, 1, 0);
    float e = cvmGet(transform, 1, 1);
    float f = cvmGet(transform, 1, 2);

    for(int i = 0; i < (int)p1.size(); i++)
    {
        float x = a*p1[i].x + b*p1[i].y + c;
        float y = d*p1[i].x + e*p1[i].y + f;
        p2.push_back(cvPoint(x, y));
    }
}

void MapFeaturesAffine(const std::vector<feature_t>& features, std::vector<feature_t>& mapped_features,
    CvMat* transform)
{
    float a = cvmGet(transform, 0, 0);
    float b = cvmGet(transform, 0, 1);
    float c = cvmGet(transform, 0, 2);
    float d = cvmGet(transform, 1, 0);
    float e = cvmGet(transform, 1, 1);
    float f = cvmGet(transform, 1, 2);

    for(int i = 0; i < (int)features.size(); i++)
    {
        float x = a*features[i].pt.x + b*features[i].pt.y + c;
        float y = d*features[i].pt.x + e*features[i].pt.y + f;
        mapped_features.push_back(feature_t(cvPoint(x, y), features[i].size, features[i].class_id));
    }
}

float CalcAffineReprojectionError(const std::vector<CvPoint>& p1, const std::vector<CvPoint>& p2,
    CvMat* transform)
{
    vector<CvPoint> mapped_p1;
    MapVectorAffine(p1, mapped_p1, transform);
    float error = 0;
    for(int i = 0; i < (int)p2.size(); i++)
    {
        float l = length(p2[i] - mapped_p1[i]);
        error += l*l;
    }

    error /= p2.size();

    return error;
}
