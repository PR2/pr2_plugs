/*
 *  features.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 4/23/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#ifndef _OUTLET_DETECTION_FEATURES_H
#define _OUTLET_DETECTION_FEATURES_H

#include <vector>
#include <cv.h>

#include "one_way_descriptor_base.h"

inline CvPoint operator -(CvPoint p1, CvPoint p2)
{
    return cvPoint(p1.x - p2.x, p1.y - p2.y);
};

inline float length(CvPoint p)
{
    return sqrt(float(p.x*p.x) + p.y*p.y);
};

inline cv::Point2f operator -(cv::Point2f p1, cv::Point2f p2)
{
    return cv::Point2f(p1.x - p2.x, p1.y - p2.y);
};

inline cv::Point2f operator -(cv::Point2f p1, CvPoint p2)
{
    return cv::Point2f(p1.x - p2.x, p1.y - p2.y);
};

inline cv::Point2f operator -(CvPoint p1, cv::Point2f p2)
{
    return cv::Point2f(p1.x - p2.x, p1.y - p2.y);
};

inline float length(cv::Point2f p)
{
    return sqrt(float(p.x*p.x) + p.y*p.y);
};


#if 0
struct feature_t
{
    CvPoint center;
    float scale;
    int part_id;

    ~feature_t() {};
    feature_t(CvPoint _center = cvPoint(-1, -1), float _scale = 1, int _part_id = -1)
    {
        center = _center;
        scale = _scale;
        part_id = _part_id;
    };
};
#else

/*void operator =(const Point2f& src, CvPoint& dst)
{
    dst = cvPoint((int)src.x, (int)src.y);
};*/
#endif

void GetSURFFeatures(IplImage* src, std::vector<feature_t>& features);
void GetStarFeatures(IplImage* src, std::vector<feature_t>& features);
void GetHarrisFeatures(IplImage* src, std::vector<feature_t>& features);
void GetHoleFeatures(IplImage* src, std::vector<feature_t>& features, float hole_contrast = 1.1f);
//void GetHoleFeatures(IplImage* src, std::vector<feature_t>& features, float hole_contrast = 1.1f);

void DrawFeatures(IplImage* img, const std::vector<feature_t>& features);
void FilterFeatures(std::vector<feature_t>& features, float min_scale, float max_scale);

void SelectNeighborFeatures(std::vector<feature_t>& features, const std::vector<feature_t>& voc);

//namespace cv
//{
//template<> inline void Ptr<IplImage>::delete_obj()
//{ cvReleaseImage(&obj); }
//}

int LoadFeatures(const char* filename, std::vector<std::vector<feature_t> >& features, std::vector<IplImage*>& images);
void LoadTrainingFeatures(CvOneWayDescriptorObject& descriptors, const char* train_image_filename_object,
    const char* train_image_filename_background);

IplImage* loadImageRed(const char* filename);

// helper function for running hough transform over several scales
void ScaleFeatures(const std::vector<feature_t>& src, std::vector<feature_t>& dst, float scale);

void ApplyGamma(IplImage* img, float gamma = 4.0f);

void FilterFeaturesOnEdges(const IplImage* img, const std::vector<feature_t>& src_features, std::vector<feature_t>& dst_features, int max_edge_dist = 5, int min_contour_size = 15);


#endif // _FEATURES_H
