/*
 *  features.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 4/23/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_pose_estimation/detail/features.h"
#include "outlet_pose_estimation/detail/outlet_model.h"
#include <highgui.h>

using namespace std;

void GetSURFFeatures(IplImage* src, vector<feature_t>& features)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* surf_points = 0;
    cvExtractSURF(src, 0, &surf_points, 0, storage, cvSURFParams(512));

    features.clear();
    for(int i = 0; i < surf_points->total; i++)
    {
        CvSURFPoint* point = (CvSURFPoint*)cvGetSeqElem(surf_points, i);
        CvPoint center = cvPoint(point->pt.x, point->pt.y);
        features.push_back(feature_t(center, (float)point->size));
    }

    cvReleaseMemStorage(&storage);
}

void GetStarFeatures(IplImage* src, vector<feature_t>& features)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvSeq* star_points = cvGetStarKeypoints(src, storage, cvStarDetectorParams(45));

    features.clear();
    for(int i = 0; i < star_points->total; i++)
    {
        CvStarKeypoint* keypoint = (CvStarKeypoint*)cvGetSeqElem(star_points, i);
        features.push_back(feature_t(keypoint->pt, (float)keypoint->size));
    }

    cvReleaseMemStorage(&storage);
}

void GetHarrisFeatures(IplImage* src, vector<feature_t>& features)
{
    IplImage* grey = src;
    if(src->nChannels > 1)
    {
        grey = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
        cvCvtColor(src, grey, CV_RGB2GRAY);
    }


    IplImage* eig_img = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_32F, 1);
    IplImage* temp_img = cvCloneImage(eig_img);

    int corner_count = 1024;
    CvPoint2D32f* corners = new CvPoint2D32f[corner_count];
    cvGoodFeaturesToTrack(grey, eig_img, temp_img, corners, &corner_count, .5, 0);//, 0, 3, 1);

    for(int i = 0; i < corner_count; i++)
    {
        features.push_back(feature_t(cvPoint(corners[i].x, corners[i].y), 1.0f));
    }

    if(src->nChannels > 1)
    {
        cvReleaseImage(&grey);
    }
    cvReleaseImage(&eig_img);
    cvReleaseImage(&temp_img);
}

/*void GetHoleFeatures(IplImage* src, vector<feature_t>& features, float hole_contrast)
{
    vector<outlet_feature_t> outlet_features;
    find_outlet_features_fast(src, outlet_features, hole_contrast, 0, 0);
    for(size_t i = 0; i < outlet_features.size(); i++)
    {
        features.push_back(feature_t(feature_center(outlet_features[i]), outlet_features[i].bbox.width));
    }
}*/

void GetHoleFeatures(IplImage* src, vector<feature_t>& features, float hole_contrast)
{
    vector<outlet_feature_t> outlet_features;
    find_outlet_features_fast(src, outlet_features, hole_contrast, 0, 0);
    for(size_t i = 0; i < outlet_features.size(); i++)
    {
        features.push_back(feature_t(feature_center(outlet_features[i]), outlet_features[i].bbox.width));
    }
}

void DrawFeatures(IplImage* img, const vector<feature_t>& features)
{
    for(size_t i = 0; i < features.size(); i++)
    {
        cvCircle(img, features[i].pt, features[i].size, CV_RGB(255, 0, 0), 2);
    }
}

void FilterFeatures(vector<feature_t>& features, float min_scale, float max_scale)
{
    vector<feature_t> selected;
    for(size_t i = 0; i < features.size(); i++)
    {
        if(features[i].size >= min_scale && features[i].size <= max_scale)
        {
            selected.push_back(features[i]);
        }
    }

    features = selected;
}

void SelectNeighborFeatures(vector<feature_t>& features, const vector<feature_t>& voc)
{
    const int max_dist = 10;
    vector<feature_t> filtered;
    for(int i = 0; i < (int)features.size(); i++)
    {
        for(int j = 0; j < (int)voc.size(); j++)
        {
            if(length(features[i].pt - voc[j].pt) < max_dist)
            {
                filtered.push_back(features[i]);
            }
        }
    }

    features = filtered;
}

int CalcFeatures(IplImage* image, vector<vector<feature_t> >& features, vector<IplImage*>& images)
{
    size_t pyr_levels = features.size();
    images.resize(pyr_levels);
    IplImage* image_features = cvCloneImage(image);

    for(size_t i = 0; i < features.size(); i++)
    {
        images[i] = image_features;
        GetHoleFeatures(image_features, features[i]);
        IplImage* temp_image = cvCreateImage(cvSize(image_features->width/2, image_features->height/2), IPL_DEPTH_8U, 1);
        cvPyrDown(image_features, temp_image);
        image_features = temp_image;
    }
    cvReleaseImage(&image_features);

    int feature_count = 0;

    for(size_t i = 0; i < pyr_levels; i++)
    {
        feature_count += features[i].size();
    }

    cvReleaseImage(&image);

    return feature_count;
}

static const float template_gamma = 2.0f;//1.2f;
int LoadFeatures(const char* filename, vector<vector<feature_t> >& features, vector<IplImage*>& images)
{
    IplImage* image = loadImageRed(filename);
    ApplyGamma(image, template_gamma);

    int feature_count = CalcFeatures(image, features, images);

    return feature_count;
}

IplImage* loadImageRed(const char* filename)
{
    IplImage* temp = cvLoadImage(filename);
    IplImage* red = cvCreateImage(cvSize(temp->width, temp->height), IPL_DEPTH_8U, 1);
    cvSetImageCOI(temp, 3);
    cvCopy(temp, red);
    cvReleaseImage(&temp);

#if defined(_SCALE_IMAGE_2)
    IplImage* red2 = cvCreateImage(cvSize(red->width/2, red->height/2), IPL_DEPTH_8U, 1);
    cvResize(red, red2);
    cvReleaseImage(&red);
    red = red2;
#endif //_SCALE_IMAGE_2

    return red;
}

void ReleaseImageVector(vector<IplImage*>& images)
{
    for(size_t i = 0; i < images.size(); i++)
    {
        cvReleaseImage(&images[i]);
    }
}

void LoadTrainingFeatures(CvOneWayDescriptorObject& descriptors, const char* train_image_filename_object, const char* train_image_filename_background)
{
#if defined(_RED)
    IplImage* train_image_object = loadImageRed(train_image_filename_object);
    IplImage* train_image_background = loadImageRed(train_image_filename_background);
#else
    IplImage* train_image_object = cvLoadImage(train_image_filename_object, CV_LOAD_IMAGE_GRAYSCALE);
    IplImage* train_image_background = cvLoadImage(train_image_filename_background, CV_LOAD_IMAGE_GRAYSCALE);
#endif

#if 1
    ApplyGamma(train_image_object, template_gamma);
//    ApplyGamma(train_image_background);
#endif

#if 0
    IplImage* train_image_object_up = cvCreateImage(cvSize(train_image_object->width, train_image_object->height), IPL_DEPTH_8U, train_image_object->nChannels);
    cvPyrUp(train_image_object, train_image_object_up);
#endif

    vector<vector<feature_t> > object_features;
    object_features.resize(descriptors.GetPyrLevels());
    vector<IplImage*> images;
    int object_feature_count = LoadFeatures(train_image_filename_object, object_features, images);

#if 0
	int max_object_features = 15;
	object_feature_count = 0;
	for (int i=0;i<(int)object_features.size();i++)
	{
		while ((int)object_features[i].size() > max_object_features)
		{
			object_features[i].pop_back();
		}
		object_feature_count+=(int)object_features[i].size();
	}
#endif

    vector<vector<feature_t> > background_features;
    vector<IplImage*> background_images;
    background_features.resize(1);
    int background_feature_count = LoadFeatures(train_image_filename_background, background_features, background_images);

#if 1
	//printf("Background features count: %d\n",background_feature_count);
	int max_background_features = 20;
	background_feature_count = 0;
	for (int i=0;i<(int)background_features.size();i++)
	{
		while ((int)background_features[i].size() > max_background_features)
		{
			background_features[i].pop_back();
		}
		background_feature_count+=(int)background_features[i].size();
	}
#endif

    int train_feature_count = object_feature_count + background_feature_count;
//    printf("Found %d train points...\n", train_feature_count);

    descriptors.Allocate(train_feature_count, object_feature_count);

    int descriptor_count = 0;
    for(int i = 0; i < descriptors.GetPyrLevels(); i++)
    {
        char feature_label[1024];
        sprintf(feature_label, "%s_%d", train_image_filename_object, i);
        IplImage* img = images[i];
        descriptors.InitializeObjectDescriptors(img, object_features[i], feature_label, descriptor_count, (float)(1<<i));
        descriptor_count += object_features[i].size();
    }

    descriptors.InitializeObjectDescriptors(background_images[0], background_features[0],
                                            train_image_filename_background, object_feature_count, 1.0f, 1);
#if defined(_KDTREE)
	descriptors.ConvertDescriptorsArrayToTree();
#endif
    cvReleaseImage(&train_image_object);
    cvReleaseImage(&train_image_background);

    ReleaseImageVector(images);
    ReleaseImageVector(background_images);
}

void ScaleFeatures(const vector<feature_t>& src, vector<feature_t>& dst, float scale)
{
    dst.resize(src.size());
    for(size_t i = 0; i < src.size(); i++)
    {
        dst[i] = feature_t(cvPoint(src[i].pt.x*scale, src[i].pt.y*scale), src[i].size, src[i].class_id);
    }
}

void ApplyGamma(IplImage* img, float gamma)
{
    IplImage* flt0 = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
    IplImage* flt = cvCreateImage(cvGetSize(img), IPL_DEPTH_32F, 1);
    IplImage* u8 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 1);

    for(int i = 1; i <= img->nChannels; i++)
    {
        cvSetImageCOI(img, i);
        cvCopy(img, u8);
        cvConvertScale(u8, flt0);

//        cvMul(flt0, flt0, flt);
//        cvMul(flt, flt0, flt);
        cvPow(flt0, flt, gamma);
        double maxval;
        cvMinMaxLoc(flt, 0, &maxval);

        cvConvertScale(flt, u8, 255.0/maxval);
        cvCopy(u8, img);
    }

    cvSetImageCOI(img, 0);

    cvReleaseImage(&flt0);
    cvReleaseImage(&flt);
    cvReleaseImage(&u8);
}

void FilterFeaturesOnEdges(const IplImage* img, const vector<feature_t>& src_features, vector<feature_t>& dst_features, int max_edge_dist, int min_contour_size)
{
	IplImage* gray = cvCreateImage(cvGetSize(img), 8, 1);
	if (img->nChannels > 1)
		cvCvtColor(img,gray,CV_BGR2GRAY);
	else
		cvCopy(img,gray);

	IplImage* edges = cvCreateImage( cvGetSize(img), 8, 1 );
	cvCanny( gray, edges, 140, 160, 5 );

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* contours;
	cvFindContours(edges,storage,&contours);


	CvMemStorage* storage1 = cvCreateMemStorage(0);
	CvSeq* contours_filt = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSeq*), storage1);

	int n = 0;

	IplImage* edges1 = cvCreateImage( cvGetSize(img), 8, 1 );
	cvSet(edges1,cvScalar(0));
	
	for(CvSeq* contour = contours; contour; contour = contour->h_next)
	{
		CvRect rect = cvBoundingRect(contour);
		if ((rect.width > min_contour_size) && (rect.height > min_contour_size) )
		{			
#if 0
			cvSeqPush(contours_filt,&contour);
			CvPoint* pt1;
			CvPoint* pt2;
			for (int i=0;i<contour->total-1;i++)
			{
				pt1 = (CvPoint*)cvGetSeqElem(contour,i);
				pt2 = (CvPoint*)cvGetSeqElem(contour,i+1);
				cvLine(edges1,*pt1,*pt2,cvScalar(255));
			}	
			pt1 = (CvPoint*)cvGetSeqElem(contour,0);
			pt2 = (CvPoint*)cvGetSeqElem(contour,contour->total-1);
			cvLine(edges1,*pt1,*pt2,cvScalar(255));
#else
            cvDrawContours(edges1, contour, cvScalar(255), cvScalar(255), 0, 1);
#endif
		}		
	}

	for (int i=0;i<max_edge_dist;i++)
	{
		cvDilate(edges1,edges1);
	}
    
#if 1
    cvNamedWindow("1", 1);
    cvShowImage("1", edges1);
    cvWaitKey(0);
#endif

	dst_features.clear();

	for (int i=0;i<(int)src_features.size();i++)
	{
		if (edges1->imageData[(int)src_features[i].pt.x+(int)src_features[i].pt.y*edges1->widthStep] == 0)
		{
			dst_features.push_back(src_features[i]);
		}
	}

	cvReleaseMemStorage(&storage);
	cvReleaseMemStorage(&storage1);
	cvReleaseImage(&edges);
	cvReleaseImage(&edges1);
	cvReleaseImage(&gray);
}
