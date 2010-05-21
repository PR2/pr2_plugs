/*
 *  one_way_outlets.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/16/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include "outlet_pose_estimation/detail/one_way_outlets.h"
#include "outlet_pose_estimation/detail/outlet_model.h"
#include "outlet_pose_estimation/detail/one_way_descriptor.h"
#include "outlet_pose_estimation/detail/one_way_descriptor_base.h"
#include "outlet_pose_estimation/detail/constellation.h"
#include "outlet_pose_estimation/detail/generalized_hough.h"
#include "outlet_pose_estimation/detail/outlet_tuple.h"
//#include "outlet_pose_estimation/detail/cvcalibinit_lowres.h"

#include <highgui.h>
#include <stdio.h>

using namespace std;

void drawLine(IplImage* image, CvPoint p1, CvPoint p2, CvScalar color, int thickness)
{
    if(p1.x < 0 || p1.y < 0 || p2.y < 0 || p2.y < 0) return;

    cvLine(image, p1, p2, color, thickness);
}

void detect_outlets_2x1_one_way(IplImage* test_image, const CvOneWayDescriptorObject* descriptors,
                                vector<feature_t>& holes, IplImage* color_image,
                                const char* output_path, const char* output_filename)
{

    IplImage* image = cvCreateImage(cvSize(test_image->width, test_image->height), IPL_DEPTH_8U, 3);
    cvCvtColor(test_image, image, CV_GRAY2RGB);
    IplImage* image1 = cvCloneImage(color_image);

    int64 time1 = cvGetTickCount();

    vector<feature_t> features;
    const float default_hole_contrast = 1.5f;
    GetHoleFeatures(test_image, features);

    int64 time2 = cvGetTickCount();
#if defined(_VERBOSE)
    printf("Found %d test features, time elapsed: %f\n", (int)features.size(), float(time2 - time1)/cvGetTickFrequency()*1e-6);
#endif //_VERBOSE

    IplImage* test_image_features = cvCreateImage(cvSize(test_image->width, test_image->height), IPL_DEPTH_8U, 3);
    cvCvtColor(test_image, test_image_features, CV_GRAY2RGB);
    DrawFeatures(test_image_features, features);

    vector<feature_t> hole_candidates;
    int patch_width = descriptors->GetPatchSize().width/2;
    int patch_height = descriptors->GetPatchSize().height/2;
    for(int i = 0; i < (int)features.size(); i++)
    {
        CvPoint center = features[i].pt;
        float scale = features[i].size;

        CvRect roi = cvRect(center.x - patch_width/2, center.y - patch_height/2, patch_width, patch_height);
        cvSetImageROI(test_image, roi);
        roi = cvGetImageROI(test_image);
        if(roi.width != patch_width || roi.height != patch_height)
        {
            continue;
        }

        if(abs(center.x - 988/2) < 10 && abs(center.y - 1203/2) < 10)
        {
            int w = 1;
        }
/*        else
        {
            continue;
        }
*/
        int desc_idx = -1;
        int pose_idx = -1;
        float distance = 0;
//        printf("i = %d\n", i);
        if(i == 331)
        {
            int w = 1;
        }

#if 0
        cvNamedWindow("1", 1);
        cvShowImage("1", test_image);
        cvWaitKey(0);
#endif
        descriptors->FindDescriptor(test_image, desc_idx, pose_idx, distance);

        CvPoint center_new = descriptors->GetDescriptor(desc_idx)->GetCenter();
        CvScalar color = descriptors->IsDescriptorObject(desc_idx) ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0);
        int part_idx = descriptors->GetDescriptorPart(desc_idx);
        if(part_idx >= 0 && part_idx < 4)
        {
            color = CV_RGB(255, 255, 0);
        }

        if(part_idx == 5 || part_idx == 6)
        {
            color = CV_RGB(0, 255, 255);
        }

        if(part_idx >= 0)
        {
            feature_t candidate = features[i];
            if(part_idx < 4) candidate.class_id = 0;
                else candidate.class_id = 1;
            hole_candidates.push_back(candidate);
        }

        cvCircle(image, center, scale, color, 2);

        cvResetImageROI(test_image);

#if 0
        IplImage* image1 = cvCreateImage(cvSize(train_image->width, train_image->height), IPL_DEPTH_8U, 3);
        cvCvtColor(train_image, image1, CV_GRAY2RGB);
        IplImage* image2 = cvCreateImage(cvSize(test_image->width, test_image->height), IPL_DEPTH_8U, 3);
        cvCvtColor(test_image, image2, CV_GRAY2RGB);

        cvCircle(image1, center_new, 20, cvScalar(255, 0, 0), 2);
        cvCircle(image2, center, 20, cvScalar(255, 0, 0), 2);
#endif

        //printf("Old center: %d,%d; new center: %d,%d\n", center_new.x, center_new.y, center.x, center.y);
        CvAffinePose pose = descriptors->GetDescriptor(desc_idx)->GetPose(pose_idx);
        //            printf("i = %d, pose: %f,%f,%f,%f\n", i, pose.phi, pose.theta, pose.lambda1, pose.lambda2);
        //            printf("Distance = %f\n\n", distance);

#if 0
        cvNamedWindow("1", 1);
        cvShowImage("1", image1);

        cvNamedWindow("2", 1);
        cvShowImage("2", image2);
        cvWaitKey(0);
        cvReleaseImage(&image1);
        cvReleaseImage(&image2);
#endif
    }

    int64 time3 = cvGetTickCount();

#if defined(_VERBOSE)
    printf("Features matched. Time elapsed: %f\n", float(time3 - time2)/cvGetTickFrequency()*1e-6);
#endif //_VERBOSE

    //        printf("%d features before filtering\n", (int)hole_candidates.size());
    vector<feature_t> hole_candidates_filtered;
    float dist = calc_set_std(descriptors->_GetLabeledFeatures());
    FilterOutletFeatures(hole_candidates, hole_candidates_filtered, dist*4);
    hole_candidates = hole_candidates_filtered;
    //        printf("Set size is %f\n", dist);
    //        printf("%d features after filtering\n", (int)hole_candidates.size());

    // clustering
    vector<feature_t> clusters;
    ClusterOutletFeatures(hole_candidates, clusters, dist*4);
    //        float min_error = 0;
    //        vector<feature_t> min_features;

    vector<int> indices;

#if defined(_HOMOGRAPHY)
    CvMat* homography = cvCreateMat(3, 3, CV_32FC1);
#else
    CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
#endif //_HOMOGRAPHY

    for(int k = 0; k < (int)clusters.size(); k++)
    {
        vector<feature_t> clustered_features;
        SelectNeighborFeatures(hole_candidates, clusters[k].pt, clustered_features, dist*4);

        DetectObjectConstellation(descriptors->_GetLabeledFeatures(), clustered_features, homography, indices);

        // print statistics
        int parts = 0;
        for(int i = 0; i < (int)indices.size(); i++) parts += (indices[i] >= 0);
#if 0
        printf("Found %d parts: ", parts);
        vector<int> indices_sort = indices;
        sort(indices_sort.begin(), indices_sort.end());
        for(int i = 0; i < (int)indices_sort.size(); i++) if(indices_sort[i] >= 0) printf("%d", indices_sort[i]);
        printf("\n");
#endif

        // infer missing objects
        if(parts > 0)
        {
            holes.clear();
            InferMissingObjects(descriptors->_GetLabeledFeatures(), clustered_features, homography, indices, holes);
        }
    }

    cvReleaseMat(&homography);

#if 0
    holes.resize(6);
    for(int i = 0; i < indices.size(); i++)
    {
        if(indices[i] == -1) continue;
        holes[indices[i]] = hole_candidates[i];
    }
#endif

    int64 time4 = cvGetTickCount();

#if defined(_VERBOSE)
    printf("Object detection completed. Time elapsed: %f\n", float(time4 - time3)/cvGetTickFrequency()*1e-6);
    printf("Total time elapsed: %f\n", float(time4 - time1)/cvGetTickFrequency()*1e-6);
#endif //_VERBOSE

    IplImage* image2 = cvCloneImage(image1);
    CvScalar color_parts[] = {CV_RGB(255, 255, 0), CV_RGB(0, 255, 255)};
    for(int i = 0; i < (int)hole_candidates.size(); i++)
    {
        cvCircle(image2, hole_candidates[i].pt, hole_candidates[i].size, color_parts[hole_candidates[i].class_id], 2);
    }

    CvScalar color[] = {CV_RGB(255, 255, 0), CV_RGB(255, 255, 0), CV_RGB(128, 128, 0),
    CV_RGB(128, 128, 0), CV_RGB(0, 255, 255), CV_RGB(0, 128, 128)};
    for(int i = 0; i < (int)holes.size(); i++)
    {
        //CvScalar color = i < 4 ? CV_RGB(255, 255, 0) : CV_RGB(0, 255, 255);
        cvCircle(image1, holes[i].pt, holes[i].size, color[i], 2);
    }

    if(holes.size() >= 6)
    {
        drawLine(image1, holes[0].pt, holes[1].pt, CV_RGB(255, 0, 0), 3);
        drawLine(image1, holes[1].pt, holes[4].pt, CV_RGB(255, 0, 0), 3);
        drawLine(image1, holes[4].pt, holes[0].pt, CV_RGB(255, 0, 0), 3);

        drawLine(image1, holes[2].pt, holes[3].pt, CV_RGB(255, 0, 0), 3);
        drawLine(image1, holes[3].pt, holes[5].pt, CV_RGB(255, 0, 0), 3);
        drawLine(image1, holes[5].pt, holes[2].pt, CV_RGB(255, 0, 0), 3);
    }

#if defined(_VERBOSE)
    char test_image_filename[1024];
    sprintf(test_image_filename, "%s/features/%s", output_path, output_filename);
    cvSaveImage(test_image_filename, image);

    sprintf(test_image_filename, "%s/outlets/%s", output_path, output_filename);
    cvSaveImage(test_image_filename, image1);

    sprintf(test_image_filename, "%s/features_filtered/%s", output_path, output_filename);
    cvSaveImage(test_image_filename, image2);
#endif //_VERBOSE

    cvReleaseImage(&image);
    cvReleaseImage(&image1);
    cvReleaseImage(&image2);
}

void detect_outlets_one_way(IplImage* test_image, const outlet_template_t& outlet_template,
                            vector<outlet_t>& holes, IplImage* color_image,
                            const char* output_path, const char* output_filename, float* scale_ranges)
{
	holes.clear();
    ApplyGamma(test_image, 1.0f);

    IplImage* image = cvCloneImage(test_image);
	IplImage* image_ = cvCreateImage(cvSize(test_image->width, test_image->height), IPL_DEPTH_8U, 3);
    IplImage* image1 = cvCloneImage(color_image);
    IplImage* image2 = cvCloneImage(image1);

    int64 time1 = cvGetTickCount();

    vector<feature_t> _features, features;
    GetHoleFeatures(test_image, features, outlet_template.GetHoleContrast());
//    FilterFeaturesOnEdges(test_image, _features, features, 0, 20);

    int64 time2 = cvGetTickCount();

#if defined(_VERBOSE)
    printf("Found %d test features, time elapsed: %f\n", (int)features.size(), float(time2 - time1)/cvGetTickFrequency()*1e-6);
#endif

#if 0
    IplImage* test_image_features = cvCreateImage(cvSize(test_image->width, test_image->height), IPL_DEPTH_8U, 3);
    cvCvtColor(test_image, test_image_features, CV_GRAY2RGB);
    DrawFeatures(test_image_features, features);

    cvNamedWindow("1", 1);
    cvShowImage("1", test_image_features);
    cvWaitKey(0);

    cvReleaseImage(&test_image_features);
#endif

    CvOneWayDescriptorObject* descriptors = const_cast<CvOneWayDescriptorObject*>(outlet_template.get_one_way_descriptor_base());
    vector<feature_t> hole_candidates;
    int patch_width = descriptors->GetPatchSize().width/2;
    int patch_height = descriptors->GetPatchSize().height/2;
	float modelErrorMin = 1e10;
    float max_votes = 0;
	float min_feature_scale = 1.0;//0.4;
	float max_feature_scale = 3.0;//1.2;
	float feature_scale_step = 1.15;

    // PJM: temporarily disabling this so I can get separate packages compiling
#if 0
	//Remove the chessboard
	CvSize size;
	size.width = 4;
	size.height = 5;
	CvPoint2D32f* corners = new CvPoint2D32f[size.width * size.height];
 	if (cvFindChessboardCornersLowres(test_image, size, corners))
	{
        cv::Mat chessboard_corners(1, size.width*size.height, CV_32FC2);
		for (int i = 0; i < size.width*size.height; i++)
		{
            chessboard_corners.at<CvPoint2D32f>(0, i) = corners[i];
		}
        CvMat _chessboard_corners = chessboard_corners;
        cv::Mat hull(1, size.width*size.height, CV_32FC2);
        CvMat _hull = hull;
        cvConvexHull2(&_chessboard_corners, &_hull, CV_CLOCKWISE);
        std::vector<feature_t> features_filtered;
        for(size_t i = 0; i < features.size(); i++)
		{
            const float min_dist = 10.0f;
            if(length(features[i].pt - cv::Point2f(124, 228)) < 10)
            {
                int w = 1;
            }
            int ret = cvPointPolygonTest(&_hull, features[i].pt, 0);
            if(ret >= 0) continue;
            double dist = cvPointPolygonTest(&_hull, features[i].pt, 1);
            if(fabs(dist) < min_dist) continue;
            features_filtered.push_back(features[i]);
		}

        features = features_filtered;
	}
	delete[] corners;
	//End of
#endif

	for (float _scale = min_feature_scale; _scale <= max_feature_scale; _scale*=feature_scale_step)
	{
#if defined(_VERBOSE)
		printf("Scale: %f\n",_scale);
//        printf("Size1 %d,%d, size 2 %d,%d\n", test_image->width, test_image->height, image_->width, image_->height);
        cvResetImageROI(test_image);
        cvCvtColor(test_image, image_, CV_GRAY2RGB);
#endif
		hole_candidates.clear();
		//int q = 0;


		for(int i = 0; i < (int)features.size(); i++)
		{
			CvPoint center = features[i].pt;
			float scale = features[i].size;

			//if (_scale < scale*1.7/descriptors->GetPatchSize().width*2)
			//	continue;
			//q++;

			CvRect roi = cvRect(center.x - patch_width/2, center.y - patch_height/2, patch_width, patch_height);
			cvSetImageROI(test_image, roi);
			roi = cvGetImageROI(test_image);
			if(roi.width != patch_width || roi.height != patch_height)
			{
				continue;
			}

			roi = resize_rect(roi, 1.0f);
			cvSetImageROI(test_image, roi);

			int desc_idx = -1;
			int pose_idx = -1;
			float distance = 0;

			CvMat* avg = 0;
			CvMat* eigenvectors = 0;
			descriptors->GetLowPCA(&avg, &eigenvectors);

	#if 0
			if(abs(center.x - 252) < 10 && abs(center.y - 153) < 10)
			{
				for(int k = 0; k < descriptors->GetDescriptorCount(); k++)
				{
					int part_id = descriptors->GetDescriptorPart(k);
					if(part_id < 0) continue;
					CvPoint center = descriptors->GetDescriptor(k)->GetCenter();
					descriptors->GetDescriptor(k)->EstimatePosePCA(test_image, pose_idx, distance, avg, eigenvectors);
					printf("k = %d, part_id = %d, center = (%d, %d), distance = %f\n", k, part_id, center.x, center.y, distance);
				}
				printf("\n");
			}
	#endif



			vector<int> desc_idxs;
			vector<int> pose_idxs;
			vector<float> distances;
			vector<float> scales;
			//int n=3;


			scale = _scale;
			float cur_scale;
			float _scale_ranges[2];
			if(scale_ranges)
			{
				_scale_ranges[0] = scale_ranges[0];
				_scale_ranges[1] = scale_ranges[1];
			}
			else
			{
				//_scale_ranges[0] = MAX(0.7f, scale*1.7/descriptors->GetPatchSize().width*2);
				//_scale_ranges[0] = 0.7f;
				//_scale_ranges[1] = 2.0f;
				//_scale_ranges[0] = _scale;
				_scale_ranges[0] = _scale < scale*1.7/descriptors->GetPatchSize().width*2 ? scale*1.7/descriptors->GetPatchSize().width*2 : _scale;
				_scale_ranges[1] = _scale_ranges[0]*1.01;
			}
			descriptors->FindDescriptor(test_image, desc_idx, pose_idx, distance,&cur_scale,_scale_ranges);




	#if 0
			if(abs(center.x - 252) < 10 && abs(center.y - 153) < 10)
			{
				const CvOneWayDescriptor* descriptor = descriptors->GetDescriptor(desc_idx);
                descriptor->EstimatePosePCA(test_image, pose_idx, distance, avg, eigenvectors);

				CvPoint center = descriptor->GetCenter();
                printf("center: %d,%d, distance = %f\n", center.x, center.y, distance);

				IplImage* img_match = const_cast<CvOneWayDescriptor*>(descriptors->GetDescriptor(desc_idx))->GetPatch(pose_idx);
				IplImage* img_match_8u = cvCreateImage(cvSize(img_match->width, img_match->height), IPL_DEPTH_8U, 1);
				double maxval1;
				cvMinMaxLoc(img_match, 0, &maxval1);

				int _pose_idx;
				float _distance;
				descriptors->GetDescriptor(0)->EstimatePosePCA(test_image, _pose_idx, _distance, avg, eigenvectors);
				IplImage* img_correct_match = const_cast<CvOneWayDescriptor*>(descriptors->GetDescriptor(0))->GetPatch(_pose_idx);
				CvPoint center0 = descriptors->GetDescriptor(0)->GetCenter();
				printf("Center0: %d,%d\n", center0.x, center0.y);
				IplImage* img_correct_match_8u = cvCreateImage(cvSize(img_match->width, img_match->height), IPL_DEPTH_8U, 1);
				double maxval2;
				cvMinMaxLoc(img_correct_match, 0, &maxval2);
				double maxval = MAX(maxval1, maxval2);

				cvConvertScale(img_match, img_match_8u, 255.0/maxval);
				cvConvertScale(img_correct_match, img_correct_match_8u, 255.0/maxval);

				cvMinMaxLoc(test_image, 0, &maxval);
				cvConvertScale(test_image, test_image, 255.0/maxval);

				cvNamedWindow("match", 1);
				cvShowImage("match", img_match_8u);
				cvSaveImage("match.jpg",img_match_8u);
				cvNamedWindow("correct", 1);
				cvShowImage("correct", img_correct_match_8u);
				cvSaveImage("correct.jpg",img_correct_match_8u);
				cvNamedWindow("src", 1);
				cvShowImage("src", test_image);
				cvSaveImage("src.jpg", test_image);
				cvWaitKey(0);

			}
	#endif
#if defined(_VERBOSE)
			if(desc_idx < 0)
			{
				printf("Descriptor not found for feature %i, skipping...\n", i);
				continue;
			}
#endif //_VERBOSE

			CvPoint center_new = descriptors->GetDescriptor(desc_idx)->GetCenter();
			CvScalar color = descriptors->IsDescriptorObject(desc_idx) ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0);
			int part_idx = descriptors->GetDescriptorPart(desc_idx);

			int min_ground_idx = (int)(descriptors->GetLabeledFeatures().size()) / 3 * 2; // 3 there is number of holes in the outlet (including ground hole)
			if(part_idx >= 0 && part_idx < min_ground_idx)
			{
				color = CV_RGB(255, 255, 0);
			}

			if((part_idx >= min_ground_idx) && (part_idx <  (int)(descriptors->GetLabeledFeatures().size())))
			{
				color = CV_RGB(0, 255, 255);
				if (scale_ranges)
					scale_ranges[2] = cur_scale;
			}


			if(part_idx >= 0)
			{
				feature_t candidate = features[i];
				if(part_idx < min_ground_idx) candidate.class_id = 0;
				else candidate.class_id = 1;
				hole_candidates.push_back(candidate);
			}
            else if(descriptors->IsDescriptorObject(desc_idx))
            {
                feature_t candidate = features[i];
                candidate.class_id = 0;
                hole_candidates.push_back(candidate);
            }

			cvCircle(image_, center, scale, color, 2);

#if 0// Show descriptor for each feature
			printf("Part idx: %d\n",part_idx);
			IplImage* img_match = const_cast<CvOneWayDescriptor*>(descriptors->GetDescriptor(desc_idx))->GetPatch(pose_idx);
			IplImage* img_match_8u = cvCreateImage(cvSize(img_match->width, img_match->height), IPL_DEPTH_8U, 1);
			double maxval1;
			cvMinMaxLoc(img_match, 0, &maxval1);
			cvConvertScale(img_match, img_match_8u, 255.0/maxval1);

			IplImage* tmp = cvCloneImage(color_image);
			cvCircle(tmp,center,patch_width/2,color);
			cvNamedWindow("Descriptor",0);
			cvNamedWindow("Patch",0);
			cvNamedWindow("image_");
			cvNamedWindow("orig");
			cvShowImage("Descriptor",img_match_8u);
			cvShowImage("Patch",test_image);
			cvShowImage("image_",tmp);
			cvShowImage("orig",image_);
			cvWaitKey();
			cvReleaseImage(&tmp);
			//cvReleaseImage(&img_match);
			cvReleaseImage(&img_match_8u);
#endif

			cvResetImageROI(test_image);

		}

        cvResetImageROI(test_image);

		int64 time3 = cvGetTickCount();
#if defined(_VERBOSE)
		printf("Features matched. Time elapsed: %f\n", float(time3 - time2)/cvGetTickFrequency()*1e-6);
#endif //_VERBOSE

		//        printf("%d features before filtering\n", (int)hole_candidates.size());
		vector<feature_t> hole_candidates_filtered;
		float dist = calc_set_std(descriptors->_GetLabeledFeatures());
		FilterOutletFeatures(hole_candidates, hole_candidates_filtered, dist*4);
		hole_candidates = hole_candidates_filtered;
		//        printf("Set size is %f\n", dist);
		//        printf("%d features after filtering\n", (int)hole_candidates.size());

		// clustering
		vector<feature_t> clusters;
		ClusterOutletFeatures(hole_candidates, clusters, dist*4);

		vector<int> indices;

#if defined(_GHT) // Test histogram calculating
        float modelError;
		for(int i = 0; i < descriptors->GetPyrLevels(); i++)
		{

			vector<feature_t> train_features;
			float scale = 1.0f/(1<<i);
			ScaleFeatures(descriptors->_GetLabeledFeatures(), train_features, scale);


			vector<outlet_t> _holes;

            int64 _time1 = cvGetTickCount();
            float votes = matchOutlets(hole_candidates, outlet_template, train_features, _holes);
            int64 _time2 = cvGetTickCount();

#if defined(_VERBOSE)
            printf("GH time elapsed: %f\n", double(_time2 - _time1)/cvGetTickFrequency()*1e-6);
#endif //_VERBOSE

            if(votes > max_votes)
            {
                max_votes = votes;
                holes = _holes;
            }

#if 0
            cvNamedWindow("1", 1);
            IplImage* _test = cvCloneImage(color_image);
            draw_outlets(_test, _holes);
            cvShowImage("1", _test);
            cvWaitKey(0);
            cvReleaseImage(&_test);
#endif
		}
#endif //_GHT


		int64 time4 = cvGetTickCount();
#if defined(_VERBOSE)
		printf("Object detection completed, max_votes = %d. Time elapsed: %f\n", (int)max_votes,
               float(time4 - time3)/cvGetTickFrequency()*1e-6);
#endif //_VERBOSE

#if defined(_SAVE_VERBOSE)
        char test_image_filename[1024];
        std::string output_name = std::string(output_filename).substr(0, strlen(output_filename) - 4);
        sprintf(test_image_filename, "%s/features/%s_%f.jpg", output_path, output_name.c_str(), _scale);
        cvSaveImage(test_image_filename, image_);
#endif //_SAVE_VERBOSE
	}
	int64 time4 = cvGetTickCount();
#if defined(_VERBOSE)
    printf("Total time elapsed: %f\n", float(time4 - time1)/cvGetTickFrequency()*1e-6);
#endif

    //CvScalar color_parts[] = {CV_RGB(255, 255, 0), CV_RGB(0, 255, 255)};
    //for(int i = 0; i < (int)hole_candidates.size(); i++)
    //{
    //    cvCircle(image2, hole_candidates[i].pt, hole_candidates[i].size, color_parts[hole_candidates[i].class_id], 2);
    //}


#if 1
    if(holes.size() == 2)
    {
        findPreciseOutletLocationsAvg(test_image, outlet_template, holes);
//        printf("hole1: %f,%f, hole2: %f,%f\n", holes[0].hole1f.x, holes[0].hole1f.y, holes[0].hole2f.x, holes[0].hole2f.y);
    }
#endif


#if defined(_SAVE_VERBOSE)
    draw_outlets(image1, holes);
    char test_image_filename[1024];
    sprintf(test_image_filename, "%s/outlets/%s", output_path, output_filename);
    cvSaveImage(test_image_filename, image1);

    sprintf(test_image_filename, "%s/features_filtered/%s", output_path, output_filename);
    cvSaveImage(test_image_filename, image2);
#endif //_SAVE_VERBOSE

    cvReleaseImage(&image);
	cvReleaseImage(&image_);
    cvReleaseImage(&image1);
    cvReleaseImage(&image2);
}
//------------------------------
float calc_outlet_position(const vector<feature_t>& hole_candidates, const outlet_template_t& outlet_template,
						   vector<int>& desc_idx_vec, vector<int>& pose_idx_vec, vector<int>& idx_filtered,
						   vector<outlet_t>& outlet/*, IplImage* tmp = 0*/)
{
	vector<feature_t> _hole_candidates = hole_candidates;
	CvOneWayDescriptorObject* descriptors = const_cast<CvOneWayDescriptorObject*>(outlet_template.get_one_way_descriptor_base());
    float modelErrorMin = 1e30;
	float modelError = modelErrorMin;
	outlet.clear();
	vector<feature_t> outlet_features;


	CvMat* transform = cvCreateMat(2, 3, CV_32FC1);
    for(int i = 0; i < descriptors->GetPyrLevels(); i++)
    {
        vector<feature_t> train_features;
        ScaleFeatures(descriptors->_GetLabeledFeatures(), train_features, 1.0f/(1<<i));
		float accuracy = sqrt((float)((train_features[1].pt.x -train_features[0].pt.x)*(train_features[1].pt.x -train_features[0].pt.x)+
			(train_features[1].pt.y -train_features[0].pt.y)*(train_features[1].pt.y -train_features[0].pt.y)));

		for (int j=0;j<(int)hole_candidates.size();j++)
		{
			//if (hole_candidates[j].class_id < 1)
			//	continue;

			vector<feature_t> t_features = train_features;
			CvAffinePose pose = descriptors->GetDescriptor(desc_idx_vec[idx_filtered[j]])->GetPose(pose_idx_vec[idx_filtered[j]]);

			GenerateAffineTransformFromPose(cvSize(descriptors->GetPatchSize().width*2, descriptors->GetPatchSize().height*2),pose,transform);
			for (int k=0;k<(int)train_features.size();k++)
			{
				t_features[k].pt.x = cvmGet(transform,0,0)*train_features[k].pt.x+cvmGet(transform,0,1)*train_features[k].pt.y+cvmGet(transform,0,2);
				t_features[k].pt.y = cvmGet(transform,1,0)*train_features[k].pt.x+cvmGet(transform,1,1)*train_features[k].pt.y+cvmGet(transform,1,2);
			}


			for (int k=0;k<(int)(t_features.size());k++)
			{
				if (t_features[k].class_id != hole_candidates[j].class_id)
					continue;
				vector<feature_t> test_outlet = t_features;
				vector<feature_t> res_outlet;

				for (int l=0;l<(int)t_features.size();l++)
				{
					test_outlet[l].pt.x += (hole_candidates[j].pt.x - t_features[k].pt.x);
					test_outlet[l].pt.y += (hole_candidates[j].pt.y - t_features[k].pt.y);
				}

				float error;
				calcExactLocation(_hole_candidates,train_features,test_outlet,res_outlet,error,accuracy,false);



				if (error < modelError)
				{
					modelError = error;
					outlet_features = res_outlet;
				}

			}

		}
	}

	cvReleaseMat(&transform);
	if ((int)outlet_features.size() == 0)
	{
		return modelErrorMin;
	}
	else
	{

		outlet_t curr_outlet;

		for (int i=0;i<(int)outlet_features.size()/3;i++)
		{
			curr_outlet.hole1 = outlet_features[2*i].pt;
			curr_outlet.hole2 = outlet_features[2*i+1].pt;
			curr_outlet.ground_hole = outlet_features[2*(int)outlet_features.size()/3+i].pt;
			outlet.push_back(curr_outlet);
		}

		return modelError;
	}

}


//-----------------

void filterPoints(const std::vector<KeyPointEx>& src, const std::vector<bool>& is_detected, std::vector<KeyPointEx>& dst)
{
    dst.clear();
    for(size_t i = 0; i < src.size(); i++)
    {
        if(is_detected[i]) dst.push_back(src[i]);
    }
}

float matchOutlets(const std::vector<KeyPointEx>& test_points, const outlet_template_t& outlet_template,
                  const std::vector<KeyPointEx>& template_points, std::vector<outlet_t>& outlets)
{
    const PointMatcher& matcher = outlet_template.getGeometricMatcher();
    std::vector<float> votes;
    std::vector<std::pair<AffineBasis, AffineBasis> > matched_bases;

    matcher.match(test_points, votes, matched_bases);
    if(votes.size() == 0)
    {
        return 0;
    }
    std::vector<float>::const_iterator max_vote = std::max_element(votes.begin(), votes.end());
    int max_idx = max_vote - votes.begin();
    std::vector<KeyPointEx> mapped_points;

    mapPoints(template_points, matched_bases[max_idx].first, matched_bases[max_idx].second, mapped_points);

    std::vector<KeyPointEx> matched_test_points;
    std::vector<bool> is_detected;
    const float max_dist = 7.0f;
    findClosestPoint(mapped_points, test_points, matched_test_points, is_detected, max_dist);

    convertFeaturesToOutlet(matched_test_points, is_detected, outlets);

    // special case of 2x2 outlets
    if(outlets.size() == 4)
    {
        // test for mirror position
        cv::Point2f vec1 = outlets[1].ground_hole - outlets[0].ground_hole;
        cv::Point2f vec2 = outlets[2].ground_hole - outlets[0].ground_hole;
        float cross_prod = vec1.x*vec2.y - vec1.y*vec2.x;
        if(cross_prod < 0)
        {
            outlet_t outlet;
            // switch outlets
            outlet = outlets[0];outlets[0] = outlets[1];outlets[1] = outlet;
            outlet = outlets[2];outlets[2] = outlets[3];outlets[3] = outlet;
            for(size_t i = 0; i < 4; i++)
            {
                cv::Point p = outlets[i].hole1;
                outlets[i].hole1 = outlets[i].hole2;
                outlets[i].hole2 = p;
            }
        }
    }
    else if(outlets.size() == 2)
    {
        if(cv::Point(outlets[0].hole1) == cv::Point(outlets[1].hole1) || cv::Point(outlets[0].hole2) == cv::Point(outlets[1].hole2))
        {
            // this is a workaround, need to find the root cause and fix this
            outlets.clear();
            return 0;
        }
        cv::Point2f vec1 = outlets[1].ground_hole - outlets[0].ground_hole;
        cv::Point2f vec2 = outlets[0].hole2 - outlets[0].hole1;
        float cross_prod = vec1.x*vec2.y - vec1.y*vec2.x;
        if(cross_prod > 0)
        {
            // switch hole1 and hole2
            for(size_t i = 0; i < outlets.size(); i++)
            {
                CvPoint p = outlets[i].hole1;
                outlets[i].hole1 = outlets[i].hole2;
                outlets[i].hole2 = p;
            }
        }
    }

    return *max_vote;
}
