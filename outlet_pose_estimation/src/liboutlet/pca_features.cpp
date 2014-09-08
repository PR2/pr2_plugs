/*
 *  pca_features.cpp
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/15/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <cstdio>
#include "outlet_pose_estimation/detail/pca_features.h"
#include <highgui.h>

using namespace std;

void savePCAFeatures(const char* filename, CvMat* avg, CvMat* eigenvectors)
{
    CvMemStorage* storage = cvCreateMemStorage();
    
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_WRITE);
    cvWrite(fs, "avg", avg);
    cvWrite(fs, "eigenvectors", eigenvectors);
    cvReleaseFileStorage(&fs);   
    
    cvReleaseMemStorage(&storage);
}

void calcPCAFeatures(vector<IplImage*>& patches, const char* filename)
{
    int width = patches[0]->width;
    int height = patches[0]->height;
    int length = width*height;
    int patch_count = (int)patches.size();
    
    CvMat* data = cvCreateMat(patch_count, length, CV_32FC1);
    CvMat* avg = cvCreateMat(1, length, CV_32FC1);
    CvMat* eigenvalues = cvCreateMat(length, 1, CV_32FC1);
    CvMat* eigenvectors = cvCreateMat(length, length, CV_32FC1);
    
    for(int i = 0; i < patch_count; i++)
    {
        float sum = cvSum(patches[i]).val[0];
        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                *((float*)(data->data.ptr + data->step*i) + y*width + x) = (float)(unsigned char)patches[i]->imageData[y*patches[i]->widthStep + x]/sum;
            }
        }
    }
    
    cvCalcPCA(data, avg, eigenvalues, eigenvectors, CV_PCA_DATA_AS_ROW);
    
    // save pca data
    savePCAFeatures(filename, avg, eigenvectors);
    
    cvReleaseMat(&data);
    cvReleaseMat(&eigenvalues);
    cvReleaseMat(&eigenvectors);
}


void loadPCAFeatures(const char* path, vector<IplImage*>& patches)
{
    const int file_count = 20;
    for(int i = 0; i < file_count; i++)
    {
        char buf[1024];
        sprintf(buf, "%s/frame%04d.jpg", path, i);
        IplImage* img = loadImageRed(buf);
        
        vector<feature_t> features;
        GetHoleFeatures(img, features);
        
        for(int j = 0; j < (int)features.size(); j++)
        {
            const int patch_width = 24;
            const int patch_height = 24;
            
            CvPoint center = features[j].pt;
            
            CvRect roi = cvRect(center.x - patch_width/2, center.y - patch_height/2, patch_width, patch_height);
            cvSetImageROI(img, roi);
            roi = cvGetImageROI(img);
            if(roi.width != patch_width || roi.height != patch_height)
            {
                continue;
            }
            
            IplImage* patch = cvCreateImage(cvSize(patch_width, patch_height), IPL_DEPTH_8U, 1);
            cvCopy(img, patch);
            patches.push_back(patch);
            cvResetImageROI(img);
            
        }
        
        printf("Completed file %d, extracted %d features\n", i, (int)features.size());
        
        cvReleaseImage(&img);
    }
}
