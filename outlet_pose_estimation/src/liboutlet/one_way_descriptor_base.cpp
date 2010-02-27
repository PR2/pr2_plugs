/*
 *  one_way_descriptor_base.cpp
 *  outlet_detection
 *
 *  Created by Victor  Eruhimov on 7/9/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <cstdio>

#include "outlet_pose_estimation/detail/features.h"
#include "outlet_pose_estimation/detail/pca_features.h"
#include "outlet_pose_estimation/detail/one_way_descriptor_base.h"

#include <cv.h>
#include <highgui.h>

using namespace std;

CvMat* ConvertImageToMatrix(IplImage* patch)
{
    CvRect roi = cvGetImageROI(patch);
    CvMat* mat = cvCreateMat(1, roi.width*roi.height, CV_32FC1);

    if(patch->depth == 32)
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                mat->data.fl[y*roi.width + x] = *((float*)(patch->imageData + (y + roi.y)*patch->widthStep) + x + roi.x);
            }
        }
    }
    else if(patch->depth == 8)
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                mat->data.fl[y*roi.width + x] = (float)(unsigned char)patch->imageData[(y + roi.y)*patch->widthStep + x + roi.x];
            }
        }
    }
    else
    {
        printf("Image depth %d is not supported\n", patch->depth);
        return 0;
    }

    return mat;
}

CvOneWayDescriptorBase::CvOneWayDescriptorBase(CvSize patch_size, int pose_count, const char* train_path,
                                               const char* pca_config, const char* pca_hr_config,
                                               const char* pca_desc_config, int pyr_levels,
                                               int pca_dim_high, int pca_dim_low) : m_pca_dim_high(pca_dim_high), m_pca_dim_low(pca_dim_low)
{
//	m_pca_descriptors_matrix = 0;
    m_patch_size = patch_size;
    m_pose_count = pose_count;
    m_pyr_levels = pyr_levels;
    m_poses = 0;
    m_transforms = 0;

    m_pca_avg = 0;
    m_pca_eigenvectors = 0;
    m_pca_hr_avg = 0;
    m_pca_hr_eigenvectors = 0;
    m_pca_descriptors = 0;

    m_descriptors = 0;

    if(train_path == 0 || strlen(train_path) == 0)
    {
        // skip pca loading
        return;
    }
    char pca_config_filename[1024];
    sprintf(pca_config_filename, "%s/%s", train_path, pca_config);
    readPCAFeatures(pca_config_filename, &m_pca_avg, &m_pca_eigenvectors);
    if(pca_hr_config && strlen(pca_hr_config) > 0)
    {
        char pca_hr_config_filename[1024];
        sprintf(pca_hr_config_filename, "%s/%s", train_path, pca_hr_config);
        readPCAFeatures(pca_hr_config_filename, &m_pca_hr_avg, &m_pca_hr_eigenvectors);
    }

    m_pca_descriptors = new CvOneWayDescriptor[m_pca_dim_high + 1];
    if(pca_desc_config && strlen(pca_desc_config) > 0)
//    if(0)
    {
        //printf("Loading the descriptors...");
        char pca_desc_config_filename[1024];
        sprintf(pca_desc_config_filename, "%s/%s", train_path, pca_desc_config);
        LoadPCADescriptors(pca_desc_config_filename);
        //printf("done.\n");
    }
    else
    {
        printf("Initializing the descriptors...\n");
        InitializePoseTransforms();
        CreatePCADescriptors();
        SavePCADescriptors("pca_descriptors.yml");
    }
//    SavePCADescriptors("./pca_descriptors.yml");

}

CvOneWayDescriptorBase::~CvOneWayDescriptorBase()
{
    cvReleaseMat(&m_pca_avg);
    cvReleaseMat(&m_pca_eigenvectors);

    if(m_pca_hr_eigenvectors)
    {
        delete[] m_pca_descriptors;
        cvReleaseMat(&m_pca_hr_avg);
        cvReleaseMat(&m_pca_hr_eigenvectors);
    }


    delete []m_descriptors;

    if(!m_transforms)
    {
        delete []m_poses;
    }

    for(int i = 0; i < m_pose_count; i++)
    {
        cvReleaseMat(&m_transforms[i]);
    }
    delete []m_transforms;

#if defined(_KDTREE)
//	if (m_pca_descriptors_matrix)
//		delete m_pca_descriptors_matrix;
	cvReleaseMat(&m_pca_descriptors_matrix);
	delete m_pca_descriptors_tree;
#endif
}

void CvOneWayDescriptorBase::InitializePoses()
{
    m_poses = new CvAffinePose[m_pose_count];
    for(int i = 0; i < m_pose_count; i++)
    {
        m_poses[i] = GenRandomAffinePose();
    }
}

void CvOneWayDescriptorBase::InitializeTransformsFromPoses()
{
    m_transforms = new CvMat*[m_pose_count];
    for(int i = 0; i < m_pose_count; i++)
    {
        m_transforms[i] = cvCreateMat(2, 3, CV_32FC1);
        GenerateAffineTransformFromPose(cvSize(m_patch_size.width*2, m_patch_size.height*2), m_poses[i], m_transforms[i]);
    }
}

void CvOneWayDescriptorBase::InitializePoseTransforms()
{
    InitializePoses();
    InitializeTransformsFromPoses();
}

void CvOneWayDescriptorBase::InitializeDescriptor(int desc_idx, IplImage* train_image, const char* feature_label)
{
    m_descriptors[desc_idx].SetPCADimHigh(m_pca_dim_high);
    m_descriptors[desc_idx].SetPCADimLow(m_pca_dim_low);
    m_descriptors[desc_idx].SetTransforms(m_poses, m_transforms);

    if(!m_pca_hr_eigenvectors)
    {
        m_descriptors[desc_idx].Initialize(m_pose_count, train_image, feature_label);
    }
    else
    {
        m_descriptors[desc_idx].InitializeFast(m_pose_count, train_image, feature_label,
                                      m_pca_hr_avg, m_pca_hr_eigenvectors, m_pca_descriptors);
    }

    if(m_pca_avg)
    {
        m_descriptors[desc_idx].InitializePCACoeffs(m_pca_avg, m_pca_eigenvectors);
    }
}

void CvOneWayDescriptorBase::FindDescriptor(IplImage* src, cv::Point2f pt, int& desc_idx, int& pose_idx, float& distance) const
{

	CvRect roi = cvRect(pt.x - m_patch_size.width/4, pt.y - m_patch_size.height/4, m_patch_size.width/2, m_patch_size.height/2);
	cvSetImageROI(src, roi);


    FindDescriptor(src, desc_idx, pose_idx, distance);

	cvResetImageROI(src);

}

void CvOneWayDescriptorBase::FindDescriptor(IplImage* patch, int& desc_idx, int& pose_idx, float& distance, float* _scale, float* scale_ranges) const
{
#if 0
    ::FindOneWayDescriptor(m_train_feature_count, m_descriptors, patch, desc_idx, pose_idx, distance, m_pca_avg, m_pca_eigenvectors);
#else
    float scale_min = 0.7f;
    float scale_max = 2.0f;
    float scale_step = 1.2f;

	if (scale_ranges)
	{
		scale_min = scale_ranges[0];
		scale_max = scale_ranges[1];
	}

    float scale = 1.0f;

#if !defined(_KDTREE)
	::FindOneWayDescriptorEx(m_train_feature_count, m_descriptors, patch,
								scale_min, scale_max, scale_step, desc_idx, pose_idx, distance, scale,
								m_pca_avg, m_pca_eigenvectors);
#else
	::FindOneWayDescriptorEx(m_pca_descriptors_tree, m_descriptors[0].GetPatchSize(), m_descriptors[0].GetPCADimLow(), m_pose_count, patch,
								scale_min, scale_max, scale_step, desc_idx, pose_idx, distance, scale,
								m_pca_avg, m_pca_eigenvectors);
#endif

	if (_scale)
		*_scale = scale;

#endif
}

void CvOneWayDescriptorBase::FindDescriptor(IplImage* patch, int n, std::vector<int>& desc_idxs, std::vector<int>& pose_idxs,
    std::vector<float>& distances, std::vector<float>& _scales, float* scale_ranges) const
{
    float scale_min = 0.7f;
    float scale_max = 2.5f;
    float scale_step = 1.2f;

	if (scale_ranges)
	{
		scale_min = scale_ranges[0];
		scale_max = scale_ranges[1];
	}

	distances.resize(n);
	_scales.resize(n);
	desc_idxs.resize(n);
	pose_idxs.resize(n);
    /*float scales = 1.0f;*/

	::FindOneWayDescriptorEx(m_train_feature_count, m_descriptors, patch,
								scale_min, scale_max, scale_step ,n, desc_idxs, pose_idxs, distances, _scales,
								m_pca_avg, m_pca_eigenvectors);

}

void CvOneWayDescriptorBase::SetPCAHigh(CvMat* avg, CvMat* eigenvectors)
{
    m_pca_hr_avg = cvCloneMat(avg);
    m_pca_hr_eigenvectors = cvCloneMat(eigenvectors);
}

void CvOneWayDescriptorBase::SetPCALow(CvMat* avg, CvMat* eigenvectors)
{
    m_pca_avg = cvCloneMat(avg);
    m_pca_eigenvectors = cvCloneMat(eigenvectors);
}

void CvOneWayDescriptorBase::AllocatePCADescriptors()
{
    m_pca_descriptors = new CvOneWayDescriptor[m_pca_dim_high + 1];
    for(int i = 0; i < m_pca_dim_high + 1; i++)
    {
        m_pca_descriptors[i].SetPCADimHigh(m_pca_dim_high);
        m_pca_descriptors[i].SetPCADimLow(m_pca_dim_low);
    }
}

void CvOneWayDescriptorBase::CreatePCADescriptors()
{
    if(m_pca_descriptors == 0)
    {
        AllocatePCADescriptors();
    }
    IplImage* frontal = cvCreateImage(m_patch_size, IPL_DEPTH_32F, 1);

    eigenvector2image(m_pca_hr_avg, frontal);
    m_pca_descriptors[0].SetTransforms(m_poses, m_transforms);
    m_pca_descriptors[0].Initialize(m_pose_count, frontal, "", 0);

    for(int j = 0; j < m_pca_dim_high; j++)
    {
        CvMat eigenvector;
        cvGetSubRect(m_pca_hr_eigenvectors, &eigenvector, cvRect(0, j, m_pca_hr_eigenvectors->cols, 1));
        eigenvector2image(&eigenvector, frontal);

        m_pca_descriptors[j + 1].SetTransforms(m_poses, m_transforms);
        m_pca_descriptors[j + 1].Initialize(m_pose_count, frontal, "", 0);

        printf("Created descriptor for PCA component %d\n", j);
    }

    cvReleaseImage(&frontal);
}


int CvOneWayDescriptorBase::LoadPCADescriptors(const char* filename)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_READ);
    if(!fs)
    {
        cvReleaseMemStorage(&storage);
        printf("File %f not found...\n", filename);
        return 0;
    }

    // read affine poses
    CvFileNode* node = cvGetFileNodeByName(fs, 0, "affine poses");
    if(node != 0)
    {
        CvMat* poses = (CvMat*)cvRead(fs, node);
        //if(poses->rows != m_pose_count)
        //{
        //    printf("Inconsistency in the number of poses between the class instance and the file! Exiting...\n");
        //    cvReleaseMat(&poses);
        //    cvReleaseFileStorage(&fs);
        //    cvReleaseMemStorage(&storage);
        //}

        if(m_poses)
        {
            delete m_poses;
        }
        m_poses = new CvAffinePose[m_pose_count];
        for(int i = 0; i < m_pose_count; i++)
        {
            m_poses[i].phi = cvmGet(poses, i, 0);
            m_poses[i].theta = cvmGet(poses, i, 1);
            m_poses[i].lambda1 = cvmGet(poses, i, 2);
            m_poses[i].lambda2 = cvmGet(poses, i, 3);
        }
        cvReleaseMat(&poses);

        // now initialize pose transforms
        InitializeTransformsFromPoses();
    }
    else
    {
        printf("Node \"affine poses\" not found...\n");
    }

    node = cvGetFileNodeByName(fs, 0, "pca components number");
    if(node != 0)
    {

        m_pca_dim_high = cvReadInt(node);
        if(m_pca_descriptors)
        {
            delete []m_pca_descriptors;
        }
        AllocatePCADescriptors();
        for(int i = 0; i < m_pca_dim_high + 1; i++)
        {
            m_pca_descriptors[i].Allocate(m_pose_count, m_patch_size, 1);
            m_pca_descriptors[i].SetTransforms(m_poses, m_transforms);
            char buf[1024];
            sprintf(buf, "descriptor for pca component %d", i);
            m_pca_descriptors[i].ReadByName(fs, 0, buf);
        }
    }
    else
    {
        printf("Node \"pca components number\" not found...\n");
    }

    cvReleaseFileStorage(&fs);
    cvReleaseMemStorage(&storage);

    printf("Successfully read %d pca components\n", m_pca_dim_high);

    return 1;
}

void CvOneWayDescriptorBase::SavePCADescriptors(const char* filename)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_WRITE);

    cvWriteInt(fs, "pca components number", m_pca_dim_high);
    cvWriteComment(fs, "The first component is the average Vector, so the total number of components is <pca components number> + 1", 0);
    cvWriteInt(fs, "patch width", m_patch_size.width);
    cvWriteInt(fs, "patch height", m_patch_size.height);

    // pack the affine transforms into a single CvMat and write them
    CvMat* poses = cvCreateMat(m_pose_count, 4, CV_32FC1);
    for(int i = 0; i < m_pose_count; i++)
    {
        cvmSet(poses, i, 0, m_poses[i].phi);
        cvmSet(poses, i, 1, m_poses[i].theta);
        cvmSet(poses, i, 2, m_poses[i].lambda1);
        cvmSet(poses, i, 3, m_poses[i].lambda2);
    }
    cvWrite(fs, "affine poses", poses);
    cvReleaseMat(&poses);

    for(int i = 0; i < m_pca_dim_high + 1; i++)
    {
        char buf[1024];
        sprintf(buf, "descriptor for pca component %d", i);
        m_pca_descriptors[i].Write(fs, buf);
    }

    cvReleaseMemStorage(&storage);
    cvReleaseFileStorage(&fs);
}

void CvOneWayDescriptorBase::Allocate(int train_feature_count)
{
    m_train_feature_count = train_feature_count;
    m_descriptors = new CvOneWayDescriptor[m_train_feature_count];
    for(int i = 0; i < m_train_feature_count; i++)
    {
        m_descriptors[i].SetPCADimHigh(m_pca_dim_high);
        m_descriptors[i].SetPCADimLow(m_pca_dim_low);
    }
}

void CvOneWayDescriptorBase::InitializeDescriptors(IplImage* train_image, const vector<KeyPointEx>& features,
                                                           const char* feature_label, int desc_start_idx)
{
    for(int i = 0; i < (int)features.size(); i++)
    {
        CvPoint center = features[i].pt;

        CvRect roi = cvRect(center.x - m_patch_size.width/2, center.y - m_patch_size.height/2, m_patch_size.width, m_patch_size.height);
        cvResetImageROI(train_image);
        roi = fit_rect_fixedsize(roi, train_image);
        cvSetImageROI(train_image, roi);
//        roi = cvGetImageROI(train_image);
        if(roi.width != m_patch_size.width || roi.height != m_patch_size.height)
        {
            continue;
        }

        InitializeDescriptor(desc_start_idx + i, train_image, feature_label);

//        printf("Completed feature %d\n", i);

    }
    cvResetImageROI(train_image);

}

void CvOneWayDescriptorBase::CreateDescriptorsFromImage(IplImage* src, const std::vector<KeyPointEx>& features)
{
    m_train_feature_count = (int)features.size();

    m_descriptors = new CvOneWayDescriptor[m_train_feature_count];

    InitializeDescriptors(src, features);

}

#if defined(_KDTREE)
void CvOneWayDescriptorBase::ConvertDescriptorsArrayToTree()
{
	int n = this->GetDescriptorCount();
	if (n<1)
		return;
	int pca_dim_low = this->GetDescriptor(0)->GetPCADimLow();

	//if (!m_pca_descriptors_matrix)
	//	m_pca_descriptors_matrix = new ::flann::Matrix<float>(n*m_pose_count,pca_dim_low);
	//else
	//{
	//	if ((m_pca_descriptors_matrix->cols != pca_dim_low)&&(m_pca_descriptors_matrix->rows != n*m_pose_count))
	//	{
	//		delete m_pca_descriptors_matrix;
	//		m_pca_descriptors_matrix = new ::flann::Matrix<float>(n*m_pose_count,pca_dim_low);
	//	}
	//}

	m_pca_descriptors_matrix = cvCreateMat(n*m_pose_count,pca_dim_low,CV_32FC1);
	for (int i=0;i<n;i++)
	{
		CvMat** pca_coeffs = m_descriptors[i].GetPCACoeffs();
		for (int j = 0;j<m_pose_count;j++)
		{
			for (int k=0;k<pca_dim_low;k++)
			{
				m_pca_descriptors_matrix->data.fl[(i*m_pose_count+j)*m_pca_dim_low + k] = pca_coeffs[j]->data.fl[k];
			}
		}
	}
	cv::Mat pca_descriptors_mat(m_pca_descriptors_matrix,false);

	//::flann::KDTreeIndexParams params;
	//params.trees = 1;
	//m_pca_descriptors_tree = new KDTree(pca_descriptors_mat);
	m_pca_descriptors_tree = new cv::flann::Index(pca_descriptors_mat,cv::flann::KDTreeIndexParams(1));
	//cvReleaseMat(&m_pca_descriptors_matrix);
	//m_pca_descriptors_tree->buildIndex();
}
#endif

void CvOneWayDescriptorObject::Allocate(int train_feature_count, int object_feature_count)
{
    CvOneWayDescriptorBase::Allocate(train_feature_count);
    m_object_feature_count = object_feature_count;

    m_part_id = new int[m_object_feature_count];
}


void CvOneWayDescriptorObject::InitializeObjectDescriptors(IplImage* train_image, const vector<KeyPointEx>& features,
                                        const char* feature_label, int desc_start_idx, float scale, int is_background)
{
    InitializeDescriptors(train_image, features, feature_label, desc_start_idx);

    for(int i = 0; i < (int)features.size(); i++)
    {
        CvPoint center = features[i].pt;

        if(!is_background)
        {
            // remember descriptor part id
            CvPoint center_scaled = cvPoint(round(center.x*scale), round(center.y*scale));
            m_part_id[i + desc_start_idx] = MatchPointToPart(center_scaled);
        }
    }
    cvResetImageROI(train_image);
}

int CvOneWayDescriptorObject::IsDescriptorObject(int desc_idx) const
{
    return desc_idx < m_object_feature_count ? 1 : 0;
}

int CvOneWayDescriptorObject::MatchPointToPart(CvPoint pt) const
{
    int idx = -1;
    const int max_dist = 10;
    for(int i = 0; i < (int)m_train_features.size(); i++)
    {
        if(length(pt - m_train_features[i].pt) < max_dist)
        {
            idx = i;
            break;
        }
    }

    return idx;
}

int CvOneWayDescriptorObject::GetDescriptorPart(int desc_idx) const
{
    //    return MatchPointToPart(GetDescriptor(desc_idx)->GetCenter());
    return desc_idx < m_object_feature_count ? m_part_id[desc_idx] : -1;
}

CvOneWayDescriptorObject::CvOneWayDescriptorObject(CvSize patch_size, int pose_count, const char* train_path,
                                                   const char* pca_config, const char* pca_hr_config, const char* pca_desc_config, int pyr_levels) :
CvOneWayDescriptorBase(patch_size, pose_count, train_path, pca_config, pca_hr_config, pca_desc_config, pyr_levels)
{
    m_part_id = 0;
}

CvOneWayDescriptorObject::~CvOneWayDescriptorObject()
{
    delete []m_part_id;
}

vector<feature_t> CvOneWayDescriptorObject::_GetLabeledFeatures() const
{
    vector<feature_t> features;
    for(size_t i = 0; i < m_train_features.size(); i++)
    {
        features.push_back(m_train_features[i]);
    }

    return features;
}

void eigenvector2image(CvMat* eigenvector, IplImage* img)
{
    CvRect roi = cvGetImageROI(img);
    if(img->depth == 32)
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                float val = cvmGet(eigenvector, 0, roi.width*y + x);
                *((float*)(img->imageData + (roi.y + y)*img->widthStep) + roi.x + x) = val;
            }
        }
    }
    else
    {
        for(int y = 0; y < roi.height; y++)
        {
            for(int x = 0; x < roi.width; x++)
            {
                float val = cvmGet(eigenvector, 0, roi.width*y + x);
                img->imageData[(roi.y + y)*img->widthStep + roi.x + x] = (unsigned char)val;
            }
        }
    }
}

void readPCAFeatures(const char* filename, CvMat** avg, CvMat** eigenvectors)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_READ);
    if(!fs)
    {
        printf("Cannot open file %s! Exiting!", filename);
        cvReleaseMemStorage(&storage);
    }

    CvFileNode* node = cvGetFileNodeByName(fs, 0, "avg");
    CvMat* _avg = (CvMat*)cvRead(fs, node);
    node = cvGetFileNodeByName(fs, 0, "eigenvectors");
    CvMat* _eigenvectors = (CvMat*)cvRead(fs, node);

    *avg = cvCloneMat(_avg);
    *eigenvectors = cvCloneMat(_eigenvectors);

    cvReleaseFileStorage(&fs);
    cvReleaseMemStorage(&storage);
}
