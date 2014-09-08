/*
 *  outlet_tuple.h
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <cstdio>

#if !defined(_OUTLET_TUPLE_H)
#define _OUTLET_TUPLE_H

#include <cv.h>
#include <vector>

#include "one_way_descriptor.h"
#include "one_way_descriptor_base.h"
#include "features.h"
#include "geometric_hash.h"

inline CvPoint cvPoint(CvPoint2D32f point)
{
	return cvPoint(int(point.x), int(point.y));
}

inline CvPoint rect_center(CvRect rect)
{
	return cvPoint(rect.x + rect.width/2, rect.y + rect.height/2);
}

inline CvPoint2D32f operator +(CvPoint2D32f p1, CvPoint2D32f p2)
{
	return cvPoint2D32f(p1.x + p2.x, p1.y + p2.y);
}

inline CvPoint2D32f operator -(CvPoint2D32f p1, CvPoint2D32f p2)
{
	return cvPoint2D32f(p1.x - p2.x, p1.y - p2.y);
}

inline CvPoint3D32f operator -(CvPoint3D32f p1, CvPoint3D32f p2)
{
	return cvPoint3D32f(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

inline CvPoint operator*(CvPoint pt, float scalar)
{
    return cvPoint(pt.x*scalar, pt.y*scalar);
}

inline float length(const CvPoint2D32f& p)
{
	return sqrt(p.x*p.x + p.y*p.y);
}

inline float length(const CvPoint3D32f& p)
{
	return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

inline CvPoint3D32f operator +(CvPoint3D32f p1, CvPoint3D32f p2)
{
	return cvPoint3D32f(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}


void readTrainingBase(const char* config_filename, char* outlet_filename,
                      char* nonoutlet_filename, std::vector<feature_t>& train_features);
void readCvPointByName(CvFileStorage* fs, CvFileNode* parent, const char* name, CvPoint& pt);

struct outlet_tuple_t
{
	CvPoint2D32f centers[4];
	std::vector<CvPoint2D32f> borders[4];
	IplImage* tuple_mask;
	CvRect roi;

	~outlet_tuple_t()
	{
		cvReleaseImage(&tuple_mask);
	}
};

typedef enum
{
    outletWhite = 0,
    outletOrange = 1
} outlet_color_t;

const float default_hole_contrast = 1.1f;

class outlet_template_t
{
public:
	outlet_template_t(int count = 4, const CvPoint2D32f* templ = 0) : 
        geometric_matcher(cv::Size(30,30), cv::Point2f(-5,-5), cv::Point2f(5, 5),
        PointMatcher::PointMatcherParams(0.5,100.0,10.0,3,5.0))
	{
        m_hole_contrast = default_hole_contrast;
		initialize(count, templ);
	};

	outlet_template_t(const outlet_template_t& outlet_templ) : geometric_matcher(cv::Size(30,30), cv::Point2f(-5,-5), cv::Point2f(5, 5))
	{
        m_hole_contrast = default_hole_contrast;
		initialize(outlet_templ.get_count(), outlet_templ.get_template());
	};

	~outlet_template_t()
	{
		delete []centers;
        if(m_base)
        {
            delete m_base;
        }
	};

public:
	void initialize(int count, const CvPoint2D32f* templ, outlet_color_t outlet_color = outletOrange)
	{
		outlet_count = count;
		centers = new CvPoint2D32f[count];
		if(templ)
		{
			memcpy(centers, templ, count*sizeof(CvPoint2D32f));
		}
		else
		{
			set_default_template();
		}

        m_base = 0;
        m_pose_count = 500;
        m_patch_size = cvSize(24, 24);
        m_outlet_color = outlet_color;
};

	int get_count() const
	{
		return outlet_count;
	};

	const CvPoint2D32f* get_template() const
	{
        return centers;
	};

    const PointMatcher& getGeometricMatcher() const {return geometric_matcher;};

	void set_default_template()
	{
		centers[0] = cvPoint2D32f(0.0f, 0.0f);
		centers[1] = cvPoint2D32f(46.0f, 0.0f);
		centers[2] = cvPoint2D32f(46.15f, 38.7f);
		centers[3] = cvPoint2D32f(-0.15f, 38.7f);
	};

    void load_one_way_descriptor_base(CvSize patch_size, int pose_count, const char* train_path,
                                  const char* train_config, const char* pca_config, const char* pca_hr_config = 0)
    {
        m_train_path = train_path;
        m_train_config = train_config;
        m_pca_config = pca_config;
        if(pca_hr_config) m_pca_hr_config = pca_hr_config;
        create_one_way_descriptor_base();
    }

    void create_one_way_descriptor_base()
    {
		//m_pose_count = 50;
		//printf("mpose: %d\n",m_pose_count);
        m_base = new CvOneWayDescriptorObject(m_patch_size, m_pose_count, m_train_path.c_str(),
                                            m_pca_config.c_str(), m_pca_hr_config.c_str(),
                                            m_pca_desc_config.c_str());
        char outlet_filename[1024];
        char nonoutlet_filename[1024];
        char train_config_filename[1024];
        std::vector<feature_t> train_features;
        sprintf(train_config_filename, "%s/%s", m_train_path.c_str(), m_train_config.c_str());
        readTrainingBase(train_config_filename, outlet_filename, nonoutlet_filename, train_features);
        m_base->SetLabeledFeatures(train_features);

        char train_image_filename[1024];
        char train_image_filename1[1024];
        sprintf(train_image_filename, "%s/%s", m_train_path.c_str(), outlet_filename);
        sprintf(train_image_filename1, "%s/%s", m_train_path.c_str(), nonoutlet_filename);

        LoadTrainingFeatures(*m_base, train_image_filename, train_image_filename1);
    }

    void initialize_geometric_hash();

    const CvOneWayDescriptorObject* get_one_way_descriptor_base() const {return m_base;};

    outlet_color_t get_color() const {return m_outlet_color;};
    void get_holes_3d(CvPoint3D32f* holes) const;
    void get_holes_3d(std::vector<cv::Point3f>& holes) const;
    void get_holes_2d(CvPoint2D32f* holes) const;

    void save(const char* filename);
    int load(const char* path);

    float GetHoleContrast() const {return m_hole_contrast;}

protected:
	int outlet_count;
	CvPoint2D32f* centers;
    CvOneWayDescriptorObject* m_base;

    std::string m_train_path;
    std::string m_train_config;
    std::string m_pca_config;
    std::string m_pca_hr_config;
    std::string m_pca_desc_config;
    CvSize m_patch_size;
    int m_pose_count;
    outlet_color_t m_outlet_color;

    PointMatcher geometric_matcher;

    float m_hole_contrast; // hole minimum contrast
};

typedef struct
{
	CvPoint2D32f center;
	float angle;
	int idx;
	CvSeq* seq;
} outlet_elem_t;


CvPoint2D32f calc_center(CvSeq* seq);
int find_dir(const CvPoint2D32f* dir, int xsign, int ysign);
int order_tuple(CvPoint2D32f* centers);
int order_tuple2(std::vector<outlet_elem_t>& tuple);

void map_point_homography(CvPoint2D32f point, CvMat* homography, CvPoint2D32f& result);
void map_vector_homography(const std::vector<CvPoint2D32f>& points, CvMat* homography, std::vector<CvPoint2D32f>& result);
CvPoint2D32f calc_center(const std::vector<CvPoint2D32f>& points);



// find_outlet_centroids: finds a tuple of 4 orange outlets.
// Input parameters:
//	src: input color image
//	outlet_tuple: the output outlet tuple
//	output_path: a path for logging the results
//	filename: filename for logging the results
// Returns 1 if a tuple is found, 0 otherwise.
int find_outlet_centroids(IplImage* img, outlet_tuple_t& outlet_tuple, const char* output_path, const char* filename);


int find_tuple(std::vector<outlet_elem_t>& candidates, CvPoint2D32f* centers);

// calc_origin_scale: function for calculating the 3D position of origin (center of the upper left outlet)
// and scale (pixels to mm in rectified image)
// Input parameters:
//	centers: tuple outlet centers (an array with 4 elements)
//	map_matrix: homography matrix from a camera image to a rectified image
//	origin: output coordinates of origin
//	scale: output scale in horizontal and vertical direction. A vector p = (p.x, p.y) in a rectified image
//		in outlets plane corresponds to p1 = (p.x*scale.x, p.y*scale.y, 0) in 3D.
void calc_origin_scale(const CvPoint2D32f* centers, CvMat* map_matrix, CvPoint3D32f* origin, CvPoint2D32f* scale);

// calc_outlet_homography: calculates homography for rectifying the outlet tuple image
// Input parameters:
//	centers: outlet centers
//	map_matrix: output homography matrix (map from a camera image to a rectified image)
//  templ: outlet template
//	inverse_map_matrix: optional inverse homography matrix
void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix,
							const outlet_template_t& templ = outlet_template_t(),
							CvMat* inverse_map_matrix = 0);

void calc_outlet_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat* map_matrix, CvSize* dst_size);

int calc_image_homography(IplImage* src, CvMat* map_matrix, CvSize* dst_size, CvPoint2D32f* hor_dir = 0,
						  CvPoint3D32f* origin = 0, CvPoint2D32f* scale = 0, const char* output_path = 0,
						  const char* filename = 0, CvPoint2D32f* _centers = 0);
IplImage* find_templates(IplImage* img, IplImage* templ);

void calc_bounding_rect(int count, const CvRect* rects, CvRect& bounding_rect);

void generate_object_points_2x1(CvPoint3D32f* points);
void generate_object_points_2x1(CvPoint2D32f* points);

// calc_camera_pose: pose estimation function. Assumes that camera calibration was done with real length.
// Input parameters:
//	intrinsic_mat: matrix of camera intrinsic parameters
//	distortion_coeffs: vector of distortion coefficients. If 0, all coefficients are assumed 0.
//  point_count: the number of points in object_points and image_points arrays
//  object_points: the 3D coordinates of the detected points
//  image_points: image coordinates of the detected points
//	rotation_vector, translation_vector: output transformation from outlet coordinate system (origin in the
//	center of the upper left outlet) to camera coordinate system
void calc_camera_pose(CvMat* intrinsic_mat, CvMat* distortion_coeffs, int point_count, const CvPoint3D32f*object_points,
                      const CvPoint2D32f* image_points, CvMat* rotation_vector, CvMat* translation_vector);

void calc_camera_pose_2x1(CvMat* intrinsic_mat, CvMat* distortion_coeffs, const CvPoint2D32f* centers,
                          CvMat* rotation_vector, CvMat* translation_vector);

void calc_camera_pose_2x2(CvMat* intrinsic_mat, CvMat* distortion_coeffs, const CvPoint2D32f* centers,
                          CvMat* rotation_vector, CvMat* translation_vector);

void calc_camera_outlet_pose(CvMat* intrinsic_mat, CvMat* distortion_coeffs, const outlet_template_t& outlet_template,
                             const CvPoint2D32f* image_points, CvMat* rotat, CvMat* translation_vector);

CvSeq* close_seq(CvSeq* seq, CvMemStorage* storage, int closure_dist, IplImage* workspace);


#endif //_OUTLET_TUPLE_H
