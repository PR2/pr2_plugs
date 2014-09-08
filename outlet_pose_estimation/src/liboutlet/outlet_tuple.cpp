/*
 *  outlet_tuple.cpp
 *  rectify_outlets
 *
 *  Created by Victor  Eruhimov on 1/25/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

//*****************************************************************************************
// Warning: this is research code with poor architecture, performance and no documentation!
//*****************************************************************************************

#include <limits.h>
#include <math.h>
#include <stdio.h>

#include <vector>
#include <algorithm>
using namespace std;

#include "outlet_pose_estimation/detail/outlet_tuple.h"
#include <highgui.h>
#include <cvwimage.h>

#include "outlet_pose_estimation/detail/one_way_descriptor.h"

const float pi = 3.1415926f;
static const char template_filename[] = "outlet_template.yml";


CvPoint2D32f calc_center(CvSeq* seq)
{
	CvMoments moments;
	cvMoments(seq, &moments);
	CvPoint2D32f center;
	center.x = cvGetSpatialMoment(&moments, 1, 0);
	center.y = cvGetSpatialMoment(&moments, 0, 1);
	float area = cvGetSpatialMoment(&moments, 0, 0);
	center.x /= area;
	center.y /= area;

	return center;
}

CvPoint2D32f calc_center(const vector<CvPoint2D32f>& points)
{
	CvPoint2D32f center = cvPoint2D32f(0, 0);
	for(unsigned int i = 0; i < points.size(); i++)
	{
		center.x += points[i].x;
		center.y += points[i].y;
	}

	center.x /= points.size();
	center.y /= points.size();

	return(center);
}

int find_dir(const CvPoint2D32f* dir, int xsign, int ysign)
{
	for(int i = 0; i < 4; i++)
	{
		if(dir[i].x*xsign > 0 && dir[i].y*ysign > 0)
		{
			return i;
		}
	}
	return -1;
}

int order_tuple(CvPoint2D32f* centers)
{
	CvPoint2D32f ordered[4];
	int idx[4];

	CvPoint2D32f center = cvPoint2D32f(0.0f, 0.0f);
	for(int i = 0; i < 4; i++)
	{
		center.x += centers[i].x;
		center.y += centers[i].y;
	}
	center.x *= 0.25f;
	center.y *= 0.25f;

	CvPoint2D32f dir[4];
	for(int i = 0; i < 4; i++)
	{
		dir[i].x = centers[i].x - center.x;
		dir[i].y = centers[i].y - center.y;
	}

	idx[0] = find_dir(dir, -1, -1);
	idx[1] = find_dir(dir, 1, -1);
	idx[2] = find_dir(dir, 1, 1);
	idx[3] = find_dir(dir, -1, 1);

	// check if any of the tuples are not found
	int found[4] = {-1,-1,-1,-1};
	int count_lost = 0;
	int idx_lost = 0;
	for(int i = 0; i < 4; i++)
	{
		if(idx[i] != -1)
		{
			found[idx[i]] = 1;
		}
		else
		{
			idx_lost = i;
			count_lost++;
		}
	}

	if(count_lost > 1)
	{
		printf("%d outlets cannot be ordered, not enough for a tuple\n", count_lost);
		return 0;
	}

	for(int i = 0; i < 4; i++)
	{
		if(found[i] == -1)
		{
			idx[idx_lost] = i;
		}
	}

	for(int i = 0; i < 4; i++)
	{
		ordered[i] = centers[idx[i]];
	}

	for(int i = 0; i < 4; i++)
	{
		centers[i] = ordered[i];
	}

	return 1;
}

bool helper_pred_greater(outlet_elem_t h1, outlet_elem_t h2)
{
	return h1.angle < h2.angle;
}

int find_start_idx(const vector<outlet_elem_t>& helper_vec)
{
	int start_idx = -1;
	float min_angle_diff = 1e10;
	for(int i = 0; i < 4; i++)
	{
		float angle_diff = fabs(helper_vec[i].angle + 4*pi/5);
		if(angle_diff < min_angle_diff)
		{
			min_angle_diff = angle_diff;
			start_idx = i;
		}
	}

	return start_idx;

}


int find_start_idx2(const vector<outlet_elem_t>& helper_vec)
{
	for(int i = 0; i < 4; i++)
	{
		// find the fartherst point
		float max_dist = 0;
		int max_j = -1;
		for(int j = 0; j < 4; j++)
		{
			if(j == i) continue;

			float dist = length(helper_vec[i].center - helper_vec[j].center);
			if(dist > max_dist)
			{
				max_dist = dist;
				max_j = j;
			}
		}

		if(helper_vec[i].center.x < helper_vec[max_j].center.x)
		{
			return i;
		}
	}

	assert(0);
	return -1; // should never reach this point
}

int find_start_idx3(const vector<outlet_elem_t>& helper_vec)
{
	const float mean_angle = -3*pi/4;
	float max_angle = pi/2*0.8f;;
	for(int i = 0; i < 4; i++)
	{
		float angle = helper_vec[i].angle - mean_angle;
		if(angle > pi) angle -= 2*pi;
		if(fabs(angle) > max_angle)
		{
			continue;
		}

		int prev_idx = (i + 3)%4;
		int post_idx = (i + 1)%4;
		CvPoint2D32f prev_vec = helper_vec[prev_idx].center - helper_vec[i].center;
		CvPoint2D32f post_vec = helper_vec[post_idx].center - helper_vec[i].center;
		float l1 = length(prev_vec);
		float l2 = length(post_vec);
		if(l2 > l1 && post_vec.x > 0)
		{
			return i;
		}
	}

	// this is not a tuple
	return -1;

}

int order_tuple2(vector<outlet_elem_t>& tuple)
{
	vector<outlet_elem_t> ordered;

	CvPoint2D32f center = cvPoint2D32f(0.0f, 0.0f);
	for(int i = 0; i < 4; i++)
	{
		center.x += tuple[i].center.x;
		center.y += tuple[i].center.y;
	}
	center.x *= 0.25f;
	center.y *= 0.25f;

	CvPoint2D32f dir[4];
	for(int i = 0; i < 4; i++)
	{
		dir[i].x = tuple[i].center.x - center.x;
		dir[i].y = tuple[i].center.y - center.y;

		tuple[i].angle = atan2(dir[i].y, dir[i].x);
		tuple[i].idx = i;
	}
	sort(tuple.begin(), tuple.end(), helper_pred_greater);

#if 0
	int start_idx = find_start_idx(helper_vec);
#else
	int start_idx = find_start_idx3(tuple);
    if(start_idx < 0)
    {
        return 0;
    }
#endif

	ordered = tuple;
	for(int i = 0; i < 4; i++)
	{
		int _i = (i + start_idx)%4;
		ordered[i] = tuple[_i];
	}

	tuple = ordered;
	return 1;
}

int find_outlet_centroids(IplImage* img, outlet_tuple_t& outlet_tuple, const char* output_path, const char* filename)
{
    cv::WImageBuffer1_b grey_buf(img->width, img->height);
    cv::WImageBuffer1_b binary_buf(img->width, img->height);
    IplImage* grey = grey_buf.Ipl();
	IplImage* binary = binary_buf.Ipl();
	cvCvtColor(img, grey, CV_RGB2GRAY);
	cvSmooth(grey, grey);
	cvAdaptiveThreshold(grey, binary, 255, CV_ADAPTIVE_THRESH_MEAN_C,
						CV_THRESH_BINARY_INV, 13, -1);
    IplImage* _binary = cvCloneImage(binary);
    CvMemStorage* storage = cvCreateMemStorage();
	int found_tuple = 0;

    for(int dstep = 1; dstep < 10; dstep++)
    {
        cvErode(_binary, binary, 0, dstep);
        cvDilate(binary, binary, 0, dstep);

#if defined(_VERBOSE_TUPLE)
        cvNamedWindow("1", 1);
        cvShowImage("1", binary);
        cvWaitKey(0);
        cvSaveImage("mask.jpg", binary);
#endif

        CvSeq* first = 0;
        cvFindContours(binary, storage, &first, sizeof(CvContour), CV_RETR_CCOMP);
        vector<CvSeq*> candidates;

#if defined(_VERBOSE_TUPLE)
        IplImage* img1 = cvCloneImage(img);
#endif //_VERBOSE_TUPLE

        cv::WImageBuffer1_b mask_buf( cvCloneImage(grey) );
        IplImage* mask = mask_buf.Ipl();
        for(CvSeq* seq = first; seq != NULL; seq = seq->h_next)
        {
            CvRect rect = cvBoundingRect(seq);
#if !defined(_OUTLET_HR)
            const int xmin = 30;
            const int xmax = 150;
            const int ymin = 15;
            const int ymax = 50;
            const int min_width = 5;
#else
            const int xmin = 50;
            const int xmax = 400;
            const int ymin = 30;
            const int ymax = 400;
            const int min_width = 5;
#endif //OUTLET_HR


            if(rect.width < xmin || rect.width > xmax || rect.height < ymin || rect.height > ymax)
            {
                continue;
            }

#if 0
            if(abs(rect.x - 1312) < 50 && abs(rect.y - 1576) < 50)
            {
                int w = 1;
            }
#endif

            float area = fabs(cvContourArea(seq));
            float perimeter = fabs(cvArcLength(seq));

            if(area/perimeter*2 < min_width)
            {
                continue;
            }

            //cvSetImageROI(img, rect);
            cvSetZero(mask);
            cvDrawContours(mask, seq, cvScalar(255), cvScalar(255), 0, CV_FILLED);

            CvScalar mean = cvAvg(img, mask);
            //cvResetImageROI(img);
            if(mean.val[2]/mean.val[1] < 1.8f)
            {
                continue;
            }

            candidates.push_back(seq);

#if defined(_VERBOSE_TUPLE)
            cvDrawContours(img1, seq, CV_RGB(255, 0, 0), CV_RGB(255, 0, 0), 0, 2);
#endif //_VERBOSE_TUPLE
        }

#if defined(_VERBOSE_TUPLE)
        cvNamedWindow("1", 1);
        cvShowImage("1", img1);
        cvWaitKey(0);
        cvReleaseImage(&img1);
#endif //_VERBOSE_TUPLE

        vector<outlet_elem_t> tuple;
        for(unsigned int i = 0; i < candidates.size(); i++)
        {
            vector<outlet_elem_t> tuple_candidates;
            outlet_elem_t outlet_elem;
            outlet_elem.seq = candidates[i];
            outlet_elem.center = calc_center(candidates[i]);
            tuple_candidates.push_back(outlet_elem);

            CvRect rect1 = cvBoundingRect(candidates[i]);
            for(unsigned int j = 0; j < candidates.size(); j++)
            {
                if(j <= i) continue;
                CvRect rect2 = cvBoundingRect(candidates[j]);

                // test the pair
                CvPoint center1 = rect_center(rect1);
                CvPoint center2 = rect_center(rect2);

                if(2.0f*(rect1.width - rect2.width)/(rect1.width + rect2.width) > 0.3f ||
                   2.0f*(rect1.height - rect2.height)/(rect1.height + rect2.height) > 0.3f)
                {
                    continue;
                }

                float dist = sqrt(float(center1.x - center2.x)*(center1.x - center2.x) +
                                  (center1.y - center2.y)*(center1.y - center2.y));
                if(dist > 4*rect1.width)
                {
                    continue;
                }

                // found pair, add rect2 to the tuple
                outlet_elem_t outlet_elem;
                outlet_elem.seq = candidates[j];
                outlet_elem.center = calc_center(candidates[j]);
                tuple_candidates.push_back(outlet_elem);
            }

            // find the tuple
            found_tuple = find_tuple(tuple_candidates, outlet_tuple.centers);
            if(found_tuple == 1)
            {
                // found the tuple!
                tuple = tuple_candidates;
                break;
            }
        }

#if defined(_VERBOSE_TUPLE) || defined(_VERBOSE)
        IplImage* img2 = cvCloneImage(img);
#endif //_VERBOSE_TUPLE

        if(found_tuple == 1)
        {
            for(int i = 0; i < 4; i++)
            {
                tuple[i].seq = close_seq(tuple[i].seq, storage, 10, binary);
                if(tuple[i].seq == 0)
                {
                    found_tuple = 0;
                    break;
                }
            }

            if(found_tuple == 0) break;
            // draw the mask
            if(outlet_tuple.tuple_mask)
            {
                cvSetZero(outlet_tuple.tuple_mask);
                for(int i = 0; i < 4; i++)
                {
                    cvDrawContours(outlet_tuple.tuple_mask, tuple[i].seq, cvScalar(i + 1), cvScalar(i + 1), 0, CV_FILLED);
                }
            }

            // calculate the tuple roi
            CvRect tuple_roi[4];
            for(int i = 0; i < 4; i++)
            {
                tuple_roi[i] = cvBoundingRect(tuple[i].seq);
            }
            calc_bounding_rect(4, tuple_roi, outlet_tuple.roi);

#if defined(_VERBOSE_TUPLE) || defined(_VERBOSE)
            cvCircle(img2, cvPoint(outlet_tuple.centers[0]), 10, CV_RGB(0, 255, 0));
            cvCircle(img2, cvPoint(outlet_tuple.centers[1]), 10, CV_RGB(0, 0, 255));
            cvCircle(img2, cvPoint(outlet_tuple.centers[2]), 10, CV_RGB(255, 255, 255));
            cvCircle(img2, cvPoint(outlet_tuple.centers[3]), 10, CV_RGB(0, 255, 255));
#endif //VERBOSE_TUPLE

            // save outlet borders
            for(int i = 0; i < 4; i++)
            {
                for(int j = 0; j < tuple[i].seq->total; j++)
                {
                    CvPoint* p = (CvPoint*)cvGetSeqElem(tuple[i].seq, j);
                    outlet_tuple.borders[i].push_back(cvPoint2D32f(p->x, p->y));
                }
            }
        }

#if defined(_VERBOSE_TUPLE)
        cvNamedWindow("1", 1);
        cvShowImage("1", img2);
        cvWaitKey(0);

        cvThreshold(outlet_tuple.tuple_mask, binary, 0, 255, CV_THRESH_BINARY);
        cvShowImage("1", binary);
        cvWaitKey(0);

#endif //_VERBOSE_TUPLE


#if defined(_VERBOSE)
        if(output_path && filename)
        {
            char buf[1024];
            sprintf(buf, "%s/warped/%s", output_path, filename);
            cvSaveImage(buf, img2);
        }
#endif //_VERBOSE

#if defined(_VERBOSE) || defined(_VERBOSE_TUPLE)
        cvReleaseImage(&img2);
#endif
        if(found_tuple == 1) break;
    }

	cvReleaseMemStorage(&storage);
    cvReleaseImage(&_binary);

	return found_tuple;

}

CvSeq* close_seq(CvSeq* seq, CvMemStorage* storage, int closure_dist, IplImage* workspace)
{
    cvSetZero(workspace);
    cvDrawContours(workspace, seq, cvScalar(255), cvScalar(255), 0, CV_FILLED);
    cvDilate(workspace, workspace, 0, closure_dist);
    cvErode(workspace, workspace, 0, closure_dist);

    CvSeq* first = 0;
    cvFindContours(workspace, storage, &first, sizeof(CvContour), CV_RETR_LIST);
    CvSeq* hull = cvConvexHull2(first, storage, CV_CLOCKWISE, 1);
    return(hull);
}

void calc_bounding_rect(int count, const CvRect* rects, CvRect& bounding_rect)
{
	CvPoint tl = cvPoint(INT_MAX, INT_MAX); // top left point
	CvPoint br = cvPoint(INT_MIN, INT_MIN); //bottom right point
	for(int i = 0; i < count; i++)
	{
		tl.x = MIN(tl.x, rects[i].x);
		tl.y = MIN(tl.y, rects[i].y);
		br.x = MAX(br.x, rects[i].x + rects[i].width);
		br.y = MAX(br.y, rects[i].y + rects[i].height);
	}

	bounding_rect.x = tl.x;
	bounding_rect.y = tl.y;
	bounding_rect.width = br.x - tl.x;
	bounding_rect.height = br.y - tl.y;
}

int find_tuple(vector<outlet_elem_t>& candidates, CvPoint2D32f* centers)
{
	if(candidates.size() < 4)
	{
		// should be at least 4 candidates for a tuple
		return 0;
	}

	if(candidates.size() > 15)
	{
		// too many candidates -- the algorithm will be slow
		return 0;
	}

	if(candidates.size() == 4)
	{
		// we've got a tuple!
		int ret = order_tuple2(candidates);
        if(ret == 0)
        {
            return 0;
        }

		for(int i = 0; i < 4; i++)
		{
			centers[i] = candidates[i].center;
		}
		return 1;
	}

	printf("find_tuple: The case of more than 4 candidates is not yet supported!\n");
	return 0;

/*
	// look for affine-transformed rectangle with outlet centers in its corners
	// for doing that iterate through all possible 4-tuples
	int idx[4] = {0, 0, 0, -1};
	while(1)
	{
		// move to the next 4-tuple
		for(int i = 3; i >= 0; i--)
		{
			if(
	}
 */
}

const int outlet_width = 50;
const int outlet_height = 25;

const float xsize = 46.1f;
const float ysize = 38.7f;
const float outlet_xsize = 12.37; // mm, the distance between the holes
const float outlet_ysize = 11.5; // mm, the distance from the ground hole to the power holes

void calc_outlet_homography(const CvPoint2D32f* centers, CvMat* map_matrix,
							const outlet_template_t& templ, CvMat* inverse_map_matrix)
{
	CvPoint2D32f rectified[4];

#if 0
	rectified[0] = centers[0];
	rectified[1] = cvPoint2D32f(centers[0].x + outlet_width, centers[0].y);
	rectified[2] = cvPoint2D32f(centers[0].x + outlet_width, centers[0].y + outlet_height);
	rectified[3] = cvPoint2D32f(centers[0].x, centers[0].y + outlet_height);
#else
	memcpy(rectified, templ.get_template(), templ.get_count()*sizeof(CvPoint2D32f));
#endif

	cvGetPerspectiveTransform(centers, rectified, map_matrix);

	if(inverse_map_matrix)
	{
		cvGetPerspectiveTransform(rectified, centers, inverse_map_matrix);
	}
}

void map_point_homography(CvPoint2D32f point, CvMat* homography, CvPoint2D32f& result)
{
	CvMat* src = cvCreateMat(1, 1, CV_32FC2);
	CvMat* dst = cvCreateMat(1, 1, CV_32FC2);

    src->data.fl[0] = point.x;
    src->data.fl[1] = point.y;

    cvPerspectiveTransform(src, dst, homography);

    result.x = dst->data.fl[0];
    result.y = dst->data.fl[1];

    cvReleaseMat(&src);
    cvReleaseMat(&dst);
}

void map_vector_homography(const vector<CvPoint2D32f>& points, CvMat* homography, vector<CvPoint2D32f>& result)
{
	int points_count = points.size();
	CvMat* src = cvCreateMat(1, points_count, CV_32FC2);
	CvMat* dst = cvCreateMat(1, points_count, CV_32FC2);

	for(unsigned int i = 0; i < points.size(); i++)
	{
		src->data.fl[2*i] = points[i].x;
		src->data.fl[2*i + 1] = points[i].y;
	}

	cvPerspectiveTransform(src, dst, homography);

	result.clear();
	for(int i = 0; i < points_count; i++)
	{
		result.push_back(cvPoint2D32f(dst->data.fl[2*i], dst->data.fl[2*i + 1]));
	}

	cvReleaseMat(&src);
	cvReleaseMat(&dst);
}

void map_image_corners(CvSize src_size, CvMat* map_matrix, CvMat* corners, CvMat* dst)
{
	corners->data.fl[0] = 0;
	corners->data.fl[1] = 0;
	corners->data.fl[2] = src_size.width;
	corners->data.fl[3] = 0;
	corners->data.fl[4] = src_size.width;
	corners->data.fl[5] = src_size.height;
	corners->data.fl[6] = 0;
	corners->data.fl[7] = src_size.height;
	cvPerspectiveTransform(corners, dst, map_matrix);
}

//void calc_image_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat** xmap, CvMat** ymap, CvSize* dst_size)
void calc_outlet_homography(const CvPoint2D32f* centers, CvSize src_size, CvMat* map_matrix, CvSize* dst_size)
{
//	CvMat* map_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* inverse_map_matrix = cvCreateMat(3, 3, CV_32FC1);
	calc_outlet_homography(centers, map_matrix, outlet_template_t(), inverse_map_matrix);

	CvMat* corners = cvCreateMat(1, 4, CV_32FC2);
	CvMat* dst = cvCreateMat(1, 4, CV_32FC2);
	map_image_corners(src_size, map_matrix, corners, dst);

	float xmin = 1e10, ymin = 1e10, xmax = -1e10, ymax = -1e10;
	float src_xmin, src_ymin;
	for(int i = 0; i < 4; i++)
	{
		if(xmin > dst->data.fl[2*i])
		{
			xmin = dst->data.fl[2*i];
			src_ymin = corners->data.fl[2*i + 1];
		}
		if(xmax < dst->data.fl[2*i])
		{
			xmax = dst->data.fl[2*i];
//			src_xmax = corners->data.fl[2*i];
		}
		if(ymin > dst->data.fl[2*i + 1])
		{
			ymin = dst->data.fl[2*i + 1];
			src_xmin = corners->data.fl[2*i];
		}
		if(ymax < dst->data.fl[2*i + 1])
		{
			ymax = dst->data.fl[2*i + 1];
//			src_ymax = corners->data.fl[2*i + 1];
		}
	}

	/// !!!! This is an ugly hack. The real way to do it is to offset homography transform
	if(dst_size)
	{
		dst_size->width = xmax;//ceil(xmax - xmin) + 1;
		dst_size->height = ymax;//ceil(ymax - ymin) + 1;
	}
/*
	CvMat* src_points = cvCreateMat(1, dst_size->width*dst_size->height, CV_32FC2);
	CvMat* dst_points = cvCreateMat(1, dst_size->width*dst_size->height, CV_32FC2);
	for(int r = 0; r < dst_size->height; r++)
	{
		for(int c = 0; c < dst_size->width; c++)
		{
			dst_points->data.fl[r*dst_size->width + c*2] = c;// - xmin;
			dst_points->data.fl[r*dst_size->width + c*2 + 1] = r;// - ymin;
		}
	}
	cvPerspectiveTransform(dst_points, src_points, inverse_map_matrix);

	*xmap = cvCreateMat(dst_size->height, dst_size->width, CV_32FC1);
	*ymap = cvCreateMat(dst_size->height, dst_size->width, CV_32FC1);
	for(int r = 0; r < dst_size->height; r++)
	{
		for(int c = 0; c < dst_size->width; c++)
		{
			cvmSet(*xmap, r, c, src_points->data.fl[r*dst_size->width + c*2]);
			cvmSet(*ymap, r, c, src_points->data.fl[r*dst_size->width + c*2 + 1]);
		}
	}

	cvReleaseMat(&src_points);
	cvReleaseMat(&dst_points);
 */
	cvReleaseMat(&corners);
	cvReleaseMat(&dst);
}

int calc_image_homography(IplImage* src, CvMat* map_matrix, CvSize* dst_size, CvPoint2D32f* hor_dir, CvPoint3D32f* origin,
						  CvPoint2D32f* scale, const char* output_path, const char* filename, CvPoint2D32f* _centers)
{
	outlet_tuple_t outlet_tuple;
	outlet_tuple.tuple_mask = 0;
	int ret = find_outlet_centroids(src, outlet_tuple, output_path, filename);
	if(!ret)
	{
		printf("Centroids not found\n");
		return 0;
	}

	if(_centers)
	{
		memcpy(_centers, outlet_tuple.centers, 4*sizeof(CvPoint2D32f));
	}

	if(hor_dir)
	{
		hor_dir->x = outlet_tuple.centers[1].x - outlet_tuple.centers[0].x;
		hor_dir->y = outlet_tuple.centers[1].y - outlet_tuple.centers[0].y;
	}

	calc_outlet_homography(outlet_tuple.centers, cvSize(src->width, src->height), map_matrix, dst_size);

	calc_origin_scale(outlet_tuple.centers, map_matrix, origin, scale);

	return 1;
}

void calc_origin_scale(const CvPoint2D32f* centers, CvMat* map_matrix, CvPoint3D32f* origin, CvPoint2D32f* scale)
{
	float scalex, scaley;
	if(origin)
	{
		CvMat* _src = cvCreateMat(1, 3, CV_32FC2);
		CvMat* _dst = cvCreateMat(1, 3, CV_32FC2);

		_src->data.fl[0] = centers[0].x;
		_src->data.fl[1] = centers[0].y;
		_src->data.fl[2] = centers[1].x;
		_src->data.fl[3] = centers[1].y;
		_src->data.fl[4] = centers[2].x;
		_src->data.fl[5] = centers[2].y;

		cvPerspectiveTransform(_src, _dst, map_matrix);

		origin->x = _dst->data.fl[0];
		origin->y = _dst->data.fl[1];
		origin->z = 0;

#if 0
		scalex = xsize/(_dst->data.fl[2] - _dst->data.fl[0]);
		scaley = ysize/(_dst->data.fl[5] - _dst->data.fl[3]);
#else
		scalex = scaley = 1.0f;
#endif

		cvReleaseMat(&_src);
		cvReleaseMat(&_dst);
	}

	if(scale)
	{
		*scale = cvPoint2D32f(scalex, scaley);
	}
}

IplImage* find_templates(IplImage* img, IplImage* templ)
{
	IplImage* templr = cvCreateImage(cvSize(outlet_width, outlet_height), IPL_DEPTH_8U, 3);
	cvResize(templ, templr);

	IplImage* dist = cvCreateImage(cvSize(img->width - templr->width + 1, img->height - templr->height + 1), IPL_DEPTH_32F, 1);
	cvMatchTemplate(img, templr, dist, CV_TM_SQDIFF);

	double max_dist, min_dist;
	cvMinMaxLoc(dist, &min_dist, &max_dist);

	IplImage* mask = cvCreateImage(cvSize(dist->width, dist->height), IPL_DEPTH_8U, 1);

	double thresh = min_dist*2.0f;
	cvThreshold(dist, mask, thresh, 255, CV_THRESH_BINARY_INV);

	for(int r = 0; r < dist->height; r++)
	{
		for(int c = 0; c < dist->width; c++)
		{
			if(mask->imageData[r*mask->widthStep + c] == 0)
			{
				continue;
			}

//			cvCircle(img, cvPoint(c + templr->width/2, r + templr->height/2), 20, CV_RGB(color, color, 0), 3);
//			cvRectangle(img, cvPoint(c, r), cvPoint(c + templr->width, r + templr->height), CV_RGB(color, color, 0), 2);
			cvRectangle(img, cvPoint(c, r), cvPoint(c + templr->width, r + templr->height), CV_RGB(0, 0, 255), 2);
		}
	}

	cvReleaseImage(&templr);
	cvReleaseImage(&dist);

	return mask;
}

void calc_camera_outlet_pose(CvMat* intrinsic_mat, CvMat* distortion_coeffs, const outlet_template_t& outlet_template,
                             const CvPoint2D32f* image_points, CvMat* rotation_vector, CvMat* translation_vector)
{
    if(outlet_template.get_count() == 4 && outlet_template.get_color() == outletOrange)
    {
        calc_camera_pose_2x2(intrinsic_mat, distortion_coeffs, image_points, rotation_vector, translation_vector);
    }

    if(outlet_template.get_count() == 2)
    {
        calc_camera_pose_2x1(intrinsic_mat, distortion_coeffs, image_points, rotation_vector, translation_vector);
    }
}

void generate_object_points_2x1(CvPoint3D32f* points)
{
    const float outlet_width = 12.37f; //mm
    const float ground_hole_offset = 11.5f; //mm
    points[0] = cvPoint3D32f(-outlet_width/2, 0.0f, 0.0f);
    points[1] = cvPoint3D32f(outlet_width/2, 0.0f, 0.0f);
    points[2] = cvPoint3D32f(-outlet_width/2, ysize, 0.0f);
    points[3] = cvPoint3D32f(outlet_width/2, ysize, 0.0f);
    points[4] = cvPoint3D32f(0.0f, -ground_hole_offset, 0.0f);
    points[5] = cvPoint3D32f(0.0f, ysize - ground_hole_offset, 0.0f);
}

void generate_object_points_2x1(CvPoint2D32f* points)
{
    const float outlet_width = 12.37f; //mm
    const float ground_hole_offset = 11.5f; //mm
    points[0] = cvPoint2D32f(-outlet_width/2, 0.0f);
    points[1] = cvPoint2D32f(outlet_width/2, 0.0f);
    points[2] = cvPoint2D32f(-outlet_width/2, ysize);
    points[3] = cvPoint2D32f(outlet_width/2, ysize);
    points[4] = cvPoint2D32f(0.0f, -ground_hole_offset);
    points[5] = cvPoint2D32f(0.0f, ysize - ground_hole_offset);
}

void calc_camera_pose_2x1(CvMat* intrinsic_mat, CvMat* distortion_coeffs, const CvPoint2D32f* centers,
                          CvMat* rotation_vector, CvMat* translation_vector)
{
    CvPoint3D32f object_points[6];
    generate_object_points_2x1(object_points);

    calc_camera_pose(intrinsic_mat, distortion_coeffs, 6, object_points, centers, rotation_vector, translation_vector);
}

void calc_camera_pose_2x2(CvMat* intrinsic_mat, CvMat* distortion_coeffs, const CvPoint2D32f* centers,
                          CvMat* rotation_vector, CvMat* translation_vector)
{
    CvPoint3D32f object_points[] = {
        cvPoint3D32f(0.0f, 0.0f, 0.0f),
        cvPoint3D32f(xsize, 0.0f, 0.0f),
        cvPoint3D32f(xsize, ysize, 0.0f),
        cvPoint3D32f(0.0f, ysize, 0.0f)
        };

    calc_camera_pose(intrinsic_mat, distortion_coeffs, 4, object_points, centers, rotation_vector, translation_vector);
}

void calc_camera_pose(CvMat* intrinsic_mat, CvMat* distortion_coeffs, int point_count, const CvPoint3D32f* object_points,
                      const CvPoint2D32f* image_points, CvMat* rotation_vector, CvMat* translation_vector)
{
	CvMat* object_mat = cvCreateMat(point_count, 3, CV_32FC1);
	CvMat* image_mat = cvCreateMat(point_count, 2, CV_32FC1);

    for(int i = 0; i < point_count; i++)
    {
        cvmSet(object_mat, i, 0, object_points[i].x);
        cvmSet(object_mat, i, 1, object_points[i].y);
        cvmSet(object_mat, i, 2, object_points[i].z);
    }

	for(int i = 0; i < point_count; i++)
	{
		cvmSet(image_mat, i, 0, image_points[i].x);
		cvmSet(image_mat, i, 1, image_points[i].y);
	}

	CvMat* _distortion_coeffs = 0;
	if(distortion_coeffs == 0)
	{
		_distortion_coeffs = cvCreateMat(1, 5, CV_32FC1);
		for(int i = 0; i < 5; i++) cvmSet(_distortion_coeffs, 0, i, 0.0f);
	}
	else
	{
		_distortion_coeffs = distortion_coeffs;
	}

	cvFindExtrinsicCameraParams2(object_mat, image_mat, intrinsic_mat, _distortion_coeffs, rotation_vector,
								 translation_vector);

	if(distortion_coeffs == 0)
	{
		cvReleaseMat(&_distortion_coeffs);
	}

	cvReleaseMat(&object_mat);
	cvReleaseMat(&image_mat);
}

void outlet_template_t::save(const char* filename)
{
    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(filename, storage, CV_STORAGE_WRITE);

    cvWriteInt(fs, "outlet count", outlet_count);
    for(int i = 0; i < outlet_count; i++)
    {
        char buf[1024];

        sprintf(buf, "outlet %d center x", i);
        cvWriteReal(fs, buf, centers[i].x);

        sprintf(buf, "outlet %d center y", i);
        cvWriteReal(fs, buf, centers[i].y);
    }

    cvWriteString(fs, "train path", m_train_path.c_str());
    cvWriteString(fs, "train config", m_train_config.c_str());
    cvWriteString(fs, "pca config", m_pca_config.c_str());
    cvWriteInt(fs, "patch width", m_patch_size.width);
    cvWriteInt(fs, "patch height", m_patch_size.height);
    cvWriteInt(fs, "pose count", m_pose_count);

    if(m_outlet_color == outletWhite)
    {
        cvWriteString(fs, "outlet color", "white");
    }

    if(m_outlet_color == outletOrange)
    {
        cvWriteString(fs, "outlet color", "orange");
    }

    cvWriteReal(fs, "hole contrast", m_hole_contrast);

    cvReleaseFileStorage(&fs);
    cvReleaseMemStorage(&storage);
}

int outlet_template_t::load(const char* path)
{
    m_train_path = string(path);

    char buf[1024];
    sprintf(buf, "%s/%s", path, template_filename);

    CvMemStorage* storage = cvCreateMemStorage();
    CvFileStorage* fs = cvOpenFileStorage(buf, storage, CV_STORAGE_READ);

    CvFileNode* node = cvGetFileNodeByName(fs, 0, "outlet count");
    if(!node)
    {
        cvReleaseFileStorage(&fs);
        cvReleaseMemStorage(&storage);
        return 0;
    }

    int _outlet_count = cvReadInt(node);

    CvPoint2D32f* _outlet_centers = new CvPoint2D32f[_outlet_count];

    for(int i = 0; i < _outlet_count; i++)
    {
        char buf[1024];

        sprintf(buf, "outlet %d center x", i);
        node = cvGetFileNodeByName(fs, 0, buf);
        _outlet_centers[i].x = cvReadReal(node);

        sprintf(buf, "outlet %d center y", i);
        node = cvGetFileNodeByName(fs, 0, buf);
        _outlet_centers[i].y = cvReadReal(node);
    }

    initialize(_outlet_count, _outlet_centers);

    node = cvGetFileNodeByName(fs, 0, "train config");
    if(node)
    {
        const char* train_config = cvReadString(node);
        m_train_config = string(train_config);
    }

    node = cvGetFileNodeByName(fs, 0, "pca config");
    if(node)
    {
        const char* pca_config = cvReadString(node);
        m_pca_config = string(pca_config);
    }

    node = cvGetFileNodeByName(fs, 0, "pca hr config");
    if(node)
    {
        const char* pca_hr_config = cvReadString(node);
        m_pca_hr_config = string(pca_hr_config);
    }

    node = cvGetFileNodeByName(fs, 0, "pca descriptors");
    if(node)
    {
        const char* pca_desc_config = cvReadString(node);
        m_pca_desc_config = string(pca_desc_config);
    }

    node = cvGetFileNodeByName(fs, 0, "patch width");
    if(node)
    {
        m_patch_size.width = cvReadInt(node);
    }

    node = cvGetFileNodeByName(fs, 0, "patch height");
    if(node)
    {
        m_patch_size.height = cvReadInt(node);
    }

    node = cvGetFileNodeByName(fs, 0, "pose count");
    if(node)
    {
        m_pose_count = cvReadInt(node);
    }

    node =cvGetFileNodeByName(fs, 0, "outlet color");
    if(node)
    {
        const char* outlet_color = cvReadString(node);
        if(strcmp(outlet_color, "white") == 0)
        {
            m_outlet_color = outletWhite;
        }
        if(strcmp(outlet_color, "orange") == 0)
        {
            m_outlet_color = outletOrange;
        }
    }

    node = cvGetFileNodeByName(fs, 0, "hole contrast");
    if(node)
    {
        m_hole_contrast = (float)cvReadReal(node);
    }


    cvReleaseFileStorage(&fs);
    cvReleaseMemStorage(&storage);
    create_one_way_descriptor_base();

    initialize_geometric_hash();

    delete [] _outlet_centers;

    return 1;
}

void outlet_template_t::get_holes_3d(CvPoint3D32f* holes) const
{
    const CvPoint2D32f* centers = get_template();

    for(int i = 0; i < get_count(); i++)
    {
        CvPoint2D32f center = centers[i];
        float alpha = 1.0f + 1.0f/60;
        holes[3*i] = cvPoint3D32f((center.x - outlet_xsize/2)*alpha, center.y*alpha, 0.0f); // power left
        holes[3*i + 1] = cvPoint3D32f((center.x + outlet_xsize/2)*alpha, center.y*alpha, 0.0f); // power right
        holes[3*i + 2] = cvPoint3D32f(center.x*alpha, (center.y - outlet_ysize)*alpha, 0.0f); // ground
    }
}

void outlet_template_t::get_holes_3d(std::vector<cv::Point3f>& holes) const
{
    const CvPoint2D32f* centers = get_template();
    holes.resize(get_count()*3);

    for(int i = 0; i < get_count(); i++)
    {
        CvPoint2D32f center = centers[i];
        float alpha = 1.0f + 0.7f/30;
        holes[3*i] = cv::Point3f(center.x - outlet_xsize/2, center.y, 0.0f)*alpha; // power left
        holes[3*i + 1] = cv::Point3f(center.x + outlet_xsize/2, center.y, 0.0f)*alpha; // power right
        holes[3*i + 2] = cv::Point3f(center.x, center.y - outlet_ysize, 0.0f)*alpha; // ground
    }
}

void outlet_template_t::get_holes_2d(CvPoint2D32f* holes) const
{
    const CvPoint2D32f* centers = get_template();

    for(int i = 0; i < get_count(); i++)
    {
        CvPoint2D32f center = centers[i];
        holes[3*i] = cvPoint2D32f(center.x - outlet_xsize/2, center.y); // power left
        holes[3*i + 1] = cvPoint2D32f(center.x + outlet_xsize/2, center.y); // power right
        holes[3*i + 2] = cvPoint2D32f(center.x, center.y - outlet_ysize); // ground
    }
}

void outlet_template_t::initialize_geometric_hash()
{
    std::vector<KeyPointEx>& template_points = m_base->GetLabeledFeatures();
    geometric_matcher.addModel(template_points);
}


void writeCvPoint(CvFileStorage* fs, const char* name, CvPoint pt)
{
    cvStartWriteStruct(fs, name, CV_NODE_SEQ);
    cvWriteRawData(fs, &pt, 1, "ii");
    cvEndWriteStruct(fs);
}

void readCvPointByName(CvFileStorage* fs, CvFileNode* parent, const char* name, CvPoint& pt)
{
    CvFileNode* node = cvGetFileNodeByName(fs, parent, name);
	if (node)
	{
		cvReadRawData(fs, node, &pt, "ii");
#if defined(_SCALE_IMAGE_2)
		pt.x /= 2;
		pt.y /= 2;
#endif //_SCALE_IMAGE_2
	}
	else
	{
		pt.x = -1;
		pt.y = -1;
	}
}

void readTrainingBase(const char* config_filename, char* outlet_filename,
                      char* nonoutlet_filename, vector<feature_t>& train_features)
{
    CvMemStorage* storage = cvCreateMemStorage();

    CvFileStorage* fs = cvOpenFileStorage(config_filename, storage, CV_STORAGE_READ);

    CvFileNode* outlet_node = cvGetFileNodeByName(fs, 0, "outlet");
    const char* str = cvReadStringByName(fs, outlet_node, "outlet filename");
    strcpy(outlet_filename, str);

    CvFileNode* nonoutlet_node = cvGetFileNodeByName(fs, 0, "nonoutlet");
    str = cvReadStringByName(fs, nonoutlet_node, "nonoutlet filename");
    strcpy(nonoutlet_filename, str);

    CvPoint pt;

	int index = 1;
	char feature_name[10];
	while (1)
	{
		sprintf(feature_name, "power%d", index++);
		readCvPointByName(fs, outlet_node, feature_name, pt);
		if ((pt.x == -1)&&(pt.y==-1))
			break;
		train_features.push_back(feature_t(pt, 1, 0));
	}

	index = 1;
	while (1)
	{
		sprintf(feature_name, "ground%d", index++);
		readCvPointByName(fs, outlet_node, feature_name, pt);
		if ((pt.x == -1)&&(pt.y==-1))
			break;
		train_features.push_back(feature_t(pt, 1, 1));
	}
    cvReleaseFileStorage(&fs);

    cvReleaseMemStorage(&storage);
}
