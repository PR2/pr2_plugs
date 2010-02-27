//Created by Alexey Latyshev
// Set of functions for Generic Hough Transform (GHT)
#include <cv.h>
#include <highgui.h>
#include "outlet_pose_estimation/detail/generalized_hough.h"
#include "outlet_pose_estimation/detail/features.h"
//using namespace cv;

using namespace std;

CV_IMPL CvSparseMat*
cvCreateOutletSparseMat( int dims, const int* sizes, int type )
{
	type = CV_MAT_TYPE( type );
	int pix_size1 = CV_ELEM_SIZE1(type);
	int pix_size = pix_size1*CV_MAT_CN(type);
	int i, size;
	CvMemStorage* storage;

	if( pix_size == 0 )
		CV_Error( CV_StsUnsupportedFormat, "invalid array data type" );

	if( dims <= 0 || dims > CV_MAX_DIM_HEAP )
		CV_Error( CV_StsOutOfRange, "bad number of dimensions" );

	if( !sizes )
		CV_Error( CV_StsNullPtr, "NULL <sizes> pointer" );

	for( i = 0; i < dims; i++ )
	{
		if( sizes[i] <= 0 )
			CV_Error( CV_StsBadSize, "one of dimesion sizes is non-positive" );
	}

	CvSparseMat* arr = (CvSparseMat*)cvAlloc(sizeof(*arr)+MAX(0,dims-CV_MAX_DIM)*sizeof(arr->size[0]));

	arr->type = CV_SPARSE_MAT_MAGIC_VAL | type;
	arr->dims = dims;
	arr->refcount = 0;
	arr->hdr_refcount = 1;
	memcpy( arr->size, sizes, dims*sizeof(sizes[0]));

	arr->valoffset = (int)cvAlign(sizeof(CvSparseNode), pix_size1);
	arr->idxoffset = (int)cvAlign(arr->valoffset + pix_size, sizeof(int));
	size = (int)cvAlign(arr->idxoffset + dims*sizeof(int), sizeof(CvSetElem));


	storage = cvCreateMemStorage( 10000000 );

	arr->heap = cvCreateSet( 0, sizeof(CvSet), size, storage );

	arr->hashsize = CV_SPARSE_HASH_SIZE0;
	size = arr->hashsize*sizeof(arr->hashtable[0]);

	arr->hashtable = (void**)cvAlloc( size );
	memset( arr->hashtable, 0, size );

	return arr;
}


//Filtering outlets out of frame
void filterOutletOutliers(vector<feature_t>& features, vector<feature_t>& dst_outlet, int accuracy)
{

	//Filtering outlets outside of the window
	bool first = 0;
	bool second = 0;
	bool third = 0;
	bool forth = 0;
	if ((int)dst_outlet.size()/3 < 3)
	{
		third = 1;
		forth = 1;
	}

	int nOutlets = (int)dst_outlet.size()/3;
	for (int i=0;i<(int)dst_outlet.size();i++)
	{
		float distance = -1;
		float min_distance = (float)1e38;
		for (int j=0;j<(int)features.size();j++)
		{
			if (dst_outlet[i].class_id == features[j].class_id)
			{
				distance = (features[j].pt.x - dst_outlet[i].pt.x)*(features[j].pt.x - dst_outlet[i].pt.x)+
					(features[j].pt.y - dst_outlet[i].pt.y)*(features[j].pt.y - dst_outlet[i].pt.y);
				if (distance < min_distance)
					min_distance = distance;
			}
		}

		distance = min_distance;

		if ((distance < 1e38)&&(distance < accuracy*accuracy))
		{
			if (nOutlets == 4)
			{
				if ((i==0)||(i==1)||(i==8))
				{
					first = 1;
				}
				if ((i==2)||(i==3)||(i==9))
				{
					second = 1;
				}
				if ((i==4)||(i==5)||(i==10))
				{
					third = 1;
				}
				if ((i==6)||(i==7)||(i==11))
				{
					forth = 1;
				}
			}
			if (nOutlets == 2)
			{
				if ((i==0)||(i==1)||(i==4))
				{
					first = 1;
				}
				if ((i==2)||(i==3)||(i==5))
				{
					second = 1;
				}

			}

		}

	}
	if (!(first && second && third && forth))
	{
		dst_outlet.clear();
		return;
	}
	//if ((!first && !second)|| (!third && !forth)||(!first && !third)|| (!first && !forth)||(!second && !forth)||(!second && !third))
	//{
	//	dst_outlet.clear();
	//}
	//Filtering by outlet angle
	else
	{
		float alpha = (float)CV_PI;
		float beta = (float)CV_PI;
		float gamma = (float)CV_PI;
		float a,b,c;

		int overSize = (int)dst_outlet.size()%3;
		for (int i=0;i<nOutlets;i++)
		{
			a = (dst_outlet[2*i].pt.x - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.x)*(dst_outlet[2*i].pt.x - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.x)+
				(dst_outlet[2*i].pt.y - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.y)*(dst_outlet[2*i].pt.y - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.y);
			b = (dst_outlet[2*i+1].pt.x - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.x)*(dst_outlet[2*i+1].pt.x - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.x)+
				(dst_outlet[2*i+1].pt.y - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.y)*(dst_outlet[2*i+1].pt.y - dst_outlet[(int)dst_outlet.size() - overSize - nOutlets+i].pt.y);
			c = (dst_outlet[2*i].pt.x - dst_outlet[2*i+1].pt.x)*(dst_outlet[2*i].pt.x - dst_outlet[2*i+1].pt.x)+
				(dst_outlet[2*i].pt.y - dst_outlet[2*i+1].pt.y)*(dst_outlet[2*i].pt.y - dst_outlet[2*i+1].pt.y);

			if ((a > 0)&&( b> 0))
				alpha = acos((a + b - c)/2/sqrt(a*b));
			if ((a > 0)&&( b> 0))
				beta = acos((a + c - b)/2/sqrt(a*c));
			gamma = (float)CV_PI - alpha - beta;


			if ((alpha > CV_PI/2)||(beta > CV_PI/2)||(gamma > CV_PI/2))
			{
				dst_outlet.clear();
				return;
			}
		}

	}
	//End of filtering by angles

	// Filtering regions "full-of-features"
#if 1
	CvPoint min, max;
	float coeff = 2.0f;
	min.x = (int)dst_outlet[0].pt.x;
	min.y = (int)dst_outlet[0].pt.y;
	max.x = min.x;
	max.y = min.y;
	for (int i=1; i<(int)dst_outlet.size(); i++)
	{
		if ((int)dst_outlet[i].pt.x > max.x)
			max.x = (int)dst_outlet[i].pt.x ;
		if ((int)dst_outlet[i].pt.y > max.y)
			max.y = (int)dst_outlet[i].pt.y ;
		if ((int)dst_outlet[i].pt.x < min.x)
			min.x = (int)dst_outlet[i].pt.x ;
		if ((int)dst_outlet[i].pt.y < min.y)
			min.y = (int)dst_outlet[i].pt.y ;
	}
	int width = max.x - min.x;
	int height = max.y - min.y;
	min.x -=(int)(width*0.2);
	max.x +=(int)(width*0.2);
	min.y -=(int)(height*0.2);
	max.y +=(int)(height*0.2);
	//Calculating number of features inside region "min x max"
	int count = 0;
	for (int i=0;i<(int)features.size(); i++)
	{
		if (((int)features[i].pt.x >= min.x)&&((int)features[i].pt.y >= min.y)&&
			((int)features[i].pt.x <= max.x)&&((int)features[i].pt.y <= max.y))
		{
			count++;
		}

	}
	//printf("Features: %d\nCount: %d\n\n",(int)features.size(),count);
	if (count > (int)(coeff*dst_outlet.size()))
	{
		dst_outlet.clear();
		return;
	}
#endif

	//End of filtering

	//End of filtering
}
// Calculates outlet's center from given feature and affine transform
CvPoint* getOutletCenter(feature_t feature, const vector<feature_t>& train_features, int feature_id, float angle1, float x_scale, float y_scale, float angle2)
{
	CvPoint* result;
	int train_length = (int)train_features.size();
	if ((feature_id < 0)||(feature_id > (train_length-1)))
		return NULL;
	CvPoint outlet_center;
	outlet_center.x = 0;
	outlet_center.y = 0;
	for (int i=0;i<train_length;i++)
	{
		outlet_center.x += (int)train_features[i].pt.x;
		outlet_center.y += (int)train_features[i].pt.y;
	}
	outlet_center.x /= train_length;
	outlet_center.y /= train_length;

	float rel_center_x = - outlet_center.x + train_features[feature_id].pt.x;
	float rel_center_y = - outlet_center.y + train_features[feature_id].pt.y;
	float t1 = rel_center_x * cos(angle1) + rel_center_y*sin(angle1);
	float t2 = - rel_center_x * sin(angle1) + rel_center_y * cos(angle1);
	rel_center_x = t1*x_scale;
	rel_center_y = t2*y_scale;
	t1 = rel_center_x * cos(angle2) + rel_center_y*sin(angle2);
	t2 = - rel_center_x * sin(angle2) + rel_center_y * cos(angle2);

	result = new CvPoint();
	result->x = (int)(feature.pt.x-t1);
	result->y = (int)(feature.pt.y-t2);

	return result;

}
// Builds 6-dimension histogram [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
CvSparseMat* buildHoughHist(vector<feature_t>& input, const vector<feature_t>& train_features, int* hist_size, float** ranges)
{
	CvSparseMat* hist;

	hist = cvCreateOutletSparseMat(6,hist_size,CV_32FC1);

	int* hist_size2 = new int[7];
	for (int i=0;i<6;i++)
		hist_size2[i] = hist_size[i];
	hist_size2[6] = (int)train_features.size();

	CvSparseMat* feature_votes = cvCreateOutletSparseMat(7,hist_size2,CV_8UC1);


	int* idx = new int[6];
	int* votes_idx = new int[7];


	float blur_coeff = 0.17f;

	for (int n = 0; n < (int)input.size();n++)
	{
		for (int feature_id = 0; feature_id < (int)train_features.size(); feature_id++)
		{
			if (input[n].class_id != train_features[feature_id].class_id)
				continue;

			for (float angle1 = ranges[2][0]+(ranges[2][1]-ranges[2][0])/(hist_size[2]*2); angle1 <= ranges[2][1]; angle1 += ((ranges[2][1]-ranges[2][0] > 0) ? (ranges[2][1]-ranges[2][0])/hist_size[2] : 1))
			{
				for (float x_scale = ranges[3][0]+(ranges[3][1]-ranges[3][0])/(hist_size[3]*2); x_scale <= ranges[3][1]; x_scale += ((ranges[3][1]-ranges[3][0] > 0) ? (ranges[3][1]-ranges[3][0])/hist_size[3] : 1))
				{
					for (float y_scale = ranges[4][0]+(ranges[4][1]-ranges[4][0])/(hist_size[4]*2); y_scale <= ranges[4][1]; y_scale += ((ranges[4][1]-ranges[4][0] > 0) ? (ranges[4][1]-ranges[4][0])/hist_size[4] : 1))
					{
						for (float angle2 = ranges[5][0]+(ranges[5][1]-ranges[5][0])/(hist_size[5]*2); angle2 <= ranges[5][1]; angle2 += ((ranges[5][1]-ranges[5][0] > 0) ? (ranges[5][1]-ranges[5][0])/hist_size[5] : 1))
						{

                            if(abs(input[n].pt.x - 379) < 5 && abs(input[n].pt.y - 312) < 5 &&
                               fabs(fabs(angle1)) < 0.2 && fabs(fabs(angle2) - 3.1415/3) < 0.2)
                            {
                                int w = 1;
                            }

							CvPoint* center = getOutletCenter(input[n],train_features,feature_id,angle1,x_scale,y_scale,angle2);

							if (center && (center->x >= ranges[0][0]) && (center->x < ranges[0][1]) && (center->y >= ranges[1][0]) && (center->y < ranges[1][1]))
							{
								//Incrementing histogram
								float idx0 = ((center->x - ranges[0][0])/(ranges[0][1]-ranges[0][0]) * hist_size[0]);
								float idx1 = ((center->y - ranges[1][0])/(ranges[1][1]-ranges[1][0]) * hist_size[1]);
								idx[0] = (int)idx0;
								idx[1] = (int)idx1;
								idx[2] = (ranges[2][1] != ranges[2][0]) ? (int)((angle1 - ranges[2][0])/(ranges[2][1]-ranges[2][0]) * hist_size[2]) : 0;
								idx[3] = (ranges[3][1] != ranges[3][0]) ?(int)((x_scale - ranges[3][0])/(ranges[3][1]-ranges[3][0]) * hist_size[3]) : 0;
								idx[4] = (ranges[4][1] != ranges[4][0]) ?(int)((y_scale - ranges[4][0])/(ranges[4][1]-ranges[4][0]) * hist_size[4]) : 0;
								idx[5] = (ranges[5][1] != ranges[5][0]) ?(int)((angle2 - ranges[5][0])/(ranges[5][1]-ranges[5][0]) * hist_size[5]) : 0;


								bool isOK = true;
								for (int i=0;i<2; i++)
								{
									if (idx[i] >= hist_size[i])
									{
										idx[i] = hist_size[i]-1;
										isOK = false;
									}
									if (idx[i] < 0)
									{
										idx[i] = 0;
										isOK = false;
									}
								}

								if (isOK)
								{
									for (int i=0;i<6;i++)
										votes_idx[i] = idx[i];
									votes_idx[6] = feature_id;

									int val = (int)cvGetND(feature_votes,votes_idx).val[0];

									if (val)
									{
										isOK = false;
									}
									else
									{
										cvSetND(feature_votes,votes_idx,cvScalar(1));
									}
								}

								if (isOK)
								{

									float value = (float)cvGetRealND(hist,idx);
									cvSetRealND(hist,idx,++value);

									//// Fast blur
									//int idx2[6];
									//for (int i=0;i<6;i++)
									//	idx2[i]=idx[i];

									//int move_x = 0;
									//int move_y = 0;


									//if (((idx0 - idx[0]) < blur_coeff)&&(idx[0] > 0))
									//	move_x=-1;
									//else
									//	if (((-idx0 + idx[0]+1) < blur_coeff)&&(idx[0] <(hist_size[0]-1)))
									//		move_x=1;
									//if (((idx1 - idx[1]) < blur_coeff)&&(idx[1] > 0))
									//	move_y=-1;
									//else
									//	if (((-idx1 + idx[1]+1) < blur_coeff)&&(idx[1] < (hist_size[1]-1)))
									//		move_y=1;

									//idx2[0] = idx[0]+move_x;
									//idx2[1] = idx[1]+move_y;

									//if ((move_x != 0) || (move_y !=0))
									//{
									//	float value2 = (float)cvGetRealND(hist,idx2);
									//	//if (value2 > value)
									//		cvSetRealND(hist,idx2,++value2);
									//	//else
									//	//	cvSetRealND(hist,idx,++value);
									//}

									////End

									//Bins Blur (x & y)
									//int idx2[6];
									//for (int i=0;i<6;i++)
									//	idx2[i]=idx[i];
									//for (int x = -1;x<=1;x++)
									//	for (int y= -1;y<=1;y++)
									//	{
									//		if (((idx[0]+x) >=0)&&((idx[1]+y) >=0)&&((idx[0]+x) < hist_size[0])&&((idx[1]+y) < hist_size[1]))
									//		{
									//			idx2[0] = idx[0]+x;
									//			idx2[1] = idx[1]+y;
									//			float value = cvGetRealND(hist,idx2);
									//			cvSetRealND(hist,idx2,++value);
									//		}
									//	}
									//End


								}


								delete center;
								center = 0;
							}
							if (center)
								delete center;
						}
					}
				}
			}
		}
	}
	delete[] idx;
	delete[] votes_idx;
	cvReleaseSparseMat(&feature_votes);

	return hist;
}
//---------
// Calculates maximums of histogram.
float** getMaxHistValues(const CvHistogram* hist, int* hist_size)
{
	float** values = new float*[1];
	*values = new float[6];

	float max_val, min_val;
	int* idx = new int[6];
	cvGetMinMaxHistValue(hist,&min_val,&max_val,0,idx);
	printf("\nVotes: %f\n ",max_val);

	(*values)[0] = hist->thresh[0][0]+(hist->thresh[0][1]-hist->thresh[0][0])/hist_size[0]*idx[0];
	(*values)[1] = hist->thresh[1][0]+(hist->thresh[1][1]-hist->thresh[1][0])/hist_size[1]*idx[1];
	(*values)[2] = hist->thresh[2][0]+(hist->thresh[2][1]-hist->thresh[2][0])/hist_size[2]*idx[2];
	(*values)[3] = hist->thresh[3][0]+(hist->thresh[3][1]-hist->thresh[3][0])/hist_size[3]*idx[3];
	(*values)[4] = hist->thresh[4][0]+(hist->thresh[4][1]-hist->thresh[4][0])/hist_size[4]*idx[4];
	(*values)[5] = hist->thresh[5][0]+(hist->thresh[5][1]-hist->thresh[5][0])/hist_size[5]*idx[5];

	delete[] idx;
	return values;
}
//---------
//---------
// Calculates maximums of histogram.
void getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float** &maxs, int& count, int MIN_VOTES)
{
	count = 0;
	//int MIN_VOTES = 4;
	CvSparseMatIterator mat_iterator;
	CvSparseNode* node = cvInitSparseMatIterator( hist, &mat_iterator );

	for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
	{
		//  const int* idx = CV_NODE_IDX( (CvSparseMat*)hist->bins, node ); /* get pointer to the element indices */
		float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
														(assume that the type is CV_32FC1) */
		if (val >= MIN_VOTES)
			(count) ++;
	}
	//Endof

	if (count > 0)
	{
		maxs = new float*[count];
		for (int i=0; i<(count); i++)
			maxs[i] = new float[6];

		int i=0;
		node = cvInitSparseMatIterator( hist, &mat_iterator );
		for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
		{
			int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
			float val = *(float*)CV_NODE_VAL( (CvSparseMat*)hist, node ); /* get value of the element
																		  (assume that the type is CV_32FC1) */
			if (val >= MIN_VOTES)
			{
				maxs[i][0] = (float)(ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx[0]+0.5));
				maxs[i][1] = (float)(ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx[1]+0.5));
				maxs[i][2] = (float)(ranges[2][0]+(ranges[2][1]-ranges[2][0])/hist_size[2]*(idx[2]+0.5));
				maxs[i][3] = (float)(ranges[3][0]+(ranges[3][1]-ranges[3][0])/hist_size[3]*(idx[3]+0.5));
				maxs[i][4] = (float)(ranges[4][0]+(ranges[4][1]-ranges[4][0])/hist_size[4]*(idx[4]+0.5));
				maxs[i][5] = (float)(ranges[5][0]+(ranges[5][1]-ranges[5][0])/hist_size[5]*(idx[5]+0.5));
				i++;
			}

			//delete[] idx;
		}

	}
	else
	{
		maxs = NULL;
		count = 0;
	}

}
//---------
int getMaxHistValues(const CvSparseMat* hist, int* hist_size, float** ranges, float** &maxs, int& count)
{
	//IplImage* img = cvCreateImage(cvSize(640,480),8,3);
	count = 0;
	float MIN_VOTES = 0;
	CvSparseMatIterator mat_iterator;
	CvSparseNode* node = cvInitSparseMatIterator( hist, &mat_iterator );

	for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
	{
		//  const int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
		float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
														(assume that the type is CV_32FC1) */
		//if (val > 0)
		//{
		//	//CvPoint pt;
		//	int* idx = CV_NODE_IDX( hist, node );
		//	int x = ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx[0]+0.5);
		//	int y = ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx[1]+0.5);
		//	int blue = ((uchar*)(img->imageData + img->widthStep*y))[x*3];
		//	int green = ((uchar*)(img->imageData + img->widthStep*y))[x*3+1];
		//	int red = ((uchar*)(img->imageData + img->widthStep*y))[x*3+2];
		//	int v = blue+green+red;

		//	v+=val;
		//	blue = ((v / 255 > 0) ? 255 : v % 255);

		//	green = ((v / 510 > 0) ? 255 : ((blue == 255) ? v % 510 - 255 : 0));

		//	red = ((green == 255) ? v % 765 - 510 : 0);

		//	//((uchar*)(img->imageData + img->widthStep*y))[x*3] = blue;
		//	//((uchar*)(img->imageData + img->widthStep*y))[x*3+1] = 100;
		//	//((uchar*)(img->imageData + img->widthStep*y))[x*3+2] = red;
		//

		//	//CvScalar color; color.val[0] = img->imageData[pt.y*img->width + pt.x];
		//	//color.val[0] +=5;
		//	// img->imageData[pt.y*img->width + pt.x]+=5;
		//	cvDrawRect(img,cvPoint(x-2,y-2),cvPoint(x+2,y+2),cvScalar(blue,green,red),CV_FILLED);

		//}
		if (val > MIN_VOTES)
			MIN_VOTES = val-1;
	}
	//cvSaveImage("1.bmp",img);

	node = cvInitSparseMatIterator( hist, &mat_iterator );

	for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
	{
		//  const int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
		float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
														(assume that the type is CV_32FC1) */
		if (val >= MIN_VOTES)
			(count) ++;
	}
	//Endof

	if (count > 0)
	{
		maxs = new float*[count];
		for (int i=0; i<(count); i++)
			maxs[i] = new float[6];

		int i=0;
		node = cvInitSparseMatIterator( hist, &mat_iterator );
		for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
		{
			int* idx = CV_NODE_IDX( hist, node ); /* get pointer to the element indices */
			float val = *(float*)CV_NODE_VAL( hist, node ); /* get value of the element
															(assume that the type is CV_32FC1) */
			if (val >= MIN_VOTES)
			{
				maxs[i][0] = (float)(ranges[0][0]+(ranges[0][1]-ranges[0][0])/hist_size[0]*(idx[0]+0.5));
				maxs[i][1] = (float)(ranges[1][0]+(ranges[1][1]-ranges[1][0])/hist_size[1]*(idx[1]+0.5));
				maxs[i][2] = (float)(ranges[2][0]+(ranges[2][1]-ranges[2][0])/hist_size[2]*(idx[2]+0.5));
				maxs[i][3] = (float)(ranges[3][0]+(ranges[3][1]-ranges[3][0])/hist_size[3]*(idx[3]+0.5));
				maxs[i][4] = (float)(ranges[4][0]+(ranges[4][1]-ranges[4][0])/hist_size[4]*(idx[4]+0.5));
				maxs[i][5] = (float)(ranges[5][0]+(ranges[5][1]-ranges[5][0])/hist_size[5]*(idx[5]+0.5));
				i++;
			}

			//delete[] idx;
		}

	}
	else
	{
		maxs = NULL;
		count = 0;
	}

	int res = (int)MIN_VOTES;
	return res;

}


// Calculates outlet features from given train outlet and affine transform
// Affine transform is array [center x, center y, rotation angle1, x scale, y scale, rotation angle 2]
void calcOutletPosition(const vector<feature_t>& train_features, float* affine_transform, vector<feature_t>& features)
{
	CvPoint center = cvPoint((int)(affine_transform[0]),(int)(affine_transform[1]));
	float angle1 = affine_transform[2];
	float x_scale = affine_transform[3];
	float y_scale = affine_transform[4];
	float angle2 = affine_transform[5];
	int train_length = (int)train_features.size();
	//CvPoint* result = new CvPoint[train_length];

	CvPoint outlet_center;
	outlet_center.x = 0;
	outlet_center.y = 0;
	for (int i=0;i<train_length;i++)
	{
		outlet_center.x += (int)train_features[i].pt.x;
		outlet_center.y += (int)train_features[i].pt.y;
	}
	outlet_center.x /= train_length;
	outlet_center.y /= train_length;

	for (int i=0; i< train_length; i++)
	{
		float rel_center_x = - outlet_center.x + train_features[i].pt.x;
		float rel_center_y = - outlet_center.y + train_features[i].pt.y;
		float t1 = rel_center_x * cos(angle1) + rel_center_y*sin(angle1);
		float t2 = - rel_center_x * sin(angle1) + rel_center_y * cos(angle1);
		rel_center_x = t1*x_scale;
		rel_center_y = t2*y_scale;
		t1 = rel_center_x * cos(angle2) + rel_center_y*sin(angle2);
		t2 = - rel_center_x * sin(angle2) + rel_center_y * cos(angle2);

		CvPoint result_point;
		result_point.x = (int)(center.x+t1);
		result_point.y = (int)(center.y+t2);
		feature_t feature;
		feature.size = train_features[i].size;
		feature.pt = result_point;
		feature.class_id = train_features[i].class_id;

		features.push_back(feature);
	}


}
//------
//Use false movements filter
// dst_outlet is outlet after moving some outlet holes to the new places
// projected_outlet is outlet after affine transform applying to the template outlet
//The algorithm checks should we apply the movement or return to the original position
void filterFalseMovements(const vector<feature_t>& projected_outlet, vector<feature_t>& dst_outlet)
{
	// Filtering false movements
	vector <float> orig_right;
	vector <float> orig_left;
	vector <float> new_right;
	vector <float> new_left;
	float dist = 0.0f;
	float diff_coeff = 1.6f;
	float diff_coeff_1 = 1/diff_coeff;
	int nPower = (int)dst_outlet.size()/3*2;
	int nGround = nPower/2;
	orig_right.resize(nPower+nGround);
	orig_left.resize(nPower+nGround);
	new_right.resize(nPower+nGround);
	new_left.resize(nPower+nGround);
	for (int i=0;i<nGround;i++)
	{
		dist = (dst_outlet[nPower+i].pt.x-dst_outlet[2*i].pt.x)*(dst_outlet[nPower+i].pt.x-dst_outlet[2*i].pt.x)+
			(dst_outlet[nPower+i].pt.y-dst_outlet[2*i].pt.y)*(dst_outlet[nPower+i].pt.y-dst_outlet[2*i].pt.y);
		new_right[nPower+i] = dist;
		new_left[2*i] = dist;

		dist = (dst_outlet[nPower+i].pt.x-dst_outlet[2*i+1].pt.x)*(dst_outlet[nPower+i].pt.x-dst_outlet[2*i+1].pt.x)+
			(dst_outlet[nPower+i].pt.y-dst_outlet[2*i+1].pt.y)*(dst_outlet[nPower+i].pt.y-dst_outlet[2*i+1].pt.y);
		new_left[nPower+i] = dist;
		new_right[2*i+1] = dist;

		dist = (dst_outlet[2*i+1].pt.x-dst_outlet[2*i].pt.x)*(dst_outlet[2*i+1].pt.x-dst_outlet[2*i].pt.x)+
			(dst_outlet[2*i+1].pt.y-dst_outlet[2*i].pt.y)*(dst_outlet[2*i+1].pt.y-dst_outlet[2*i].pt.y);
		new_right[2*i] = dist;
		new_left[2*i+1] = dist;

		dist = (projected_outlet[nPower+i].pt.x-projected_outlet[2*i].pt.x)*(projected_outlet[nPower+i].pt.x-projected_outlet[2*i].pt.x)+
			(projected_outlet[nPower+i].pt.y-projected_outlet[2*i].pt.y)*(projected_outlet[nPower+i].pt.y-projected_outlet[2*i].pt.y);
		orig_right[nPower+i] = dist;
		orig_left[2*i] = dist;

		dist = (projected_outlet[nPower+i].pt.x-projected_outlet[2*i+1].pt.x)*(projected_outlet[nPower+i].pt.x-projected_outlet[2*i+1].pt.x)+
			(projected_outlet[nPower+i].pt.y-projected_outlet[2*i+1].pt.y)*(projected_outlet[nPower+i].pt.y-projected_outlet[2*i+1].pt.y);
		orig_left[nPower+i] = dist;
		orig_right[2*i+1] = dist;

		dist = (projected_outlet[2*i+1].pt.x-projected_outlet[2*i].pt.x)*(projected_outlet[2*i+1].pt.x-projected_outlet[2*i].pt.x)+
			(projected_outlet[2*i+1].pt.y-projected_outlet[2*i].pt.y)*(projected_outlet[2*i+1].pt.y-projected_outlet[2*i].pt.y);
		orig_right[2*i] = dist;
		orig_left[2*i+1] = dist;
	}

	for (int i=0;i<nPower+nGround;i++)
	{
		if (new_left[i] > 0)
		{
			float rel = orig_left[i]/new_left[i];
			if ((rel > diff_coeff)||(rel  <diff_coeff_1))
			{
				dst_outlet[i].pt = projected_outlet[i].pt;
				continue;

			}
		}

		if (new_right[i] > 0)
		{
			float rel = orig_right[i]/new_right[i];
			if ((rel > diff_coeff)||(rel  <diff_coeff_1))
			{
				dst_outlet[i].pt = projected_outlet[i].pt;
				continue;
			}
		}
	}

	///*for (int i=nPower;i<nPower+nGround;i++)
	//{
	//	bool useProjected = false;
	//	if (new_left[i] > 0)
	//	{
	//		float rel = orig_left[i]/new_left[i];
	//		if ((rel > diff_coeff)||(rel  <diff_coeff_1))
	//		{
	//			useProjected = true;
	//		}
	//	}
	//	if (new_right[i] > 0)
	//	{
	//		float rel = orig_right[i]/new_right[i];
	//		if ((rel > diff_coeff)||(rel  <diff_coeff_1))
	//		{
	//			useProjected = true;
	//		}
	//	}
	//	if (new_right[2*(i-nPower)] > 0)
	//	{
	//		float rel = orig_right[2*(i-nPower)]/new_right[2*(i-nPower)];
	//		if ((rel > diff_coeff)||(rel  <diff_coeff_1))
	//		{
	//			useProjected = true;
	//		}
	//	}

	//	if (useProjected)
	//	{
	//		dst_outlet[i] = projected_outlet[i];
	//		dst_outlet[2*(i-nPower)] = projected_outlet[2*(i-nPower)];
	//		dst_outlet[2*(i-nPower)+1] = projected_outlet[2*(i-nPower)+1];
	//	}
	//}*/

}
//------
//for each point of src_outlet the closest point of features vector founds (not far than accuracy distznce)
//index of the closest features puts into index array (its size MUST BE equal to src_outlet.size())
//index[i] == -1 means unable to choose the closest feature
//max_diff_coeff - coefficient is used for two distances comparision. If relation betweeen distances to the two nearest features less than sqrt(max_diff_coeff) we unable to choose one of them
void getNearestFeaturesIndexes(const vector<feature_t>& src_outlet, const vector<feature_t>& features, int* indexes, int accuracy, float max_diff_coeff)
{
		//float max_diff_coeff = 2.0f;
	// Trying to find the nearest features for src_outlet holes
	for (int i=0;i<(int)src_outlet.size();i++)
	{
		int min_index = -1;
		float min_distance = (float)1e30;
		float last_min_distance;
		for (int j=0;j<(int)features.size();j++)
		{
			if (features[j].class_id == src_outlet[i].class_id)
			{
				float distance = (features[j].pt.x - src_outlet[i].pt.x)*(features[j].pt.x - src_outlet[i].pt.x)+
					(features[j].pt.y - src_outlet[i].pt.y)*(features[j].pt.y - src_outlet[i].pt.y);
				if ((distance < min_distance)/* && (distance < accuracy*accuracy)*/)
				{
					last_min_distance = min_distance;
					min_distance = distance;
					min_index = j;
				}
				else
				{
					//Add points comparing
					if ((distance < last_min_distance)&&((features[j].pt.x != features[min_index].pt.x)||(features[j].pt.y != features[min_index].pt.y)))
					{
						last_min_distance = distance;
					}
				}
			}

		}
		if (min_distance < accuracy*accuracy)
			indexes[i] = min_index;
		else
			min_index = -1;
		if (min_index > -1)
		{
			if ((min_distance > 0) && (last_min_distance/min_distance <= max_diff_coeff))
			{
				indexes[i] = -1;
				//printf("---->Unable to choose feature\n");
			}
		}
	}
	// Removing the same indexes
	for (int i=0;i< (int)(src_outlet.size());i++)
	{
		if (indexes[i] >=0)
		{
			bool wasRepeat = false;
			for (int j=i+1;j< (int)(src_outlet.size());j++)
			{
				if (indexes[i]==indexes[j])
				{
					indexes[j] = -1;
					wasRepeat = true;
				}
			}
			if (wasRepeat)
				indexes[i] = -1;
		}
	}
}
//------
//for each point of dst_outlet the closest point of features vector founds (not far than distance between two power holes / 3)
//index of the closest feature is in the index array (its size MUST BE equal to dst_outlet.size())
//index[i] == -1 means unable to choose the closest feature
//max_diff_coeff - coefficient is used for two distances comparision. If relation betweeen distances to the two nearest features less than sqrt(max_diff_coeff) we unable to choose one of them
// if we are able to choose the closest feature we believe it is real outlet feature
void attractOutletToFeatures(const vector<feature_t>& train_features, const vector<feature_t>& features,vector<feature_t>& dst_outlet, const int* indexes, float max_diff_coeff)
{
		for (int i=0;i< (int)(dst_outlet.size());i++)
		{
			// The second attraction
			//Temp
			int min_index = -1;
			float min_distance = (float)1e38;
			float last_min_distance;
			for (int j=0;j<(int)features.size();j++)
			{
				if (features[j].class_id == dst_outlet[i].class_id)
				{
					float distance = (features[j].pt.x - dst_outlet[i].pt.x)*(features[j].pt.x - dst_outlet[i].pt.x)+
						(features[j].pt.y - dst_outlet[i].pt.y)*(features[j].pt.y - dst_outlet[i].pt.y);
					if (distance < min_distance)
					{
						last_min_distance = distance;
						//setting min distance to power holes distance / 3
						float acc = (float)((train_features[1].pt.x - train_features[0].pt.x)*(train_features[1].pt.x - train_features[0].pt.x)+
							(train_features[1].pt.y - train_features[0].pt.y)*(train_features[1].pt.y - train_features[0].pt.y))/9;
						if (distance < acc)
						{
							min_distance = distance;
							min_index = j;
						}
					}
					else
					{
						if (distance < last_min_distance)
						{
							last_min_distance = distance;
						}
					}
				}
			}
			if (min_index >= 0)
			{
				if (((min_distance > 0) && (last_min_distance/min_distance <= max_diff_coeff))||(min_distance == 0))
					dst_outlet[i] = features[min_index];

			}
			else

				//End

				if (indexes[i] >=0)
				{
					dst_outlet[i] = features[indexes[i]];
				}
		}
}

//------
void calcExactLocation(vector<feature_t>& features,const vector<feature_t>& train_features, vector<feature_t>& src_outlet,
                       vector<feature_t>& dst_outlet, float& reprojectionError, int accuracy, bool useSecondAttraction)
{
	float max_diff_coeff = 2.0f;
	if (((int)train_features.size()) == ((int)src_outlet.size()))
	{
		vector<CvPoint> train_points;
		train_points.clear();

		//vector<CvPoint> src_outlet_points;
		vector<CvPoint> features_points;
		features_points.clear();
		int* indexes = new int[(int)train_features.size()];


		for (int i=0;i<(int)train_features.size();i++)
		{
			indexes[i] = -1;
		}
		//for (int i=0;i<(int)src_outlet.size();i++)
		//{
		//	src_outlet_points.push_back(src_outlet[i].center);
		//}


		getNearestFeaturesIndexes(src_outlet,features, indexes, accuracy, max_diff_coeff);


		for (int i=0;i< (int)(src_outlet.size());i++)
		{
			if (indexes[i] >=0)
			{
				train_points.push_back(train_features[i].pt);
				features_points.push_back(features[indexes[i]].pt);
			}
		}
		int nPoints = (int)train_points.size();
		//The nearest features were found

		if (((int)train_points.size() > 3) /*&& ((int)train_points.size() > ((int)train_features.size()/2))*/)
		{
			//Projecting the template outlet to the image
			CvMat* homography = cvCreateMat(2, 3, CV_32FC1);
			FindAffineTransform(train_points, features_points, homography);
			reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography) + 1000000 - 10000*(int)train_points.size();
			dst_outlet.clear();
			MapFeaturesAffine(train_features, dst_outlet, homography);

			vector<feature_t> projected_outlet = dst_outlet;

			// Trying to find the nearest feaatures for dst_outlet holes and change holes position to features positions
			if (useSecondAttraction)
			{
				attractOutletToFeatures(train_features,features,dst_outlet,indexes,max_diff_coeff);
			}


			////// Other calculation of reproj. error
			//train_points.clear();
			//features_points.clear();
			//for (int i=0;i< (int)(src_outlet.size());i++)
			//{
			//	if (indexes[i] >=0)
			//	{
			//		train_points.push_back(temp[i].pt);
			//		features_points.push_back(dst_outlet[i].pt);
			//	}

			//}
			//float reprojectionError_new =	CalcAffineReprojectionError(train_points, features_points, homography) + 1000000 - 10000*(int)train_points.size();
			////reprojectionError =	CalcAffineReprojectionError(train_points, features_points, homography)/* + 1000*(int)train_points.size()*/;
			////// end
			//if (reprojectionError_new > reprojectionError)
			//{
			//	dst_outlet = temp;
			//	printf("old\n");
			//}
			//else
			//{
			//	reprojectionError = reprojectionError_new;
			//	printf("new\n");
			//}


#if 1
			filterFalseMovements(projected_outlet, dst_outlet);
#endif

			cvReleaseMat(&homography);

		}
		else
		{
			dst_outlet.clear();
			reprojectionError = (float)1e38;
		}


		delete[] indexes;
	}
	else
	{
		dst_outlet.clear();
		reprojectionError = (float)1e38;
	}


}
//----------

void convertFeaturesToOutlet(const vector<feature_t>& res_features, vector<outlet_t>& holes, IplImage* resImage)
{
	holes.clear();
	outlet_t outlet;

	for (int i=0;i<(int)res_features.size()/3;i++)
	{
		outlet.hole1 = res_features[2*i].pt;
		outlet.hole2 = res_features[2*i+1].pt;
		outlet.ground_hole = res_features[(int)res_features.size()/3*2+i].pt;
		holes.push_back(outlet);
		if (resImage)
		{
			CvScalar pointColor = cvScalar(255,0,50);
			cvLine(resImage, cvPoint((int)(outlet.ground_hole.x+7), (int)(outlet.ground_hole.y)), cvPoint((int)(outlet.ground_hole.x-7),(int)(outlet.ground_hole.y)),pointColor,2);
			cvLine(resImage, cvPoint((int)(outlet.ground_hole.x), (int)(outlet.ground_hole.y+7)), cvPoint((int)(outlet.ground_hole.x), (int)(outlet.ground_hole.y-7)),pointColor,2);
			pointColor = cvScalar(0,255,50);
			cvLine(resImage, cvPoint((int)(outlet.hole1.x+7), (int)(outlet.hole1.y)), cvPoint((int)(outlet.hole1.x-7),(int)(outlet.hole1.y)),pointColor,2);
			cvLine(resImage, cvPoint((int)(outlet.hole1.x), (int)(outlet.hole1.y+7)), cvPoint((int)(outlet.hole1.x), (int)(outlet.hole1.y-7)),pointColor,2);
			cvLine(resImage, cvPoint((int)(outlet.hole2.x+7), (int)(outlet.hole2.y)), cvPoint((int)(outlet.hole2.x-7),(int)(outlet.hole2.y)),pointColor,2);
			cvLine(resImage, cvPoint((int)(outlet.hole2.x), (int)(outlet.hole2.y+7)), cvPoint((int)(outlet.hole2.x), (int)(outlet.hole2.y-7)),pointColor,2);
		}
	}
}

void convertFeaturesToOutlet(const vector<feature_t>& res_features, const vector<bool>& is_detected, vector<outlet_t>& holes)
{
	holes.clear();
	outlet_t outlet;

	for (int i=0;i<(int)res_features.size()/3;i++)
	{
		outlet.hole1 = res_features[2*i].pt;
		outlet.hole1_detected = is_detected[2*i];

		outlet.hole2 = res_features[2*i+1].pt;
		outlet.hole2_detected = is_detected[2*i+1];

		outlet.ground_hole = res_features[(size_t)res_features.size()/3*2+i].pt;
		outlet.ground_hole_detected = is_detected[(size_t)res_features.size()/3*2+i];

		holes.push_back(outlet);
	}
}
//----------
//repainting features (hole candidates)
//features are closed to the ground hole will be ground hole candidates
//the same is for power holes
void repaintFeatures(const vector<feature_t> hole_candidates, const vector<feature_t>& hole_features, vector<feature_t>& hole_candidates_repainted, int accuracy = 10)
{
	float dist_coeff = (float)0.015;
	for(int i = 0; i < (int)hole_candidates.size(); i++)
	{
		float min_dist = 1e10;
		int min_idx = -1;
		for (int j=0; j<(int)hole_features.size();j++)
		{
			float d = (hole_features[j].pt.x - hole_candidates[i].pt.x)*(hole_features[j].pt.x - hole_candidates[i].pt.x) +
				(hole_features[j].pt.y - hole_candidates[i].pt.y)*(hole_features[j].pt.y - hole_candidates[i].pt.y);
			if ((d < accuracy*accuracy*dist_coeff)&&(d<min_dist))
			{
				min_idx = j;
				min_dist = d;
			}
		}
		hole_candidates_repainted.push_back(hole_candidates[i]);
		if (min_idx>=0)
			hole_candidates_repainted[i].class_id = hole_features[min_idx].class_id;
	}
}
//----------
float generalizedHoughTransform(vector<feature_t>& hole_candidates, const vector<feature_t>& train_features, int* hist_size, float** ranges,vector<outlet_t>& holes, IplImage* ghtImage, IplImage* resImage, char* output_path, char* base_filename)
{
    const float max_error = (float)1e38;
    const int min_votes = 3;

	IplImage* ght = 0;
	if (ghtImage)
	{
		ght = cvCloneImage(ghtImage);
	}
	CvSparseMat* hist = buildHoughHist(hole_candidates,train_features,hist_size,ranges);
	float** values = new float*[1];// = getMaxHistValues(hist,hist_size);
	int count = 1;
	int votes = 0;
	votes = getMaxHistValues(hist,hist_size,ranges,values,count);
#if defined(_VERBOSE)
	printf("Votes: %d\n",votes);
#endif
	cvReleaseSparseMat(&hist);

    if(votes < min_votes)
    {
        cvReleaseImage(&ght);
        return max_error;
    }

	//// Second GHT
	//	x_size = test_image->width/11;
	//	y_size = test_image->height/11;
	//	int hist_size2[] = {x_size, y_size, angle1_size, x_scale_size, y_scale_size, angle2_size};
	//	hist = buildHoughHist(hole_candidates,train_features,hist_size2,ranges);
	//	float** values2 = new float*[1];// = getMaxHistValues(hist,hist_size);
	//	int count2 = 1;
	//	int votes2 = 0;
	//	votes2 = getMaxHistValues(hist,hist_size2,ranges,values2,count2);
	//#if defined(_VERBOSE)
	//		printf("Votes2: %d\n",votes2);
	//#endif
	//	cvReleaseSparseMat(&hist);
	//
	//	if (votes2 > votes)
	//	{
	//		for (int i=0;i<count;i++)
	//			delete[] values[i];
	//		delete[] values;
	//		values = new float*[count2];
	//		for (int i=0;i<count2;i++)
	//		{
	//			values[i] = new float[6];
	//			for (int j=0;j<6;j++)
	//			{
	//				values[i][j] = values2[i][j];
	//			}
	//		}
	//		count = count2;
	//		votes = votes2;
	//	}
	//	for (int i=0;i<count2;i++)
	//		delete[] values2[i];
	//	delete[] values2;
	//
	//// End

	vector<feature_t> hole_features;
	vector<feature_t> hole_features_corrected;
	vector<feature_t> res_features;

	float accuracy = sqrt((float)((train_features[1].pt.x -train_features[0].pt.x)*(train_features[1].pt.x -train_features[0].pt.x)+
		(train_features[1].pt.y -train_features[0].pt.y)*(train_features[1].pt.y -train_features[0].pt.y)));
	float error = (float)1e38;
	int index = -1;
	//IplImage* t = cvCloneImage(resImage);

	//We have count candidates to the outlet position
	//Chooseng the best one
	for (int j=0;j<count;j++)
	{
		float currError;
		hole_features.clear();
		calcOutletPosition(train_features, values[j],hole_features);

		//drawing current candidate if we have to do it
		if (ghtImage && (hole_features.size() > 0) && output_path && base_filename)
		{
			char path[1024];
			sprintf(path,"%s/%s_%d.jpg",output_path,base_filename,j);
			IplImage* tmp = cvCloneImage(ght);
			//cvNamedWindow(path);
			//cvShowImage(path,tmp);
			//cvWaitKey();
			for(int l = 0; l < (int)hole_features.size(); l++)
			{

				CvScalar pointColor = hole_features[l].class_id == 0 ? cvScalar(0,255,50) : cvScalar(255,0,50);
				cvLine(tmp, cvPoint((int)(hole_features[l].pt.x+7), (int)(hole_features[l].pt.y)), cvPoint((int)(hole_features[l].pt.x-7),(int)(hole_features[l].pt.y)),pointColor,2);
				cvLine(tmp, cvPoint((int)(hole_features[l].pt.x), (int)(hole_features[l].pt.y+7)), cvPoint((int)(hole_features[l].pt.x), (int)(hole_features[l].pt.y-7)),pointColor,2);
			}
			//cvShowImage(path,tmp);
			//cvWaitKey();
			cvSaveImage(path,tmp);
			cvReleaseImage(&tmp);
		}

		if (ghtImage)
		{
			for(int i = 0; i < (int)hole_features.size(); i++)
			{
				CvScalar pointColor = hole_features[i].class_id == 0 ? cvScalar(0,255,50) : cvScalar(255,0,50);
				cvLine(ghtImage, cvPoint((int)(hole_features[i].pt.x+7), (int)(hole_features[i].pt.y)), cvPoint((int)(hole_features[i].pt.x-7),(int)(hole_features[i].pt.y)),pointColor,2);
				cvLine(ghtImage, cvPoint((int)(hole_features[i].pt.x), (int)(hole_features[i].pt.y+7)), cvPoint((int)(hole_features[i].pt.x), (int)(hole_features[i].pt.y-7)),pointColor,2);

			}
		}
		//Repaint features
		vector<feature_t> hole_candidates2;
		if (hole_features.size() >0)
		{
			repaintFeatures(hole_candidates, hole_features, hole_candidates2,(int)accuracy);
		}
		else
		{
			hole_candidates2 = hole_candidates;
		}
		//  CvScalar color_parts[] = {CV_RGB(255, 255, 0), CV_RGB(0, 255, 255)};
		//  for(int i = 0; i < (int)hole_candidates.size(); i++)
		//  {
		//      cvCircle(ght, hole_candidates2[i].pt, hole_candidates2[i].size, color_parts[hole_candidates2[i].class_id], 2);

		//  }
		//cvNamedWindow("1");
		//cvShowImage("1",ght);
		//cvWaitKey();
		//End of
#if 1
		calcExactLocation(hole_candidates2,train_features,hole_features,hole_features_corrected,currError,(int)accuracy);
#else
		calcExactLocation(hole_candidates2,train_features,hole_features,hole_features_corrected,currError,(int)accuracy,false);
#endif

		if (currError < error)
		{
			index = j;
			error = currError;
			res_features.clear();
			for (int i =0; i< (int)hole_features_corrected.size(); i++)
				res_features.push_back(hole_features_corrected[i]);
		}

//        printf("candidate %d, error %f\n", j, currError);

	}

#if 1
	if (res_features.size() > 0)
	{
		filterOutletOutliers(hole_candidates,res_features,(int)accuracy);
	}
#endif

	// int outlet_holes_count = 3;
	if ((int)(res_features.size()) > 0)
	{
		convertFeaturesToOutlet(res_features, holes, resImage);
	}
	else
	{
		error = max_error;
	}


	for (int i=0;i<count;i++)
		delete[] values[i];
	delete[] values;
	if (ghtImage)
		cvReleaseImage(&ght);

	return error;
}
