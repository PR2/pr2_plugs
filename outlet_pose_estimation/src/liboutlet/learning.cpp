/*
 *  learning.cpp
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 12/29/08.
 *  Copyright 2008 Argus Corp. All rights reserved.
 *
 */

#include <stdio.h>
#include "outlet_pose_estimation/detail/learning.h"

CvRTrees* train_rf(CvMat* predictors, CvMat* labels)
{
	int stat[2];
	get_stat(labels, stat);
	printf("%d negative samples, %d positive samples\n", stat[0], stat[1]);
	
	const int tree_count = 500;
	const float priors[] = {0.25f,0.75f};
	CvRTrees* rtrees = new CvRTrees();
	CvRTParams rtparams = CvRTParams(5, 10, 0, false, 2, priors, true, 
									 (int)sqrt((float)predictors->cols), tree_count, 1e-6, 
									 CV_TERMCRIT_ITER + CV_TERMCRIT_EPS);
	CvMat* var_type = cvCreateMat(predictors->cols + 1, 1, CV_8UC1);
	for(int i = 0; i < predictors->cols; i++)
	{
		*(int*)(var_type->data.ptr + i*var_type->step) = CV_VAR_NUMERICAL;
	}
	*(int*)(var_type->data.ptr + predictors->cols*var_type->step) = CV_VAR_CATEGORICAL;
	rtrees->train(predictors, CV_ROW_SAMPLE, labels, 0, 0, var_type, 0, rtparams);
	return rtrees;
}

void get_stat(CvMat* labels, int* stat)
{
	stat[0] = 0;
	stat[1] = 0;
	for(int i = 0; i < labels->rows; i++)
	{
		int val = *(int*)(labels->data.ptr + labels->step*i);
		stat[val]++;
	}
}
