/*
 *  learning.h
 *  outlet_model
 *
 *  Created by Victor  Eruhimov on 12/29/08.
 *  Copyright 2008 Argus Corp. All rights reserved.
 *
 */
#if !defined(_LEARNING_H)
#define _LEARNING_H

#include <ml.h>

void get_stat(CvMat* labels, int* stat);
CvRTrees* train_rf(CvMat* predictors, CvMat* labels);

#endif //_LEARNING_H
