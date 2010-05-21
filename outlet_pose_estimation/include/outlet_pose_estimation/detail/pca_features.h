/*
 *  pca_features.h
 *  online_patch
 *
 *  Created by Victor  Eruhimov on 5/15/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_PCA_FEATURES_H)
#define _PCA_FEATURES_H

#include <cv.h>
#include "features.h"

void savePCAFeatures(const char* filename, CvMat* avg, CvMat* eigenvectors);
void calcPCAFeatures(std::vector<IplImage*>& patches, const char* filename);
void loadPCAFeatures(const char* path, std::vector<IplImage*>& patches);

#endif //_PCA_FEATURES_H
