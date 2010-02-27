#if !defined(_CALIB_LOWRES_H)
#define _CALIB_LOWRES_H

#include <opencv/cv.h>

int cvFindChessboardCornersLowres(IplImage* img, CvSize size, CvPoint2D32f* corners, int* corner_count = 0);

#endif
