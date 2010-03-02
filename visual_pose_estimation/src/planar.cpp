#include <visual_pose_estimation/planar.h>

namespace cv{

Point3f massCenter(const vector<Point3f>& points)
{
    Point3f center(0.0f, 0.0f, 0.0f);
    for(size_t i = 0; i < points.size(); i++)
    {
        center += points[i];
    }
    
    center = center*(1.0f/points.size());
    
    return center;
}
    
Point3f crossProduct(Point3f& p1, Point3f& p2)
{
    Point3f p3(p1.y*p2.z - p1.z*p2.y, p1.z*p2.x - p1.x*p2.z, p1.x*p2.y - p1.y*p2.x);
    return p3;
}
    
    
// Algorithm:
// plane equation: P*N + c = 0
// we find point rays in 3D from image points and camera parameters
// then we fit c by minimizing average L2 distance between rotated and translated object points
// and points found by crossing point rays with plane. We use the fact that center of mass
// of object points and fitted points should coincide.
void findPlanarObjectPose(const vector<Point3f>& _object_points, const Mat& image_points, const Point3f& normal,
                const Mat& intrinsic_matrix, const Mat& distortion_coeffs, double& alpha, double& C, vector<Point3f>& object_points_crf)
{
    vector<Point2f> _rays;
    undistortPoints(image_points, _rays, intrinsic_matrix, distortion_coeffs);

    // filter out rays that are parallel to the plane
    vector<Point3f> rays;
    vector<Point3f> object_points;
    for(size_t i = 0; i < _rays.size(); i++)
    {
        Point3f ray(_rays[i].x, _rays[i].y, 1.0f);
        double proj = ray.dot(normal);
        if(fabs(proj) > std::numeric_limits<double>::epsilon())
        {
            rays.push_back(ray*(1.0/ray.dot(normal)));
            object_points.push_back(_object_points[i]);
        }
    }

    Point3f pc = massCenter(rays);
    Point3f p0c = massCenter(object_points);
    
    vector<Point3f> drays;
    drays.resize(rays.size());
    for(size_t i = 0; i < rays.size(); i++)
    {
        drays[i] = rays[i] - pc;
        object_points[i] -= p0c;
    }

    double s1 = 0.0, s2 = 0.0, s3 = 0.0;
    for(size_t i = 0; i < rays.size(); i++)
    {
        Point3f vprod = crossProduct(drays[i], object_points[i]);
        s1 += vprod.dot(normal);
        s2 += drays[i].dot(object_points[i]);
        s3 += drays[i].dot(drays[i]);
    }
    
    alpha = atan2(s1, s2);
    C = (s2*cos(alpha) + s1*sin(alpha))/s3;
    
//    printf("alpha = %f, C = %f\n", alpha, C);
    
    object_points_crf.resize(rays.size());
    for(size_t i = 0; i < rays.size(); i++)
    {
        object_points_crf[i] = rays[i]*C;
    }
}

Point3f getPlanarObjectNormal(const Mat& objectPoints)
{
    Point3f off1 = objectPoints.at<Point3f>(1, 0) - objectPoints.at<Point3f>(0, 0);
    Point3f off2 = objectPoints.at<Point3f>(2, 0) - objectPoints.at<Point3f>(0, 0);
    Point3f normal0 = crossProduct(off1, off2);
    normal0 = normal0*(1.0/norm(normal0));
    return normal0;
}

void fit2DRotation(const vector<Point3f>& points1, const vector<Point3f>& points2, Point3f normal, Mat& rvec)
{
    Point3f center1 = massCenter(points1);
    Point3f center2 = massCenter(points2);

    float sum1 = 0.0f, sum2 = 0.0f;
    for(size_t i = 0; i < points1.size(); i++)
    {
        Point3f off1 = points1[i] - center1;
        Point3f off2 = points2[i] - center2;
        Point3f off1p = crossProduct(off1, normal);
        sum1 += off2.dot(off1);
        sum2 += off2.dot(off1p);
    }

    double alpha = -atan2(sum1, sum2);

    if(rvec.empty() == true)
    {
        rvec.create(3, 1, CV_32F);
    }
    rvec.at<Point3f>(0, 0) = normal*(alpha/norm(normal));
}

void solvePlanarPnP(const Mat& objectPoints, const Mat& imagePoints, const Mat& cameraMatrix, const Mat& distCoeffs,
                    Mat& _rvec, Mat& _tvec, bool useExtrinsicGuess)
{
    CV_Assert(objectPoints.depth() == CV_32F && imagePoints.depth() == CV_32F);

    if(useExtrinsicGuess == false)
    {
        solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, _rvec, _tvec, false);
        return;
    }

    Mat rvec, tvec;
    _rvec.convertTo(rvec, CV_32FC1);
    _tvec.convertTo(tvec, CV_32FC1);

    // calculate rotation matrix
    Mat R(3, 3, CV_32FC1);
    Rodrigues(rvec, R);
    CV_Assert(R.type() == CV_32FC1);

    // calculate object normal
    Point3f normal0 = getPlanarObjectNormal(objectPoints);
//    printf("Normal0: %f %f %f\n", normal0.x, normal0.y, normal0.z);

    Mat Normal0(3, 1, CV_32F);
    Normal0.at<Point3f>(0, 0) = normal0;
    Mat Normal = R*Normal0;
    Point3f normal = Normal.at<Point3f>(0, 0);
    normal = normal*(1.0/norm(normal));
    if(normal.z < 0) normal = -normal; // z points from the camera
//    printf("Normal: %f %f %f\n", normal.x, normal.y, normal.z);

    vector<Point3f> rotated_object_points;
    rotated_object_points.resize(objectPoints.rows);
    for(size_t i = 0; i < rotated_object_points.size(); i++)
    {
        Mat p = objectPoints.rowRange(i, i + 1);
        p = p.reshape(1, 3);        
        Mat res = R*p;
        rotated_object_points[i] = res.at<Point3f>(0, 0);
    }

    double alpha, C;
    vector<Point3f> object_points_crf;
    findPlanarObjectPose(rotated_object_points, imagePoints, normal, cameraMatrix, distCoeffs, alpha, C, object_points_crf);

    Mat rp(3, 1, CV_32FC1);
    rp.at<Point3f>(0, 0) = normal*alpha;

    Mat Rp;
    Rodrigues(rp, Rp);

    R = Rp*R;
    Rodrigues(R, rvec);

    Point3f center1 = massCenter(rotated_object_points);
    Mat mcenter1(3, 1, CV_32FC1, &center1);
    Mat mcenter1_alpha = Rp*mcenter1;
    Point3f center1_alpha = mcenter1_alpha.at<Point3f>(0, 0);

    Point3f center2 = massCenter(object_points_crf);
    tvec.at<Point3f>(0, 0) = center2 - center1_alpha;

    Mat mobj;
    objectPoints.copyTo(mobj);
    mobj = mobj.reshape(1);

    CV_Assert(R.type() == CV_32FC1 && mobj.type() == CV_32FC1);
    Mat mrobj = R*mobj.t();
    mrobj = mrobj.t();
    Point3f p1 = mrobj.at<Point3f>(0, 0) + center2 - center1;
    Point3f p2 = object_points_crf[0];
//    printf("point1: %f %f %f\n", p1.x, p1.y, p1.z);
//    printf("point2: %f %f %f\n", p2.x, p2.y, p2.z);

    rvec.convertTo(_rvec, _rvec.depth());
    tvec.convertTo(_tvec, _tvec.depth());
}

} //namespace cv
