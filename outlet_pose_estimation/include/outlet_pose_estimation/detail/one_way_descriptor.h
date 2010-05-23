/*
 *  one_way_desctiptor.h
 *
 *
 *  Created by Victor  Eruhimov on 4/19/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_ONE_WAY_DESCRIPTOR)
#define _ONE_WAY_DESCRIPTOR

#include <string>

#include <cv.h>

#if defined (_KDTREE)
#include <cxcore.h>
#endif

inline int round(float value)
{
    if(value > 0)
    {
        return int(value + 0.5f);
    }
    else
    {
        return int(value - 0.5f);
    }
}

inline CvRect resize_rect(CvRect rect, float alpha)
{
	return cvRect(rect.x + round((float)(0.5*(1 - alpha)*rect.width)), rect.y + round((float)(0.5*(1 - alpha)*rect.height)),
				  round(rect.width*alpha), round(rect.height*alpha));
}

/*
inline CvRect fit_rect_roi(CvRect rect, CvRect roi)
{
	CvRect fit = rect;
	fit.x = MAX(fit.x, roi.x);
	fit.y = MAX(fit.y, roi.y);
	fit.width = MIN(fit.width, roi.x + roi.width - fit.x - 1);
	fit.height = MIN(fit.height, roi.y + roi.height - fit.y - 1);
	assert(fit.width > 0);
	assert(fit.height > 0);
	return(fit);
}

inline CvRect fit_rect(CvRect rect, IplImage* img)
{
	CvRect roi = cvGetImageROI(img);
	return fit_rect_roi(rect, roi);
}

inline CvRect double_rect(CvRect small_rect)
{
	return cvRect(small_rect.x - small_rect.width/2, small_rect.y - small_rect.height/2,
				  small_rect.width*2, small_rect.height*2);
}*/
CvMat* ConvertImageToMatrix(IplImage* patch);

class CvCameraPose
{
public:
    CvCameraPose()
    {
        m_rotation = cvCreateMat(1, 3, CV_32FC1);
        m_translation = cvCreateMat(1, 3, CV_32FC1);
    };

    ~CvCameraPose()
    {
        cvReleaseMat(&m_rotation);
        cvReleaseMat(&m_translation);
    };

    void SetPose(CvMat* rotation, CvMat* translation)
    {
        cvCopy(rotation, m_rotation);
        cvCopy(translation, m_translation);
    };

    CvMat* GetRotation() {return m_rotation;};
    CvMat* GetTranslation() {return m_translation;};

protected:
    CvMat* m_rotation;
    CvMat* m_translation;
};

// CvAffinePose: defines a parameterized affine transformation of an image patch.
// An image patch is rotated on angle phi (in degrees), then scaled lambda1 times
// along horizontal and lambda2 times along vertical direction, and then rotated again
// on angle (theta - phi).
/*class CvAffinePose
{
public:
    float phi;
    float theta;
    float lambda1;
    float lambda2;
};*/

// AffineTransformPatch: generates an affine transformed image patch.
// - src: source image (roi is supported)
// - dst: output image. ROI of dst image should be 2 times smaller than ROI of src.
// - pose: parameters of an affine transformation
void AffineTransformPatch(IplImage* src, IplImage* dst, CvAffinePose pose);

// GenerateAffineTransformFromPose: generates an affine transformation matrix from CvAffinePose instance
// - size: the size of image patch
// - pose: affine transformation
// - transform: 2x3 transformation matrix
void GenerateAffineTransformFromPose(CvSize size, CvAffinePose pose, CvMat* transform);

// Generates a random affine pose
CvAffinePose GenRandomAffinePose();


const static int num_mean_components = 500;
const static float noise_intensity = 0.15f;

// CvOneWayDescriptor: incapsulates a descriptor for a single point
class CvOneWayDescriptor
{
public:
    CvOneWayDescriptor();
    ~CvOneWayDescriptor();

    // allocates memory for given descriptor parameters
    void Allocate(int pose_count, CvSize size, int nChannels);

    // GenerateSamples: generates affine transformed patches with averaging them over small transformation variations.
    // If external poses and transforms were specified, uses them instead of generating random ones
    // - pose_count: the number of poses to be generated
    // - frontal: the input patch (can be a roi in a larger image)
    // - norm: if nonzero, normalizes the output patch so that the sum of pixel intensities is 1
    void GenerateSamples(int pose_count, IplImage* frontal, int norm = 0);

    // GenerateSamplesFast: generates affine transformed patches with averaging them over small transformation variations.
    // Uses precalculated transformed pca components.
    // - frontal: the input patch (can be a roi in a larger image)
    // - pca_hr_avg: pca average vector
    // - pca_hr_eigenvectors: pca eigenvectors
    // - pca_descriptors: an array of precomputed descriptors of pca components containing their affine transformations
    //   pca_descriptors[0] corresponds to the average, pca_descriptors[1]-pca_descriptors[pca_dim] correspond to eigenvectors
    void GenerateSamplesFast(IplImage* frontal, CvMat* pca_hr_avg,
                    CvMat* pca_hr_eigenvectors, CvOneWayDescriptor* pca_descriptors);

    // sets the poses and corresponding transforms
    void SetTransforms(CvAffinePose* poses, CvMat** transforms);

    // Initialize: builds a descriptor.
    // - pose_count: the number of poses to build. If poses were set externally, uses them rather than generating random ones
    // - frontal: input patch. Can be a roi in a larger image
    // - feature_name: the feature name to be associated with the descriptor
    // - norm: if 1, the affine transformed patches are normalized so that their sum is 1
    void Initialize(int pose_count, IplImage* frontal, const char* feature_name = 0, int norm = 0);

    // InitializeFast: builds a descriptor using precomputed descriptors of pca components
    // - pose_count: the number of poses to build
    // - frontal: input patch. Can be a roi in a larger image
    // - feature_name: the feature name to be associated with the descriptor
    // - pca_hr_avg: average vector for PCA
    // - pca_hr_eigenvectors: PCA eigenvectors (one vector per row)
    // - pca_descriptors: precomputed descriptors of PCA components, the first descriptor for the average vector
    // followed by the descriptors for eigenvectors
    void InitializeFast(int pose_count, IplImage* frontal, const char* feature_name,
                                            CvMat* pca_hr_avg, CvMat* pca_hr_eigenvectors, CvOneWayDescriptor* pca_descriptors);

    // ProjectPCASample: unwarps an image patch into a vector and projects it into PCA space
    // - patch: input image patch
    // - avg: PCA average vector
    // - eigenvectors: PCA eigenvectors, one per row
    // - pca_coeffs: output PCA coefficients
    void ProjectPCASample(IplImage* patch, CvMat* avg, CvMat* eigenvectors, CvMat* pca_coeffs) const;

    // InitializePCACoeffs: projects all warped patches into PCA space
    // - avg: PCA average vector
    // - eigenvectors: PCA eigenvectors, one per row
    void InitializePCACoeffs(CvMat* avg, CvMat* eigenvectors);

    // EstimatePose: finds the closest match between an input patch and a set of patches with different poses
    // - patch: input image patch
    // - pose_idx: the output index of the closest pose
    // - distance: the distance to the closest pose (L2 distance)
    void EstimatePose(IplImage* patch, int& pose_idx, float& distance) const;

    // EstimatePosePCA: finds the closest match between an input patch and a set of patches with different poses.
    // The distance between patches is computed in PCA space
    // - patch: input image patch
    // - pose_idx: the output index of the closest pose
    // - distance: distance to the closest pose (L2 distance in PCA space)
    // - avg: PCA average vector. If 0, matching without PCA is used
    // - eigenvectors: PCA eigenvectors, one per row
    void EstimatePosePCA(CvArr* patch, int& pose_idx, float& distance, CvMat* avg, CvMat* eigenvalues) const;

    // GetPatchSize: returns the size of each image patch after warping (2 times smaller than the input patch)
    CvSize GetPatchSize() const
    {
        return m_patch_size;
    }

    // GetInputPatchSize: returns the required size of the patch that the descriptor is built from
    // (2 time larger than the patch after warping)
    CvSize GetInputPatchSize() const
    {
        return cvSize(m_patch_size.width*2, m_patch_size.height*2);
    }

    // GetPatch: returns a patch corresponding to specified pose index
    // - index: pose index
    // - return value: the patch corresponding to specified pose index
    IplImage* GetPatch(int index);

    // GetPose: returns a pose corresponding to specified pose index
    // - index: pose index
    // - return value: the pose corresponding to specified pose index
    CvAffinePose GetPose(int index) const;

    // Save: saves all patches with different poses to a specified path
    void Save(const char* path);

    // ReadByName: reads a descriptor from a file storage
    // - fs: file storage
    // - parent: parent node
    // - name: node name
    // - return value: 1 if succeeded, 0 otherwise
    int ReadByName(CvFileStorage* fs, CvFileNode* parent, const char* name);

    // Write: writes a descriptor into a file storage
    // - fs: file storage
    // - name: node name
    void Write(CvFileStorage* fs, const char* name);

    // GetFeatureName: returns a name corresponding to a feature
    const char* GetFeatureName() const;

    // GetCenter: returns the center of the feature
    CvPoint GetCenter() const;

    void SetPCADimHigh(int pca_dim_high) {m_pca_dim_high = pca_dim_high;};
    void SetPCADimLow(int pca_dim_low) {m_pca_dim_low = pca_dim_low;};

	int GetPCADimLow() const;
	int GetPCADimHigh() const;

	CvMat** GetPCACoeffs() const {return m_pca_coeffs;}

protected:
    int m_pose_count; // the number of poses
    CvSize m_patch_size; // size of each image
    IplImage** m_samples; // an array of length m_pose_count containing the patch in different poses
    IplImage* m_input_patch;
    IplImage* m_train_patch;
    CvMat** m_pca_coeffs; // an array of length m_pose_count containing pca decomposition of the patch in different poses
    CvAffinePose* m_affine_poses; // an array of poses
    CvMat** m_transforms; // an array of affine transforms corresponding to poses

    std::string m_feature_name; // the name of the feature associated with the descriptor
    CvPoint m_center; // the coordinates of the feature (the center of the input image ROI)

    int m_pca_dim_high; // the number of descriptor pca components to use for generating affine poses
    int m_pca_dim_low; // the number of pca components to use for comparison
};

void FindOneWayDescriptor(int desc_count, const CvOneWayDescriptor* descriptors, IplImage* patch, int& desc_idx, int& pose_idx, float& distance,
                          CvMat* avg = 0, CvMat* eigenvalues = 0);

void FindOneWayDescriptor(int desc_count, const CvOneWayDescriptor* descriptors, IplImage* patch, int n,
                    std::vector<int>& desc_idxs, std::vector<int>&  pose_idxs, std::vector<float>& distances,
                    CvMat* avg = 0, CvMat* eigenvalues = 0);

#if defined(_KDTREE)
void FindOneWayDescriptor(cv::flann::Index* m_pca_descriptors_tree, CvSize patch_size, int m_pca_dim_low, int m_pose_count, IplImage* patch, int& desc_idx, int& pose_idx, float& distance,
                          CvMat* avg = 0, CvMat* eigenvalues = 0);
#endif

void FindOneWayDescriptorEx(int desc_count, const CvOneWayDescriptor* descriptors, IplImage* patch,
                            float scale_min, float scale_max, float scale_step,
                            int& desc_idx, int& pose_idx, float& distance, float& scale,
                            CvMat* avg, CvMat* eigenvectors);

void FindOneWayDescriptorEx(int desc_count, const CvOneWayDescriptor* descriptors, IplImage* patch,
							float scale_min, float scale_max, float scale_step,
							int n, std::vector<int>& desc_idxs, std::vector<int>& pose_idxs,
							std::vector<float>& distances, std::vector<float>& scales,
							CvMat* avg, CvMat* eigenvectors);

#if defined(_KDTREE)
void FindOneWayDescriptorEx(cv::flann::Index* m_pca_descriptors_tree, CvSize patch_size, int m_pca_dim_low, int m_pose_count, IplImage* patch,
							float scale_min, float scale_max, float scale_step,
							int& desc_idx, int& pose_idx, float& distance, float& scale,
							CvMat* avg, CvMat* eigenvectors);
#endif


#endif //_ONE_WAY_DESCRIPTOR

