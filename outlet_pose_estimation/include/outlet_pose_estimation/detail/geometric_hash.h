/*
 *  geometric_hash.h
 *  edge_matcher
 *
 *  Created by Victor  Eruhimov on 11/1/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#if !defined(_GEOMETRIC_HASH)
#define _GEOMETRIC_HASH

#include <math.h>

#include <cv.h>

#include <vector>
#include <map>
#include <list>
#include <utility>

#include "one_way_descriptor_base.h"
#include "features.h"

/*class AffineBasis
{
public:
    virtual cv::Point2f getOrigin() const = 0;
    virtual void getBasis(cv::Point2f* basis) const = 0;
    virtual cv::Point2f getCoords(cv::Point2f point) const = 0;
};*/

///*inline*/ float length(cv::Point2f p);
/*{
    return sqrt(float(p.x*p.x) + p.y*p.y);
//}*/

#define PI 3.1415926

class AffineBasis
{
protected:
    int model_id;
    cv::Point2f origin;
    cv::Point2f basis[2];

public:
    AffineBasis(cv::Point2f _origin, const cv::Point2f* _basis, int _model_id)
    {
        origin = _origin;
        basis[0] = _basis[0];
        basis[1] = _basis[1];
        model_id = _model_id;
    };
    AffineBasis(cv::Point2f _origin, cv::Point2f point1, cv::Point2f point2, int _model_id)
    {
        origin = _origin;
        basis[0] = point1 - origin;
        basis[1] = point2 - origin;
        model_id = _model_id;
    };

    ~AffineBasis() {};

    cv::Point2f getOrigin() const {return origin;};
    void getBasis(cv::Point2f* _basis) const
    {
        _basis[0] = basis[0];
        _basis[1] = basis[1];
    };

    cv::Point2f getCoords(cv::Point2f point) const;
    cv::Point2f getPoint(cv::Point2f coords) const;

    int getModelID() const {return model_id;};

    float getAngle() const {return acos(basis[0].dot(basis[1])/(length(basis[0])*length(basis[1])));};
};

typedef int ModelBasisID;

class GeometricHash
{
protected:
    cv::Size size;
    cv::Point2f range[2]; // 0 is minimum, 1 is maximum
    std::vector<AffineBasis> bases;
    std::vector<std::list<ModelBasisID> > hash;
    std::list<ModelBasisID> empty_list;

    int getBin(cv::Point2f coords) const;

public:
    GeometricHash(cv::Size _size, cv::Point2f range_min, cv::Point2f range_max);
    ModelBasisID addBasis(const AffineBasis& basis);
    const std::vector<AffineBasis>& getBases() const;
    void addEntry(const ModelBasisID& basisID, cv::Point2f point);
    const std::list<ModelBasisID>& getEntries(cv::Point2f point) const;

};

class GeometricHash3D
{
protected:
    cv::Point3i size;
    cv::Point3f range[2]; // 0 is minimum, 1 is maximum
    std::vector<AffineBasis> bases;
    std::vector<std::list<ModelBasisID> > hash;
    std::list<ModelBasisID> empty_list;

    int getBin(cv::Point3f coords) const;

public:
    GeometricHash3D(cv::Point3i _size, const cv::Point3f* _range);
    ModelBasisID addBasis(const AffineBasis& basis);
    const std::vector<AffineBasis>& getBases() const;
    void addEntry(const ModelBasisID& basisID, CvSeq* seq, int idx_offset, int idx_point);
    const std::list<ModelBasisID>& getEntries(cv::Point3f point) const;
};

class EdgeMatcher
{
protected:
    std::vector<CvSeq*> edges;
    GeometricHash3D hash;
    static const float min_angle;

public:
    typedef std::map<int,std::pair<int,int> > ModelVotes;

    EdgeMatcher(cv::Point3f _size, const cv::Point3f* _range) : hash(_size, _range) {};

    int addModel(CvSeq* edge);
    void addModelBasis(CvSeq* edge, int idx_origin, const AffineBasis& basis);
    AffineBasis match(CvSeq* edge, std::map<int,std::pair<int,int> >& votes) const;
    void aggregateVotes(const std::vector<int>& basis_votes, std::map<int,std::pair<int,int> >& agg_votes) const;
    void matchBasis(CvSeq* edge, const AffineBasis& basis, int idx_origin, std::vector<int>& votes) const;
    CvSeq* getModel(int modelID) {return edges[modelID];};
    CvSeq* getModelByBasisID(int basis_id) const {return edges[hash.getBases()[basis_id].getModelID()];};
    const AffineBasis& getBasis(int basis_id) const {return hash.getBases()[basis_id];};
    const GeometricHash3D& getHash() const {return hash;};

    static bool votes_greater(const std::pair<int,std::pair<int,int> >& elem1, const std::pair<int,std::pair<int,int> >& elem2)
    {
        return elem1.second.second < elem2.second.second;
    }
};

AffineBasis getPointEdgeBasis(cv::Point2f point, CvSeq* edge, int i, int modelID);
float fitEdges(CvSeq* model, const AffineBasis& model_basis, CvSeq* test, const AffineBasis& test_basis);
float fitEdgesSym(CvSeq* model_seq, const AffineBasis& model_basis, CvSeq* test_seq, const AffineBasis& test_basis);
float fitPoints(const std::vector<cv::Point2f>& set1, const std::vector<cv::Point2f>& set2);
float fitPointsSym(const std::vector<cv::Point2f>& set1, const std::vector<cv::Point2f>& set2);
CvSeq* mapContour(CvSeq* contour, AffineBasis src, AffineBasis dst, CvMemStorage* storage);
void mapPoints(const std::vector<cv::Point2f>& src, const AffineBasis& src_basis, const AffineBasis& dst_basis, std::vector<cv::Point2f>& dst);
void mapPoints(const std::vector<KeyPointEx>& src, const AffineBasis& src_basis, const AffineBasis& dst_basis, std::vector<KeyPointEx>& dst);

class PointEdgeMatcher : public EdgeMatcher
{
public:
    typedef std::pair<cv::Point2f,CvSeq*> Model;

private:
    std::vector<Model> models;

public:
    PointEdgeMatcher(cv::Point3f _size, const cv::Point3f* _range) : EdgeMatcher(_size, _range) {};

    int addModel(const Model& model);
    const Model& getModel(int model_id) {return models[model_id];};
    const Model& getModelByBasisID(int basis_id) const {return models[hash.getBases()[basis_id].getModelID()];};
    AffineBasis match(cv::Point2f point, CvSeq* edge, std::map<int,std::pair<int, int> >& votes) const;
};

float calcAffineSeqDist(const AffineBasis& basis, CvSeq* seq, int idx1, int idx2, int is_mapped = 0);

class PointMatcher
{
public:
    struct PointMatcherParams
    {
        PointMatcherParams(float _min_angle = 0.5f, float _max_basis_length = 100.0f, float _min_basis_length = 10.0f,
            int _min_hash_votes = 3, float _min_validated_votes = 5.0f, float _min_distortion_ratio = 0.8f) :
            min_angle(_min_angle), max_basis_length(_max_basis_length), min_basis_length(_min_basis_length),
            min_hash_votes(_min_hash_votes), min_validated_votes(_min_validated_votes), min_distortion_ratio(_min_distortion_ratio) {};

        float min_angle;
        float max_basis_length;
        float min_basis_length;
        int min_hash_votes;
        float min_validated_votes;
        float min_distortion_ratio;
    };

protected:
    GeometricHash hash;
    std::vector<KeyPointEx> template_points;
    PointMatcherParams params;

public:
    typedef std::map<int,std::pair<int,int> > ModelVotes;

    PointMatcher(cv::Size _size, cv::Point2f range_min, cv::Point2f range_max,
        PointMatcher::PointMatcherParams _params = PointMatcher::PointMatcherParams()) : hash(_size, range_min, range_max), params(_params) {};

    int addModel(const std::vector<KeyPointEx>& points);
    void addModelBasis(const std::vector<KeyPointEx>& points, const AffineBasis& basis);
    int match(const std::vector<KeyPointEx>& points, std::vector<float>& votes,
                            std::vector<std::pair<AffineBasis,AffineBasis> >& matched_bases) const;
    void matchBasis(const std::vector<KeyPointEx>& points, const AffineBasis& basis, std::vector<int>& votes) const;
    const AffineBasis& getBasis(int basis_id) const {return hash.getBases()[basis_id];};
    const GeometricHash& getHash() const {return hash;};
    const std::vector<KeyPointEx>& getTemplatePoints() const {return template_points;};
};

float validatePointMatch(const std::vector<KeyPointEx>& set1, const AffineBasis& basis1,
                         const std::vector<KeyPointEx>& set2, const AffineBasis& basis2);

void getProximityPoints(const std::vector<KeyPointEx>& points, KeyPointEx point, float max_dist, std::vector<int>& indices);
void findClosestPoint(const std::vector<KeyPointEx>& guess, const std::vector<KeyPointEx>& candidates, std::vector<KeyPointEx>& output, std::vector<bool>& is_detected, float max_dist);
double affineDistortionRatio(const AffineBasis& basis1, const AffineBasis& basis2);


#endif //_GEOMETRIC_HASH
