/*
 *  geometric_hash.cpp
 *  edge_matcher
 *
 *  Created by Victor  Eruhimov on 11/1/09.
 *  Copyright 2009 Argus Corp. All rights reserved.
 *
 */

#include <stdio.h>
#include "outlet_pose_estimation/detail/geometric_hash.h"

cv::Point2f AffineBasis::getCoords(cv::Point2f point) const
{
    point = point - origin;

    cv::Point2f coords;
    float d = basis[0].x*basis[1].y - basis[1].x*basis[0].y;
    coords.x = (point.x*basis[1].y - point.y*basis[1].x)/d;
    coords.y = (point.y*basis[0].x - point.x*basis[0].y)/d;

    return coords;
}

cv::Point2f AffineBasis::getPoint(cv::Point2f coords) const
{
    return origin + basis[0]*coords.x + basis[1]*coords.y;
}


GeometricHash::GeometricHash(cv::Size _size, cv::Point2f range_min, cv::Point2f range_max)
{
    size = _size;
    hash.resize(size.width*size.height);

    range[0] = range_min;
    range[1] = range_max;
}

int GeometricHash::getBin(cv::Point2f coords) const
{
    int idx[2];
    idx[0] = (int)floor((coords.x - range[0].x)/(range[1].x - range[0].x)*size.width);
    idx[1] = (int)floor((coords.y - range[0].y)/(range[1].y - range[0].y)*size.height);
    if(idx[0] >= size.width || idx[1] >= size.height)
    {
        return -1;
    }

    return idx[1]*size.width + idx[0];
}

ModelBasisID GeometricHash::addBasis(const AffineBasis& basis)
{
    bases.push_back(basis);
    return bases.size() - 1;
}

void GeometricHash::addEntry(const ModelBasisID& basisID, cv::Point2f point)
{
    const AffineBasis& basis = bases[basisID];
    cv::Point2f coords = basis.getCoords(point);
    int hashIdx = getBin(coords);
    if(hashIdx >= 0 && hashIdx < hash.size())
    {
        hash[hashIdx].push_back(basisID);
/*        if(basisID == 90)
        {
            printf("Basis 90, coords %f,%f, added to hash bin %d\n", coords.x, coords.y, hashIdx);
        }*/
    }
}

const std::list<ModelBasisID>& GeometricHash::getEntries(cv::Point2f coords) const
{
    int hashIdx = getBin(coords);
    if(hashIdx >= 0)
    {
        return hash[hashIdx];
    }
    else
    {
        return empty_list;
    }
}

const std::vector<AffineBasis>& GeometricHash::getBases() const
{
    return bases;
}


// GeometricHash3D implementation
GeometricHash3D::GeometricHash3D(cv::Point3i _size, const cv::Point3f* _range)
{
    size = _size;
    hash.resize(size.x*size.y*size.z);

    range[0] = _range[0];
    range[1] = _range[1];
}

int GeometricHash3D::getBin(cv::Point3f coords) const
{
    int idx[3];
    idx[0] = (int)round((coords.x - range[0].x)/(range[1].x - range[0].x)*size.x);
    idx[1] = (int)round((coords.y - range[0].y)/(range[1].y - range[0].y)*size.y);
    idx[2] = (int)round((coords.z - range[0].z)/(range[1].z - range[0].z)*size.z);
    if(idx[0] >= size.x || idx[1] >= size.y || idx[2] >= size.z)
    {
        return -1;
    }

    return idx[2]*size.x*size.y + idx[1]*size.x + idx[0];
}

ModelBasisID GeometricHash3D::addBasis(const AffineBasis& basis)
{
    bases.push_back(basis);
    return bases.size() - 1;
}

float calcAffineSeqDist(const AffineBasis& basis, CvSeq* seq, int idx1, int idx2, int is_mapped)
{
    if(!is_mapped)
    {
        cv::Point2f unit_vec[2] = {cv::Point2f(1.0, 0.0), cv::Point2f(0.0, 1.0)};
        seq = mapContour(seq, basis, AffineBasis(cv::Point2f(0.0, 0.0), unit_vec, -1), seq->storage);
    }
    float dist1 = fabs(cvArcLength(seq, cvSlice(idx1, idx2), 1));
    float dist2 = fabs(cvArcLength(seq, cvSlice(idx2, idx1), 1));
    float dist = MIN(dist1, dist2);
//    if(seq == (void*)0xc460f8 && dist > 20) printf("%f\n", dist);
    return dist;
}

void GeometricHash3D::addEntry(const ModelBasisID& basisID, CvSeq* seq, int idx_origin, int idx_point)
{
    const AffineBasis& basis = bases[basisID];
    cv::Point2f point2 = cv::Point2f(*(CvPoint*)cvGetSeqElem(seq, idx_point));
    cv::Point2f coords = basis.getCoords(point2);
    float dist = calcAffineSeqDist(basis, seq, idx_origin, idx_point);
    cv::Point3f point3 = cv::Point3f(coords.x, coords.y, dist);

    int hashIdx = getBin(point3);
    if(hashIdx >= 0 && hashIdx < hash.size())
    {
        hash[hashIdx].push_back(basisID);
        /*        if(basisID == 90)
         {
         printf("Basis 90, coords %f,%f, added to hash bin %d\n", coords.x, coords.y, hashIdx);
         }*/
    }
}

const std::list<ModelBasisID>& GeometricHash3D::getEntries(cv::Point3f coords) const
{
    int hashIdx = getBin(coords);
    if(hashIdx >= 0)
    {
        return hash[hashIdx];
    }
    else
    {
        return empty_list;
    }
}

const std::vector<AffineBasis>& GeometricHash3D::getBases() const
{
    return bases;
}

// Edge matcher implementation

AffineBasis getEdgeBasis(CvSeq* edge, int i, int j, int modelID)
{
    cv::Point2f pibegin = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, i));
    cv::Point2f piend = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, i+1));
    cv::Point2f pjbegin = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, j));
    cv::Point2f pjend = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, j+1));

    cv::Point2f iorigin = pibegin;
    cv::Point2f idir = piend - pibegin;

    cv::Point2f jorigin = (pjbegin + pjend)*0.5f;
    cv::Point2f jdir = pjend - pjbegin;

    cv::Point2f basis[2];
    // construct a basis on (iorigin, jorigin, idir)
    basis[0] = jorigin - iorigin;
    basis[1] = idir;

    AffineBasis edge_basis(iorigin, basis, modelID);

    return edge_basis;
}

AffineBasis getPointEdgeBasis(cv::Point2f point, CvSeq* edge, int i, int modelID)
{
    cv::Point2f pibegin = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, i));
    cv::Point2f piend = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, i+1));

    cv::Point2f basis[2];
    // construct a basis on (iorigin, jorigin, idir)
    basis[0] = point - pibegin;
    basis[1] = piend - pibegin;

    AffineBasis edge_basis(pibegin, basis, modelID);

    return edge_basis;
}


int EdgeMatcher::addModel(CvSeq* edge)
{
    edges.push_back(edge);
    int model_id = edges.size() - 1;

    // iterate throuhg all possible bases (pairs of points)
    for(int i = 0; i < edge->total; i++)
    {
        for(int j = 0; j < edge->total; j++)
        {
            if(i <= j) continue;

            AffineBasis basis = getEdgeBasis(edge, i, j, model_id);
            if(basis.getAngle() < min_angle) continue;

            addModelBasis(edge, i, basis);

        }
    }

    return model_id;
}


// ****************** Edge matcher implementation ***********************
const float EdgeMatcher::min_angle = 0.15f;

void EdgeMatcher::addModelBasis(CvSeq* edge, int idx_origin, const AffineBasis& basis)
{
    ModelBasisID basisID = hash.addBasis(basis);
    CvRect bbox = cvBoundingRect(edge);
    for(int i = 0; i < edge->total; i++)
    {
//        cv::Point2f point = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, i));
        hash.addEntry(basisID, edge, idx_origin, i);
    }
}

AffineBasis EdgeMatcher::match(CvSeq* edge, std::map<int,std::pair<int, int> >& votes) const
{
    std::vector<int> basis_votes;
    basis_votes.assign(hash.getBases().size(), 0);

    // choose edge points for basis
    int i = rand()%edge->total;
    int j = rand()%edge->total;
    AffineBasis basis = getEdgeBasis(edge, i, j, -1);
    matchBasis(edge, basis, i, basis_votes);

    aggregateVotes(basis_votes, votes);

    return basis;
}

void EdgeMatcher::aggregateVotes(const std::vector<int>& basis_votes, std::map<int,std::pair<int,int> >& agg_votes) const
{
    for(size_t i = 0; i < basis_votes.size(); i++)
    {
        int modelID = hash.getBases()[i].getModelID();
        if(agg_votes.find(modelID) == agg_votes.end())
        {
            agg_votes[modelID] = std::pair<int,int>(i, basis_votes[i]);
        }
        else
        {
            if(basis_votes[i] > agg_votes[modelID].second)
            {
                agg_votes[modelID] = std::pair<int,int>(i,basis_votes[i]);
            }
        }
    }
}

void EdgeMatcher::matchBasis(CvSeq* edge, const AffineBasis& basis, int idx_origin, std::vector<int>& votes) const
{
    cv::Point2f unit_vec[] = {cv::Point2f(1.0,0.0), cv::Point2f(0.0,1.0)};
    CvSeq* mapped_edge = mapContour(edge, basis, AffineBasis(cv::Point2f(0.0), unit_vec, -1), edge->storage);

    for(int i = 0; i < edge->total; i++)
    {
#if 0
        cv::Point2f point = cv::Point2f(*(CvPoint*)cvGetSeqElem(edge, i));
        cv::Point2f coords = basis.getCoords(point);
#else
        cv::Point2f coords = cv::Point2f(*(CvPoint*)cvGetSeqElem(mapped_edge, i));
#endif
        float dist = calcAffineSeqDist(basis, mapped_edge, idx_origin, i, 1);
        if(length(coords) < 2.0) continue;
        const std::list<ModelBasisID>& bases_list = hash.getEntries(cv::Point3f(coords.x,coords.y,dist));
        for(std::list<ModelBasisID>::const_iterator it = bases_list.begin(); it != bases_list.end(); it++)
        {
//            AffineBasis basis = hash.getBases()[*it];
            votes[*it]++;
/*            if(*it == 90)
            {
                printf("Coords %f,%f: added vote for point %d\n", coords.x, coords.y, i);
            }*/
        }
    }
}

float fitEdges(CvSeq* model_seq, const AffineBasis& model_basis, CvSeq* test_seq, const AffineBasis& test_basis)
{
    float dist = 0;
    for(int i = 0; i < test_seq->total; i++)
    {
        cv::Point2f test_point = cv::Point2f(*(CvPoint*)cvGetSeqElem(test_seq, i));
        cv::Point2f test_coords = test_basis.getCoords(test_point);

        cv::Point2f test_point1 = cv::Point2f(*(CvPoint*)cvGetSeqElem(test_seq, i + 1));
        cv::Point2f test_coords1 = test_basis.getCoords(test_point1);

        float min_dist = 1e10;
        for(int j = 0; j < model_seq->total; j++)
        {
            cv::Point2f model_point = cv::Point2f(*(CvPoint*)cvGetSeqElem(model_seq, j));
            cv::Point2f model_coords = model_basis.getCoords(model_point);
            float _dist = length(model_coords - test_coords);
            min_dist = MIN(min_dist, _dist*_dist);
        }

        dist += min_dist;//*length(test_coords1 - test_coords);
    }

    cv::Point2f basis[2];
    model_basis.getBasis(basis);
    dist = sqrt(dist/length(basis[0])*length(basis[1]));

    dist /= sqrt(test_seq->total);
    return dist;
}

float fitEdgesSym(CvSeq* model_seq, const AffineBasis& model_basis, CvSeq* test_seq, const AffineBasis& test_basis)
{
    float error1 = fitEdges(model_seq, model_basis, test_seq, test_basis);
    float error2 = fitEdges(test_seq, test_basis, model_seq, model_basis);

    return error1 + error2;
}

CvSeq* mapContour(CvSeq* contour, AffineBasis src, AffineBasis dst, CvMemStorage* storage)
{
    CvSeq* seq = cvCreateSeq(CV_SEQ_POLYGON, sizeof(CvContour), sizeof(CvPoint), storage);
    for(int i = 0; i < contour->total; i++)
    {
        cv::Point2f src_point = *(CvPoint*)cvGetSeqElem(contour, i);
        cv::Point2f coords = src.getCoords(src_point);
        CvPoint dst_point = dst.getPoint(coords);
        cvSeqPush(seq, &dst_point);
    }

    return seq;
}

int PointEdgeMatcher::addModel(const PointEdgeMatcher::Model& model)
{
    models.push_back(model);
    int model_id = models.size() - 1;

    cv::Point2f point = model.first;
    CvSeq* edge = model.second;
    for(int j = 0; j < edge->total; j++)
    {
        AffineBasis basis = getPointEdgeBasis(point, edge, j, model_id);
        addModelBasis(edge, j, basis);
    }

    return model_id;
}

AffineBasis PointEdgeMatcher::match(cv::Point2f point, CvSeq* edge, std::map<int,std::pair<int, int> >& votes) const
{
    std::vector<int> basis_votes;
    basis_votes.assign(hash.getBases().size(), 0);

    // choose random basis
    int i = rand()%edge->total;
    AffineBasis basis = getPointEdgeBasis(point, edge, i, -1);
    matchBasis(edge, basis, i, basis_votes);

    aggregateVotes(basis_votes, votes);

    return basis;
}

void mapPoints(const std::vector<cv::Point2f>& src, const AffineBasis& src_basis, const AffineBasis& dst_basis, std::vector<cv::Point2f>& dst)
{
    dst.clear();
    for(size_t i = 0; i < src.size(); i++)
    {
        dst.push_back(dst_basis.getPoint(src_basis.getCoords(src[i])));
    }
}

void mapPoints(const std::vector<KeyPointEx>& src, const AffineBasis& src_basis, const AffineBasis& dst_basis, std::vector<KeyPointEx>& dst)
{
    dst.clear();
    for(size_t i = 0; i < src.size(); i++)
    {
        dst.push_back(KeyPointEx(dst_basis.getPoint(src_basis.getCoords(src[i].pt))));
    }
}

float fitPoints(const std::vector<cv::Point2f>& set1, const std::vector<cv::Point2f>& set2)
{
    float dist = 0;
    for(size_t i = 0; i < set1.size(); i++)
    {
        float min_dist = 1e10;
        for(size_t j = 0; j < set2.size(); j++)
        {
            float _dist = length(set1[i] - set2[j]);
//            min_dist = MIN(min_dist, _dist*_dist);
            if(_dist*_dist < min_dist)
            {
                min_dist = _dist*_dist;
            }
        }

        dist += min_dist;
    }

    dist = sqrt(dist/set1.size());

    return dist;
}

float fitPointsSym(const std::vector<cv::Point2f>& set1, const std::vector<cv::Point2f>& set2)
{
    float error1 = fitPoints(set1, set2);
    float error2 = fitPoints(set2, set1);

    return error1 + error2;
}

/* ******************************************************************************************* */


int PointMatcher::addModel(const std::vector<KeyPointEx>& points)
{
    template_points = points;

    for(size_t idx1 = 0; idx1 < points.size(); idx1++)
    {
        if(points[idx1].class_id < 0) continue;

        for(size_t idx2 = 0; idx2 < points.size(); idx2++)
        {
            if(points[idx2].class_id < 0) continue;

            if(idx1 == idx2) continue;
            float dist = length(points[idx1].pt - points[idx2].pt);
            if(dist > params.max_basis_length || dist < params.min_basis_length) continue;

            for(size_t idx3 = 0; idx3 < points.size(); idx3++)
            {
                if(points[idx2].class_id < 0) continue;

                if(idx3 == idx1 || idx3 == idx2) continue;
                float dist1 = length(points[idx3].pt - points[idx1].pt);
                float dist2 = length(points[idx3].pt - points[idx2].pt);
                if(dist1 > params.max_basis_length || dist1 < params.min_basis_length ||
                   dist2 > params.max_basis_length || dist2 < params.min_basis_length) continue;

                AffineBasis basis(points[idx1].pt, points[idx2].pt, points[idx3].pt, -1);
                if(basis.getAngle() < params.min_angle || fabs(basis.getAngle() - PI) < params.min_angle) continue;

                addModelBasis(points, basis);
            }
        }
    }

    printf("Added %d bases\n", hash.getBases().size());

    return 0;
}

void PointMatcher::addModelBasis(const std::vector<KeyPointEx>& points, const AffineBasis& basis)
{
    int basis_id = hash.addBasis(basis);
    for(size_t i = 0; i < points.size(); i++)
    {
        hash.addEntry(basis_id, points[i].pt);
    }
}

int PointMatcher::match(const std::vector<KeyPointEx>& points, std::vector<float>& votes,
                        std::vector<std::pair<AffineBasis,AffineBasis> >& matched_bases) const
{
    std::vector<int> basis_votes;

#if defined(_VERBOSE)
    int votes_hist[] = {0, 0, 0, 0, 0, 0};
#endif //_VERBOSE

    for(size_t idx1 = 0; idx1 < points.size(); idx1++)
    {
        if(points[idx1].class_id < 0) continue;

        int w = 0;
        if(abs(points[idx1].pt.x - 269) < 10 && abs(points[idx1].pt.y - 145) < 10)
        {
            w = 1;
        }

        std::vector<int> proximity_indices;
        getProximityPoints(points, points[idx1], params.max_basis_length, proximity_indices);

        size_t idx2 = 0;
        int k = 0;
        for(; k < 10; k++)
        {
            idx2 = proximity_indices[rand()%proximity_indices.size()];
            assert(points[idx2].class_id >= 0);
            if(idx1 == idx2) continue;
            float dist = length(points[idx1].pt - points[idx2].pt);
            if(dist > params.max_basis_length || dist < params.min_basis_length) continue;
            break;
        }
        if(k == 10) continue;

#if defined(_VERBOSE)
        if(w == 1)
        {
            printf("checkpoint 1\n");
        }
#endif //_VERBOSE

        size_t idx3 = 0;
        k = 0;
        for(; k < 10; k++)
        {
            idx3 = proximity_indices[rand()%proximity_indices.size()];
            assert(points[idx3].class_id >= 0);
            if(idx3 == idx1 || idx3 == idx2) continue;
            float dist1 = length(points[idx3].pt - points[idx1].pt);
            float dist2 = length(points[idx3].pt - points[idx2].pt);
            if(dist1 > params.max_basis_length || dist1 < params.min_basis_length ||
               dist2 > params.max_basis_length || dist2 < params.min_basis_length) continue;

            AffineBasis _basis(points[idx1].pt, points[idx2].pt, points[idx3].pt, -1);
            double angle = _basis.getAngle();
            if(angle < params.min_angle || fabs(angle - PI) < params.min_angle || cvIsNaN(angle)) continue;

            break;
        }

        if(k == 10) continue;

#if defined(_VERBOSE)
        if(w == 1)
        {
            printf("checkpoint 2\n");
        }
#endif //_VERBOSE

        AffineBasis basis(points[idx1].pt, points[idx2].pt, points[idx3].pt, -1);
        double angle = basis.getAngle();
        assert(angle >= params.min_angle && fabs(angle - PI) >= params.min_angle && !cvIsNaN(angle));

#if defined(_VERBOSE)
        if(w == 1)
        {
            printf("checkpoint 3\n");
        }
#endif //_VERBOSE

        basis_votes.assign(hash.getBases().size(), 0);
        matchBasis(points, basis, basis_votes);


        for(size_t votes_idx = 0; votes_idx < basis_votes.size(); votes_idx++)
        {
#if defined(_VERBOSE)
            if(basis_votes[votes_idx] >= 0 && basis_votes[votes_idx] < 6)
            {
                votes_hist[basis_votes[votes_idx]]++;
            }
#endif //_VERBOSE

            if(basis_votes[votes_idx] < params.min_hash_votes) continue;
            AffineBasis template_basis = getBasis(votes_idx);
            cv::Point2f origin = template_basis.getOrigin();
            cv::Point2f vec[2];
            template_basis.getBasis(vec);
            vec[0] += origin;
            vec[1] += origin;
            if(length(origin - cv::Point2f(56, 53)) < 10 && length(vec[0] - cv::Point2f(51, 72)) < 10 &&
               length(vec[1] - cv::Point2f(106, 80)) < 10)
            {
                int w = 1;
            }

            float validated_votes = validatePointMatch(template_points, template_basis, points, basis);
            float distortion_ratio = affineDistortionRatio(template_basis, basis);
//            printf("basis origin %f,%f,%f, vec1 %f,%f,%f, vec2 %f,%f,%f, votes %f\n",
            if(validated_votes >= params.min_validated_votes && distortion_ratio > params.min_distortion_ratio)
            {
#if defined(_VERBOSE)
                printf("ratio = %f\n", distortion_ratio);
#endif
                std::pair<AffineBasis,AffineBasis> matched_basis(template_basis, basis);
                votes.push_back(validated_votes);
                matched_bases.push_back(matched_basis);
            }
        }

    }

#if defined(_VERBOSE)
    printf("Votes:");
    for(int i = 0; i < 6; i++)
    {
        printf(" %d", votes_hist[i]);
    }
    printf("\n");
#endif

//    printf("Found %d bases\n", matched_bases.size());

    return 0;
}

void PointMatcher::matchBasis(const std::vector<KeyPointEx>& points, const AffineBasis& basis, std::vector<int>& votes) const
{
    for(size_t i = 0; i < points.size(); i++)
    {
        cv::Point2f point = points[i].pt;
        cv::Point2f coords = basis.getCoords(point);

        const std::list<ModelBasisID>& entries = hash.getEntries(coords);
        for(std::list<ModelBasisID>::const_iterator it = entries.begin(); it != entries.end(); it++)
        {
            votes[*it]++;
        }
    }
}

size_t findClosestPoint(const std::vector<KeyPointEx>& set, KeyPointEx point, bool use_class_id = true)
{
    float min_dist = 1e10;
    size_t min_idx = -1;
    for(size_t i = 0; i < set.size(); i++)
    {
        if(use_class_id && set[i].class_id != point.class_id) continue;

        float dist = length(set[i].pt - point.pt);
        if(dist < min_dist)
        {
            min_dist = dist;
            min_idx = i;
        }
    }

    return min_idx;
}

void findClosestPoint(const std::vector<KeyPointEx>& guess, const std::vector<KeyPointEx>& candidates, std::vector<KeyPointEx>& output, std::vector<bool>& is_detected, float max_dist)
{
    output.resize(guess.size());
    is_detected.resize(guess.size());
    for(size_t i = 0; i < guess.size(); i++)
    {
        size_t idx = findClosestPoint(candidates, guess[i], false);
        KeyPointEx candidate = candidates[idx];
        float dist = length(guess[i].pt - candidate.pt);
        if(dist < max_dist)
        {
            output[i] = candidate;
            is_detected[i] = true;
        }
        else
        {
            output[i] = guess[i];
            is_detected[i] = false;
        }
    }
}

float validatePointMatch(const std::vector<KeyPointEx>& set1, const AffineBasis& basis1,
                        const std::vector<KeyPointEx>& set2, const AffineBasis& basis2)
{
    const float min_dist = 5.0;
    int votes = 0;
    for(size_t idx1 = 0; idx1 < set1.size(); idx1++)
    {
        // project the point
        KeyPointEx p1_projected = KeyPointEx(basis2.getPoint(basis1.getCoords(set1[idx1].pt)), set1[idx1].size, set1[idx1].class_id);

        // find the closest point
        bool use_class_id = true;
#if defined(_WITHOUT_CLASS_ID)
        use_class_id = false;
#endif //_WITHOUT_CLASS_ID
        int idx2 = findClosestPoint(set2, KeyPointEx(p1_projected), use_class_id);
        if(idx2 < 0) continue;
        float dist = length(set2[idx2].pt - p1_projected.pt);
        if(dist < min_dist) votes++;
    }

    return float(votes);
}

void getProximityPoints(const std::vector<KeyPointEx>& points, KeyPointEx point, float max_dist, std::vector<int>& indices)
{
    for(size_t i = 0; i < points.size(); i++)
    {
//        if(points[i].class_id != point.class_id) continue; I don't know why I did that!
        if(points[i].class_id < 0) continue;

        if(length(points[i].pt - point.pt) < max_dist)
        {
            indices.push_back(i);
        }
    }
}

double affineDistortionRatio(const AffineBasis& basis1, const AffineBasis& basis2)
{
    cv::Point2f _basis1[2], _basis2[2];
    basis1.getBasis(_basis1);
    basis2.getBasis(_basis2);
    double basis1_l1 = cv::norm(_basis1[0]);
    double basis1_l2 = cv::norm(_basis1[1]);
    double basis2_l1 = cv::norm(_basis2[0]);
    double basis2_l2 = cv::norm(_basis2[1]);

    double ratio1 = basis1_l1/basis1_l2;
    double ratio2 = basis2_l1/basis2_l2;
    double ratio = ratio1 < ratio2 ? ratio1/ratio2 : ratio2/ratio1;

    return ratio;
}
