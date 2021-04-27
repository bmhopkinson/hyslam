/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/ modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <FeatureMatcher.h>

#include <FeatureViews.h>
#include <FeatureExtractorSettings.h>
#include <GenUtils.h>
#include <Camera.h>
#include <Map.h>  //long term - eliminate this dependency
#include <MapPointDB.h>//long term - eliminate this dependency

#include <opencv2/core/core.hpp>
#include "DBoW2/FeatureVector.h"

#include <algorithm>
#include <iterator>
#include<stdint-gcc.h>
#include <limits.h>

using namespace std;

namespace HYSLAM
{

const int FeatureMatcher::HISTO_LENGTH = 30;

FeatureMatcher::FeatureMatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}


FeatureMatcher::FeatureMatcher(FeatureMatcherSettings settings){
    mfNNratio = settings.nnratio;
    mbCheckOrientation = settings.checkOri;
    TH_LOW = settings.TH_LOW;
    TH_HIGH = settings.TH_HIGH;
}

int FeatureMatcher::_SearchByProjection_(Frame &frame, const std::vector<MapPoint*> &landmarks, const float th,
                                         std::vector< std::unique_ptr<LandMarkCriterion> >     &landmark_criteria,
                                         std::vector< std::unique_ptr<LandMarkViewCriterion> > &landmarkview_criteria,
                                         std::vector< std::unique_ptr<GlobalCriterion> >       &global_criteria,
                                         CriteriaData &criteria_data
                         ){

    std::map<MapPoint*, SingleMatchData> matches;
    std::vector<MapPoint*> cand_lms = landmarks;

    const FeatureViews &views = frame.getViews();
    FeatureExtractorSettings orb_params = views.orbParams();

    //apply landmark criteria

    for(auto criterion = landmark_criteria.begin(); criterion != landmark_criteria.end(); ++criterion){//winnow down potential landmark matches
        cand_lms = (*criterion)->apply(frame, cand_lms, criteria_data);
    }

    //apply keypoint criteria to all passing landmarks
    int nn = 0;
    for(auto it = cand_lms.begin(); it != cand_lms.end(); ++it ){

        MapPoint* lm = *it;
        if(!lm){
            continue;
        }
        nn++;

        cv::Mat uv;
        frame.ProjectLandMark(lm, uv);
        float u = uv.at<float>(0,0);
        float v = uv.at<float>(1,0);
        float ur = uv.at<float>(2,0);
        float radius = th * frame.landMarkSizePixels(lm)/orb_params.size_ref;
        std::vector<size_t> cand_lmviews = frame.GetFeaturesInAreaNEW(u,v,radius);

        for(auto criterion  = landmarkview_criteria.begin(); criterion != landmarkview_criteria.end(); ++criterion ){ //winnow down potential views
            cand_lmviews = (*criterion)->apply(frame, lm, cand_lmviews, criteria_data);
        }

        if(!cand_lmviews.empty()){  //at this point should have match or not
            size_t idx = cand_lmviews[0];
            SingleMatchData match_data;
            match_data.idx = idx;
            match_data.distance = criteria_data.getBestScore();
            matches.insert(std::make_pair(lm, match_data) );
        }
    }
   // std::cout << "total cand_views checked: " << nn << std::endl;

    //APPLY GLOBAL CRITERIA and remove matches that don't pass from "matches"
    for(auto criterion  = global_criteria.begin(); criterion != global_criteria.end(); ++criterion ){
        matches = (*criterion)->apply(matches, frame, criteria_data);
    }
 //probably better to return these matches and let the calling function do the association if it wants to
    for(auto it = matches.begin(); it != matches.end(); ++it){
        MapPoint* lm = it->first;
        SingleMatchData match_data = it->second;
        size_t idx = match_data.idx;
        frame.associateLandMark(idx, lm, true);
    }

    return matches.size();
}

int FeatureMatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th)
{
    CriteriaData criteria_data;
    std::vector< std::unique_ptr<LandMarkCriterion> >     landmark_criteria;
    landmark_criteria.push_back( std::make_unique<ProjectionCriterion>() );
    landmark_criteria.push_back( std::make_unique<DistanceCriterion>() );

    std::vector< std::unique_ptr<LandMarkViewCriterion> > landmarkview_criteria;
    landmarkview_criteria.push_back( std::make_unique<PreviouslyMatchedCriterion>() );
    landmarkview_criteria.push_back( std::make_unique<FeatureSizeCriterion>(0.5,1.5) );
    landmarkview_criteria.push_back( std::make_unique<StereoConsistencyCriterion>(th) );
    landmarkview_criteria.push_back( std::make_unique<BestScoreCriterion>( TH_HIGH ,mfNNratio) );

    std::vector< std::unique_ptr<GlobalCriterion> >       global_criteria;

    int nmatches = _SearchByProjection_(F, vpMapPoints, th,
                                           landmark_criteria, landmarkview_criteria, global_criteria, criteria_data );

    return nmatches;

}

int FeatureMatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
{
    CriteriaData criteria_data;
    Frame last_frame_copy = LastFrame;
    criteria_data.setPreviousFrame(&last_frame_copy);

    std::vector<MapPoint*> vpMapPoints = LastFrame.replicatemvpMapPoints();

    int n_mpts = 0;
    for(auto it = vpMapPoints.begin(); it != vpMapPoints.end(); ++it){
        if(*it){
            n_mpts++;
        }
    }

    std::vector< std::unique_ptr<LandMarkCriterion> >     landmark_criteria;
    landmark_criteria.push_back( std::make_unique<ProjectionCriterion>() );
   // landmark_criteria.push_back( std::make_unique<DistanceCriterion>() );

    std::vector< std::unique_ptr<LandMarkViewCriterion> > landmarkview_criteria;
    landmarkview_criteria.push_back( std::make_unique<PreviouslyMatchedCriterion>() );
    landmarkview_criteria.push_back( std::make_unique<FeatureSizeCriterion>(0.5,1.5) );
    landmarkview_criteria.push_back( std::make_unique<StereoConsistencyCriterion>(th) );
    landmarkview_criteria.push_back( std::make_unique<BestScoreCriterion>( TH_HIGH ,mfNNratio) );

    std::vector< std::unique_ptr<GlobalCriterion> >       global_criteria;
    global_criteria.push_back( std::make_unique<RotationConsistencyCriterion>() );
  //  global_criteria.push_back( std::make_unique<GlobalBestScoreCriterion>(false) );//in practice this didn't seem necessary

    int nmatches = _SearchByProjection_(CurrentFrame, vpMapPoints, th,
                                         landmark_criteria, landmarkview_criteria, global_criteria, criteria_data );

    return nmatches;
}



int FeatureMatcher::SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist){
    CriteriaData criteria_data;

    std::vector<MapPoint*> landmarks_pKF = pKF->getAssociatedLandMarks();

    //eliminate landmarks already found
    std::vector<MapPoint*> landmarks_previously_found(sAlreadyFound.begin(), sAlreadyFound.end());
    std::sort(landmarks_pKF.begin(), landmarks_pKF.end());
    std::sort(landmarks_previously_found.begin(), landmarks_previously_found.end());
    std::vector<MapPoint*> vpMapPoints;
    std::set_difference(landmarks_pKF.begin(), landmarks_pKF.end(),
                        landmarks_previously_found.begin(), landmarks_previously_found.end(),
                        std::inserter(vpMapPoints, vpMapPoints.begin())
    );


    std::vector< std::unique_ptr<LandMarkCriterion> >     landmark_criteria;
    landmark_criteria.push_back( std::make_unique<ProjectionCriterion>() );
    landmark_criteria.push_back( std::make_unique<DistanceCriterion>() );

    std::vector< std::unique_ptr<LandMarkViewCriterion> > landmarkview_criteria;
    landmarkview_criteria.push_back( std::make_unique<PreviouslyMatchedCriterion>() );
    landmarkview_criteria.push_back( std::make_unique<FeatureSizeCriterion>(0.5,1.5) );
    //   landmarkview_criteria.push_back( std::make_unique<StereoConsistencyCriterion>(th) );
    landmarkview_criteria.push_back( std::make_unique<BestScoreCriterion>( ORBdist ,1.00) ); //don't impose constraint that best is substantially better than 2nd best

    std::vector< std::unique_ptr<GlobalCriterion> >       global_criteria;
    global_criteria.push_back( std::make_unique<RotationConsistencyCriterion>() );

    int nmatches = _SearchByProjection_(CurrentFrame, vpMapPoints, th,
                                         landmark_criteria, landmarkview_criteria, global_criteria, criteria_data );

    return nmatches;
}

//can't yet convert this to using _SearchByBoW_ b/c Frame and KeyFrame don't have common base class - it would make sense to derive them both from a common AbstractFrame class
//int FeatureMatcher::SearchByBoW2(KeyFrame* pKF,Frame &F, std::vector<MapPoint*> &vpMapPointMatches)
int FeatureMatcher::SearchByBoW(KeyFrame *pKF, Frame &F, std::map<size_t, MapPoint*> &matches)
{
    std::map<size_t, size_t> matches_internal;

    const FeatureViews Fviews = F.getViews();
    const FeatureViews KFviews = pKF->getViews();

    const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;
    // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
    DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
    DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
    DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
    DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

    while(KFit != KFend && Fit != Fend)
    {
        if(KFit->first == Fit->first)
        {
            std::vector<unsigned int> vIndicesKF = KFit->second;
            std::vector<unsigned int> vIndicesF = Fit->second;

            PreviouslyMatchedIndexCriterion previously_matched(true);
            vIndicesKF = previously_matched.apply(pKF, vIndicesKF);

            for(auto it = vIndicesKF.begin(); it != vIndicesKF.end(); ++it){
                size_t idx1 = *it;
                std::vector<unsigned int> vIndicesF_copy = vIndicesF;
                BestMatchBoWCriterion  best_match(TH_LOW, mfNNratio);

                vIndicesF_copy = best_match.apply(idx1, vIndicesF_copy, KFviews, Fviews);

                if(!vIndicesF_copy.empty()){  //at this point should have match or not
                    size_t idx2 = vIndicesF_copy[0];
                    matches_internal.insert(std::make_pair(idx1, idx2) );
                }
            }


            KFit++;
            Fit++;
        }
        else if(KFit->first < Fit->first)
        {
            KFit = vFeatVecKF.lower_bound(Fit->first);
        }
        else
        {
            Fit = F.mFeatVec.lower_bound(KFit->first);
        }
    }
    RotationConsistencyBoW rotation_consistency;
    matches_internal = rotation_consistency.apply(matches_internal, KFviews, Fviews);

    for(auto it = matches_internal.begin(); it != matches_internal.end(); ++it){
        size_t idx_kf = it->first;
        size_t idx_f  = it->second;
        MapPoint* lm = pKF->hasAssociation(idx_kf);
        matches[idx_f]  = lm;
    }


    return matches_internal.size();
}


int FeatureMatcher:: _SearchByBoW_(KeyFrame* pKF1, KeyFrame* pKF2, std::map<size_t, size_t> &matches,
                                   std::vector< std::unique_ptr<BoWIndexCriterion> > &index_criteria,
                                   std::vector< std::unique_ptr<BoWMatchCriterion> > &match_criteria,
                                   std::vector< std::unique_ptr<RotationConsistencyBoW> > &global_criteria
)
{
    std::map<size_t, size_t> matches_internal;

    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    const FeatureViews views1 = pKF1->getViews();
    const FeatureViews views2 = pKF2->getViews();

    while(f1it!=f1end && f2it!=f2end)
    {
        if(f1it->first == f2it->first)
        {

           std::vector<unsigned int> indices1 = f1it->second;
           std::vector<unsigned int> indices2 = f2it->second;
            for(auto criterion  = index_criteria.begin(); criterion != index_criteria.end(); ++criterion){
                indices1 = (*criterion)->apply( pKF1, indices1);
                indices2 = (*criterion)->apply( pKF2, indices2);
            }

            for(auto it = indices1.begin(); it != indices1.end(); ++it){
                size_t idx1 = *it;
                std::vector<unsigned int> indices2_copy = indices2;
                for(auto criterion = match_criteria.begin(); criterion != match_criteria.end(); ++criterion){
                    indices2_copy = (*criterion)->apply(idx1, indices2_copy, views1, views2);
                }

                if(!indices2_copy.empty()){  //at this point should have match or not
                    size_t idx2 = indices2_copy[0];
                    matches_internal.insert(std::make_pair(idx1, idx2) );
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    // APPLY GLOBAL CRITERIA
    for(auto criterion = global_criteria.begin(); criterion != global_criteria.end(); ++ criterion){
        matches_internal = (*criterion)->apply(matches_internal, views1, views2);
    }
    matches = matches_internal;

    return matches.size();

}
int FeatureMatcher::SearchByBoW2(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12) {
    std::vector< std::unique_ptr<BoWIndexCriterion> > index_criteria;
    index_criteria.push_back(std::make_unique<PreviouslyMatchedIndexCriterion>(true));

    std::vector< std::unique_ptr<BoWMatchCriterion> > match_criteria;
    match_criteria.push_back(std::make_unique<BestMatchBoWCriterion>(TH_LOW, mfNNratio));

    std::vector< std::unique_ptr<RotationConsistencyBoW> > global_criteria; //right now rotation is the only global criteria so this is a bit silly but formated for extension
    global_criteria.push_back(std::make_unique<RotationConsistencyBoW>());

    std::map<size_t, size_t> matches;
    int nmatches = _SearchByBoW_(pKF1, pKF2, matches, index_criteria, match_criteria, global_criteria);

    //rearrange output for expected form
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    vpMatches12 = std::vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    for(int i = 0; i < vpMatches12.size(); ++i){
        auto it = matches.find(i);
        if(it != matches.end()){
            MapPoint* lm = pKF2->hasAssociation(it->second);
            vpMatches12[i] = lm;
        }
    }

    return matches.size();
}

int FeatureMatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                                           std::vector<std::pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo){

    std::vector< std::unique_ptr<BoWIndexCriterion> > index_criteria;
    index_criteria.push_back(std::make_unique<PreviouslyMatchedIndexCriterion>(false));
    if(bOnlyStereo) {
        index_criteria.push_back(std::make_unique<StereoIndexCriterion>());
    }

    //Compute epipole in second image
    float ex, ey;
    GenUtils::Epipole(pKF1, pKF2, ex, ey);

    std::vector< std::unique_ptr<BoWMatchCriterion> > match_criteria;
    match_criteria.push_back(std::make_unique<EpipolarConsistencyBoWCriterion>(F12,ex, ey));
    match_criteria.push_back(std::make_unique<BestMatchBoWCriterion>(TH_LOW, 1.000));

    std::vector< std::unique_ptr<RotationConsistencyBoW> > global_criteria; //right now rotation is the only global criteria so this is a bit silly but formated for extension
    global_criteria.push_back(std::make_unique<RotationConsistencyBoW>());

    std::map<size_t, size_t> matches;
    int nmatches = _SearchByBoW_(pKF1, pKF2, matches, index_criteria, match_criteria, global_criteria);

    //rearrange output for expected form
    for(auto it = matches.begin(); it != matches.end(); ++it){
        vMatchedPairs.push_back(std::make_pair(it->first, it->second));
    }

    return matches.size();
}

int FeatureMatcher::SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    std::map<size_t, size_t> matches; // idx in frame 2 is key, matching index in frame 1 is value
    const FeatureViews F1views = F1.getViews();
    const FeatureViews F2views = F2.getViews();

    std::vector< std::unique_ptr<MonoInitViewCriterion> > landmarkview_criteria;
    landmarkview_criteria.push_back( std::make_unique<MonoInitScoreExceedsPrevious>() );
    landmarkview_criteria.push_back( std::make_unique<MonoInitBestScore>(TH_LOW,mfNNratio ) );

    MonoCriteriaData criteria_data;

    for(size_t i1=0, iend1=F1views.numViews(); i1<iend1; i1++) {
        cv::KeyPoint kp1 = F1views.keypt(i1);

        vector<size_t> cand_matches = F2.GetFeaturesInArea(vbPrevMatched[i1].x, vbPrevMatched[i1].y, windowSize);
        if( cand_matches.empty() ){
            continue;
        }

        criteria_data.setIndex(i1);
        criteria_data.setDescriptor(F1views.descriptor(i1));

        for(auto criterion  = landmarkview_criteria.begin(); criterion != landmarkview_criteria.end(); ++criterion ){ //winnow down potential views
            cand_matches = (*criterion)->apply( cand_matches, F2views, criteria_data);
        }

        if(!cand_matches.empty()){  //at this point should have match or not
            size_t idx2 = cand_matches[0];
            matches[idx2] = i1;  //note: this will overwrite any old match
            FeatureDescriptor i1_desc = F1views.descriptor(i1);
            float bestDist = i1_desc.distance(F2views.descriptor(idx2));
            criteria_data.setDistance(idx2, bestDist);
        }
    }

    const FeatureViews& views2  = F2.getViews();
    const FeatureViews& views1  = F1.getViews();
    matches = RotationConsistency(matches, views2, views1 );

    // output data: vnMatches12 matches from frame 1 to frame 2 and previously matched keypt data

    std::map<size_t, size_t> matches_inverse;
    for(auto it = matches.begin(); it != matches.end(); ++it){
        matches_inverse[it->second] = it->first;
    }
    vnMatches12 = vector<int>(F1views.numViews(),-1);
    for(int i1 = 0 ; i1 < vnMatches12.size(); i1++){
        auto mit = matches_inverse.find(i1);
        if(mit != matches_inverse.end()){  //there is a match to this keypoint
            size_t idx2 = mit->second;
            vnMatches12[i1] = idx2;
            vbPrevMatched[i1]=F2views.keypt(idx2).pt;
        }
    }

    return matches.size();

}

int FeatureMatcher::Fuse(KeyFrame *pKF, const std::vector<MapPoint *> &vpMapPoints, std::map<std::size_t, MapPoint*> &fuse_matches, const float th, const float reprojection_err) {

    std::vector< std::unique_ptr<LandMarkCriterion> >     landmark_criteria;
    landmark_criteria.push_back( std::make_unique<ProjectionCriterion>() );
    landmark_criteria.push_back( std::make_unique<DistanceCriterion>() );
    landmark_criteria.push_back( std::make_unique<ViewingAngleCriterion>(1.047) );

    std::vector< std::unique_ptr<LandMarkViewCriterion> >     landmarkview_criteria;
    landmarkview_criteria.push_back( std::make_unique<FeatureSizeCriterion>(0.5,1.5) );
    landmarkview_criteria.push_back( std::make_unique<ProjectionViewCriterion>(reprojection_err) );
    landmarkview_criteria.push_back( std::make_unique<BestScoreCriterion>(TH_LOW ,1.000) );

    const FeatureViews views = pKF->getViews();
    FeatureExtractorSettings orb_params = views.orbParams();
    CriteriaData criteria_data;

    std::vector<MapPoint*> cand_lms = vpMapPoints;
    //pre-screen landmarks
    auto new_first = std::remove_if(cand_lms.begin(), cand_lms.end(),
                                    [pKF](MapPoint* lm) {
                                        if(!lm){return true;}
                                        return (lm->isBad() || lm->IsInKeyFrame(pKF) || lm->Protected());
                                    });
    cand_lms.erase(new_first, cand_lms.end());

   // std::cout << "Fuser: candidates passing pre_screen: " << cand_lms.size() << std::endl;

    //apply landmark criteria
    for(auto criterion = landmark_criteria.begin(); criterion != landmark_criteria.end(); ++criterion){//winnow down potential landmark matches
        cand_lms = (*criterion)->apply(pKF, cand_lms, criteria_data);
    }
  // std::cout << "post landmark_criteria: " << cand_lms.size() << std::endl;
    //apply keypoint criteria to all passing landmarks
    for(auto it = cand_lms.begin(); it != cand_lms.end(); ++it ) {
        MapPoint* lm = *it;
        if(!lm){
            continue;
        }

        cv::Mat uv;
        pKF->ProjectLandMark(lm, uv);
        float u = uv.at<float>(0,0);
        float v = uv.at<float>(1,0);
        float ur = uv.at<float>(2,0);
        float radius = th * pKF->landMarkSizePixels(lm)/orb_params.size_ref;
        std::vector<size_t> cand_lmviews = pKF->GetFeaturesInArea(u,v,radius);


        for(auto criterion  = landmarkview_criteria.begin(); criterion != landmarkview_criteria.end(); ++criterion ){ //winnow down potential views
            cand_lmviews = (*criterion)->apply(pKF, lm, cand_lmviews, criteria_data);
        }

       // std::cout << "n_lmviews post criteria: " << cand_lmviews.size() << std::endl;
        if(!cand_lmviews.empty()){  //at this point should have match or not
            size_t idx = cand_lmviews[0];
            fuse_matches.insert(std::make_pair(idx, lm) );
        }
    }

    return fuse_matches.size();
}

int FeatureMatcher::Fuse(KeyFrame *pKF, cv::Mat Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)
{
    const FeatureViews views = pKF->getViews();
    FeatureExtractorSettings orb_params = views.orbParams();
    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    const set<MapPoint*> spAlreadyFound = pKF->GetMapPoints();

    int nFused=0;

    const int nPoints = vpPoints.size();

    // For each candidate MapPoint project and match
    for(int iMP=0; iMP<nPoints; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        cv::Mat uv_ur;
        if(!pKF->ProjectLandMark(pMP, uv_ur)){
            continue;
        }
        float u, v, ur;
        u  = uv_ur.at<float>(0,0);
        v  = uv_ur.at<float>(1,0);
        ur = uv_ur.at<float>(2,0);

        // Depth must be inside the scale pyramid of the image
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist3D = cv::norm(PO);

        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist3D)
            continue;

        float radius = th * pKF->landMarkSizePixels(pMP)/orb_params.size_ref;

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius

        const FeatureDescriptor dMP = pMP->GetDescriptor();

        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(); vit!=vIndices.end(); vit++)
        {
            const size_t idx = *vit;
            const FeatureDescriptor &dKF = views.descriptor(idx);

            float dist = dMP.distance(dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if(bestDist<=TH_LOW)
        {
            MapPoint* pMPinKF = pKF->hasAssociation(bestIdx);
            if(pMPinKF)
            {
                if(!pMPinKF->isBad())
                    vpReplacePoint[iMP] = pMPinKF;
            }
            else
            {
                pKF->getMap()->addAssociation(pKF, bestIdx, pMP, true);
            }
            nFused++;
        }
    }

    return nFused;
}


//BELOW ARE LEGACY FUNCTIONS MOSTLY USED IN LOOP CLOSING THAT I"D LIKE TO give a more abstracted logical form
int FeatureMatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)
{
    // Get Calibration Parameters for later projection
    const Camera camera = pKF->getCamera();
    const float &fx = camera.fx();
    const float &fy = camera.fy();
    const float &cx = camera.cx();
    const float &cy = camera.cy();

    const FeatureViews KFviews = pKF->getViews(); //this could get large convert to const ref eventually
    FeatureExtractorSettings orb_params = KFviews.orbParams();

    // Decompose Scw
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;

    // Set of MapPoints already found in the KeyFrame
    set<MapPoint*> spAlreadyFound(vpMatched.begin(), vpMatched.end());
    spAlreadyFound.erase(static_cast<MapPoint*>(NULL));

    int nmatches=0;

    // For each Candidate MapPoint Project and Match
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++)
    {
        MapPoint* pMP = vpPoints[iMP];

        // Discard Bad MapPoints and already found
        if(pMP->isBad() || spAlreadyFound.count(pMP))
            continue;

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw*p3Dw+tcw;

        // Depth must be positive
        if(p3Dc.at<float>(2)<0.0)
            continue;

        // Project into Image
        const float invz = 1/p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0)*invz;
        const float y = p3Dc.at<float>(1)*invz;

        const float u = fx*x+cx;
        const float v = fy*y+cy;

        // Point must be inside the image
        if(!pKF->IsInImage(u,v))
            continue;

        // Depth must be inside the scale invariance region of the point
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        cv::Mat PO = p3Dw-Ow;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            continue;

        // Viewing angle must be less than 60 deg
        cv::Mat Pn = pMP->GetNormal();

        if(PO.dot(Pn)<0.5*dist)
            continue;

        float radius = th * pKF->landMarkSizePixels(pMP)/orb_params.size_ref;

        const vector<size_t> vIndices = pKF->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const FeatureDescriptor dMP = pMP->GetDescriptor();

        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            if(vpMatched[idx])
                continue;

            const FeatureDescriptor &dKF = KFviews.descriptor(idx);

            const float dist = dMP.distance(dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_LOW)
        {
            vpMatched[bestIdx]=pMP;
            nmatches++;
        }

    }

    return nmatches;
}

int FeatureMatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
                                 const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)
{
    const FeatureViews KF1views = pKF1->getViews();
    FeatureExtractorSettings orb_params_KF1 = KF1views.orbParams();
    const FeatureViews KF2views = pKF2->getViews();
    FeatureExtractorSettings orb_params_KF2 = KF2views.orbParams();

    // Camera 1 from world
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();

    //Camera 2 from world
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    //Transformation between cameras
    cv::Mat sR12 = s12*R12;
    cv::Mat sR21 = (1.0/s12)*R12.t();
    cv::Mat t21 = -sR21*t12;

    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const int N1 = vpMapPoints1.size();

    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const int N2 = vpMapPoints2.size();

    vector<bool> vbAlreadyMatched1(N1,false);
    vector<bool> vbAlreadyMatched2(N2,false);

    for(int i=0; i<N1; i++)
    {
        MapPoint* pMP = vpMatches12[i];
        if(pMP)
        {
            vbAlreadyMatched1[i]=true;
            int idx2 = pMP->GetIndexInKeyFrame(pKF2);
            if(idx2>=0 && idx2<N2)
                vbAlreadyMatched2[idx2]=true;
        }
    }

    vector<int> vnMatch1(N1,-1);
    vector<int> vnMatch2(N2,-1);

    // Transform from KF1 to KF2 and search
    for(int i1=0; i1<N1; i1++)
    {
        MapPoint* pMP = vpMapPoints1[i1];

        if(!pMP || vbAlreadyMatched1[i1])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos(); //because of scaling need to manually transform point into camera 2's frame
        cv::Mat p3Dc1 = R1w*p3Dw + t1w;
        cv::Mat p3Dc2 = sR21*p3Dc1 + t21;

        cv::Mat uv_ur;
        if( !pKF2->getCamera().Project(p3Dc2, uv_ur) ){
            continue;
        }

        float u, v, ur;
        u  = uv_ur.at<float>(0,0);
        v  = uv_ur.at<float>(1,0);
        ur = uv_ur.at<float>(2,0);

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc2);

        // Depth must be inside the scale invariance region
        if(dist3D<minDistance || dist3D>maxDistance )
            continue;

        float radius = th * pKF2->landMarkSizePixels(pMP)/orb_params_KF2.size_ref;

        const vector<size_t> vIndices = pKF2->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const FeatureDescriptor dMP = pMP->GetDescriptor();

        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            const cv::KeyPoint &kp = KF2views.keypt(idx);
            const FeatureDescriptor &dKF = KF2views.descriptor(idx);
            const float dist = dMP.distance( dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch1[i1]=bestIdx;
        }
    }

    // Transform from KF2 to KF1 and search
    for(int i2=0; i2<N2; i2++)
    {
        MapPoint* pMP = vpMapPoints2[i2];

        if(!pMP || vbAlreadyMatched2[i2])
            continue;

        if(pMP->isBad())
            continue;

        cv::Mat p3Dw = pMP->GetWorldPos();
        cv::Mat p3Dc2 = R2w*p3Dw + t2w;
        cv::Mat p3Dc1 = sR12*p3Dc2 + t12;

        cv::Mat uv_ur;
        if( !pKF1->getCamera().Project(p3Dc1, uv_ur) ){
            continue;
        }

        float u, v, ur;
        u  = uv_ur.at<float>(0,0);
        v  = uv_ur.at<float>(1,0);
        ur = uv_ur.at<float>(2,0);

        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const float dist3D = cv::norm(p3Dc1);

        // Depth must be inside the scale pyramid of the image
        if(dist3D<minDistance || dist3D>maxDistance)
            continue;

        float radius = th * pKF1->landMarkSizePixels(pMP)/orb_params_KF1.size_ref;

        const vector<size_t> vIndices = pKF1->GetFeaturesInArea(u,v,radius);

        if(vIndices.empty())
            continue;

        // Match to the most similar keypoint in the radius
        const FeatureDescriptor dMP = pMP->GetDescriptor();

        float bestDist = std::numeric_limits<float>::max();
        int bestIdx = -1;
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;
            const cv::KeyPoint &kp = KF1views.keypt(idx);
            const FeatureDescriptor &dKF = KF1views.descriptor(idx);

            const float dist = dMP.distance(dKF);

            if(dist<bestDist)
            {
                bestDist = dist;
                bestIdx = idx;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            vnMatch2[i2]=bestIdx;
        }
    }

    // Check agreement
    int nFound = 0;

    for(int i1=0; i1<N1; i1++)
    {
        int idx2 = vnMatch1[i1];

        if(idx2>=0)
        {
            int idx1 = vnMatch2[idx2];
            if(idx1==i1)
            {
                vpMatches12[i1] = vpMapPoints2[idx2];
                nFound++;
            }
        }
    }

    return nFound;
}



int FeatureMatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
//aim to replace this with SearchByBoW2 - but haven't been able to test in LoopClosing yet so preserving until then
{
    const FeatureViews KF1views = pKF1->getViews();
    FeatureExtractorSettings orb_params_KF1 = KF1views.orbParams();
    const FeatureViews KF2views = pKF2->getViews();
    FeatureExtractorSettings orb_params_KF2 = KF2views.orbParams();

    const vector<cv::KeyPoint> vKeysUn1 = KF1views.getKeys();
    const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
    const vector<MapPoint*> vpMapPoints1 = pKF1->GetMapPointMatches();
    const std::vector<FeatureDescriptor> Descriptors1 = KF1views.getDescriptors();

    const vector<cv::KeyPoint> vKeysUn2 = KF2views.getKeys();
    const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
    const vector<MapPoint*> vpMapPoints2 = pKF2->GetMapPointMatches();
    const std::vector<FeatureDescriptor> Descriptors2 = KF2views.getDescriptors();

    vpMatches12 = vector<MapPoint*>(vpMapPoints1.size(),static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(),false);

    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);

    const float factor = 1.0f/HISTO_LENGTH;

    int nmatches = 0;

    DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
    DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
    DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
    DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

    while(f1it != f1end && f2it != f2end)
    {
        if(f1it->first == f2it->first)
        {
            for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = vpMapPoints1[idx1];
                if(!pMP1)
                    continue;
                if(pMP1->isBad())
                    continue;

                const FeatureDescriptor &d1 = Descriptors1[idx1];

                float bestDist1 = std::numeric_limits<float>::max();
                int bestIdx2 =-1 ;
                float bestDist2 = std::numeric_limits<float>::max();

                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if(vbMatched2[idx2] || !pMP2)
                        continue;

                    if(pMP2->isBad())
                        continue;

                    const FeatureDescriptor &d2 = Descriptors2[idx2];

                    float dist = d1.distance(d2);

                    if(dist<bestDist1)
                    {
                        bestDist2=bestDist1;
                        bestDist1=dist;
                        bestIdx2=idx2;
                    }
                    else if(dist<bestDist2)
                    {
                        bestDist2=dist;
                    }
                }

                if(bestDist1<TH_LOW)
                {
                    if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
                    {
                        vpMatches12[idx1]=vpMapPoints2[bestIdx2];
                        vbMatched2[bestIdx2]=true;

                        if(mbCheckOrientation)
                        {
                            float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
                            if(rot<0.0)
                                rot+=360.0f;
                            int bin = round(rot*factor);
                            if(bin==HISTO_LENGTH)
                                bin=0;
                            assert(bin>=0 && bin<HISTO_LENGTH);
                            rotHist[bin].push_back(idx1);
                        }
                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if(f1it->first < f2it->first)
        {
            f1it = vFeatVec1.lower_bound(f2it->first);
        }
        else
        {
            f2it = vFeatVec2.lower_bound(f1it->first);
        }
    }

    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
                nmatches--;
            }
        }
    }

    return nmatches;
}

void FeatureMatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}

} //namespace ORB_SLAM
