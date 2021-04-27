
#ifndef FEATUREMATCHER_H
#define FEATUREMATCHER_H

/*
 * a sprawling class (should be broken up) that matches features to landmarks, or features between images
 * matchers make use of criteria which as successively applied to potential features or landmarks until a candidate match is found.
 * there can also be global criteria applied to a set of candidate matches looking to see that the set follows some criteria (e.g. rotation consistency)
 * algorithms are generic wrt feature type.
 * the constructor takes a FeatureMatcherSetting structure which primarily contains information on the distance thresholds for considering a match valid (TH_LOW, TH_HIGH)
 * nnratio is the "nearest neighbor" ratio and is used for checks in which the best score should be nnratio better than the second best potential match (this value is <1 b/c scores are distances)
 *
 * logically there are multiple types of matching:
 * 1. Projection Matching - SearchByProjection()
 *  _SearchByProjection_(Frame &frame, const std::vector<MapPoint*> &landmarks, const float th,
                             std::vector< std::unique_ptr<LandMarkCriterion> >     &landmark_criteria,
                             std::vector< std::unique_ptr<LandMarkViewCriterion> > &landmarkview_criteria,
                             std::vector< std::unique_ptr<GlobalCriterion> >       &global_criteria,
                             CriteriaData &criteria_data
    );
    general purpose, internal search by projection function. attempts to match landmarks to features in frame by applying:
    first: landmark_criteria (one of which should be that the landmark projects into the frame), for landmarks that pass the criteria, landmarkview_criteria (landmarkviews are features/keypoints)
    are applied to all features in the image neighborhood where the landmark projected. after applying the landmarkview_criteria there should be either a single match or none.
    if there's a match it is saved and subsequently the global_criteria are applied to the set of potential landmark->feature matches.
    for the landmark->feature matches, a new association is registered in the frame (would be more logical to pass the matches back and let some other function make the association)
 *
 *  SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th = 3) - used in TrackLocalMap
 *      attempts to match vpMapPoints to features in frame
 *
 *  SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono) - used in TrackMotionModel
 *      attempts to match landmarks tracked in LastFrame to features in CurrentFrame
 *
 *  SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist) - used in Relocalization
 *      attempts to match landmarks tracked in pKF to features in CurrentFrame, but excludes landmarks in sAlreadyFound
 *
 *  SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th) - used in LoopDetection
 *      LEGACY IMPLEMENTATION - attempts to match landmarks in vpPoints to features in pKF accounting for similarity trasnform (Scw) and excluding vpMatched
 *
 * 2. Bag of Words (BoW) matching - SearchByBoW()
 *  SearchByBoW(KeyFrame *pKF, Frame &F, std::map<size_t, MapPoint*> &matches)
 *  uses DBoW2 feature vectors from pKF and F and identifies features in both that belong to the same vocabulary node.
 *  These are potential matches based on feature similarity. then applies a set of criteria to these potential feature matches to further winnow potential matches
 *  Finally, applies rotation consistency criteria for global consistency among potential matches.
 *  This is essentially a specialized version of what is done below but can't generalize b/c is matching between KeyFrame and Frame (those below are matching between two keyframes)
 *
 *
 *   _SearchByBoW_(KeyFrame* pKF1, KeyFrame* pKF2, std::map<size_t, size_t> &matches,
                      std::vector< std::unique_ptr<BoWIndexCriterion> > &index_criteria,
                      std::vector< std::unique_ptr<BoWMatchCriterion> > &match_criteria,
                      std::vector< std::unique_ptr<RotationConsistencyBoW> > &global_criteria
      );
 *  general purpose, internal search by bag of words function.
 *  uses DBoW2 feature vectors from pKF1 and pK2 and identifies features in both that belong to the same vocabulary node.
 *  These are potential matches based on feature similarity. then applies a set of criteria to 1) indexes (e.g. index already has a landmark match)
 *  2) match_criteria (e.g. best match )  and finally 3) global_criteria (e.g. rotation consistency) to these potential feature matches to further winnow matches
 *
 *  SearchByBoW2(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12)
 *  finds matches between pKF1 and pK2 based on feature similarity (as assessed by membership in the same feature vocabulary node) uses _SeachByBow_()
 *
 *  SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12); LEGACY  - aim to replace this with SearchByBoW2 - but haven't been able to test in LoopClosing yet so preserving until then
 *
 *  SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<std::pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo)
    uses _SearchByBoW_() to triangulate new LandMarks observed in pKF1 and pKF2, makes use of fundamental matrix (F12) to ensure BoW matches fall on epipolar lines and so are geometrically valid
 *
 *  3. Fuse - functions used for finding landmarks to fuse (reducing size of map and providing greater connectivity between keyframes) - also reports new feature->landmark matches
 *  Fuse(KeyFrame* pKF, const std::vector<MapPoint *> &vpMapPoints, std::map<std::size_t, MapPoint*> &fuse_matches, const float th=3.0, const float reprojection_err = 5.99);
 *    attempts to find landmarks for fusion (output: fuse_matches) from candidate landmarks in vpMapPoints with landmarks viewed in pKF.
 *    for each candidate in vpMapPoints, projects into pKF and gets features in area then applies a series of criteria to see if any of these features should
 *    be fused (or new matches)
 *
 *   Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, std::vector<MapPoint *> &vpReplacePoint);
 *    LEGACY for Loop Closing: Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
 *
 *  4. SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10)
 *      used in monocular initialization. takes features that were matched in previous frame F1 and attempts to propagate those matches to F2
 *      by taking all features in a window in F2 centered around the point where the matched feature was in F1 and then applying several criteria:
 *      distance is less than any previous match, distance is best of candidate matches and less than threshold, apply global rotation consistency criterion
 *      outputs vnMatches12 (for each feature in F1 gives index of feature match in F2 or -1 = no match), and updated vbPrevMatched (similar structure but value is position in F2)
 *      which are then used in monocular initialization
 *
 *   5. SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
 *     LEGACY for Loop Closing: Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12] , In the stereo and RGB-D case, s12=1
 */

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <MatchCriteria.h>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace HYSLAM
{

struct FeatureMatcherSettings{
    float nnratio = 0.6;
    float TH_HIGH = 100.0;
    float TH_LOW = 50.0;
    bool checkOri = true;
};

class FeatureMatcher
{
public:

    FeatureMatcher(float nnratio=0.6, bool checkOri=true);
    FeatureMatcher(FeatureMatcherSettings settings);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th = 3);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::map<size_t, MapPoint*> &matches);
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
    int SearchByBoW2(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<std::pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize = 10);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const std::vector<MapPoint *> &vpMapPoints, std::map<std::size_t, MapPoint*> &fuse_matches, const float th=3.0, const float reprojection_err = 5.99);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, std::vector<MapPoint *> &vpReplacePoint);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

public:

//    static const int TH_LOW;
//    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:
    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
    float TH_LOW;
    float TH_HIGH;

    int _SearchByProjection_(Frame &frame, const std::vector<MapPoint*> &landmarks, const float th,
                             std::vector< std::unique_ptr<LandMarkCriterion> >     &landmark_criteria,
                             std::vector< std::unique_ptr<LandMarkViewCriterion> > &landmarkview_criteria,
                             std::vector< std::unique_ptr<GlobalCriterion> >       &global_criteria,
                             CriteriaData &criteria_data
    );

    int _SearchByBoW_(KeyFrame* pKF1, KeyFrame* pKF2, std::map<size_t, size_t> &matches,
                      std::vector< std::unique_ptr<BoWIndexCriterion> > &index_criteria,
                      std::vector< std::unique_ptr<BoWMatchCriterion> > &match_criteria,
                      std::vector< std::unique_ptr<RotationConsistencyBoW> > &global_criteria
    );
};



}// namespace ORB_SLAM

#endif // ORBMATCHER_H
