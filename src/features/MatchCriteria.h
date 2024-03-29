#ifndef MATCHCRITERIA_H_
#define MATCHCRITERIA_H_

/*
 * collection of match criterion classes used for feature matching
 * criterion is applied to a set of input candidates, those passing the criterion are returned
 *
 * GENERAL PURPOSE:
 * 1. LandMarkCriterion - abstract base class for criterion that are are applied to landmarks
 *   interface:
 *   std::vector<MapPoint*> apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data)
 *   std::vector<MapPoint*> apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data)
 *   - takes a set of landmarks (candidate_lms) which may match to Frame (or KeyFrame) and applies a criterion to winnow match
 *     returns a vector with candidate_lms that pass the criterion. optionally may use CriteriaData
 *
 *   concrete classes:
 *   ProjectionCriterion - landmark must project into the image
 *   DistanceCriterion - landmark distance to camera must be within in min and max valid viewing distance
 *   ViewingAngleCriterion - angle between landmark's normal and viewing vector in Frame must not exceed a threshold (max_angle passed in constructor)
 *
 * 2. LandMarkViewCriterion -  abstract base class for criteria that are are applied to landmarksviews (i.e. features)
 *   interface:
 *   std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data)
 *   std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data)
 *   - considers potential feature matches (candidate_views) in Frame or KeyFrame to landmark lm. applies criterion to winnow match
 *   and returns vector of feature indices passing criterion
 *
 *   concrete classes:
 *   PreviouslyMatchedCriterion - feature must not have an existing match to a landmark
 *   StereoConsistencyCriterion - if stereo (camera and feature), feature's u-coordinate in right image must be consistent with projection of landmark lm
 *   BestScoreCriterion - feature's descriptor must have the "best score" (lowest distance) to landmark's descriptor among candidates.
 *              further more, that best distance must not be greater than score_threshold (passed in constructor),
 *              and optionally must be less than nnRatio*2ndBestScore to ensure the best match is substantially better than alternative possibilities
 *   ProjectionViewCriterion - squared distance between projection of landmark lm into image and candidate feature is less than error_threshold (passed in constructor)
 *   FeatureSizeCriterion - feature size must be similar to that of projected landmark lm (within frac_smaller and frac_larger passed in constructor)
 *
 * 3. GlobalCriterion - abstract pass class for criteria that are applied to sets of candidate landmark->feature matches, winnowing these
 *   interface:
 *    MatchesFound apply(MatchesFound current_matches, Frame &frame, CriteriaData &data)
 *    - takes a set of candidate matches between keypoints and landmarks (current_matches, have already passed LandMark and LandMarkView criteria) in Frame
 *    and applies a criterion to winnow these matches based on global property of the matches
 *
 *    concrete classes:
 *    RotationConsistencyCriterion - considers landmarks that were matched in both current Frame (F) and previous frame (in CriteriaData).
 *      the rotation of keypoint angles between successive frames should be roughly the same among all valid matches.
 *      enforces this by binning rotation angles and only allows matches in 3 maximum bins in histogram to pass
 *    GlobalBestScoreCriterion - a single feature in frame F may be matched to multiple landmarks - keep only the one with the best descriptor match to the landmark
 *    formulated this so it could work with distance measures (lowest distance = best match) or similarity measures (highest similarity = best match)
 *    can set this with keep_highest (set false for distance measure, true for similarity measure)
 *
 * MONOCULAR INITALIZATION
 * 4. MonoInitViewCriterion - abstract pass class for Monocular Initialization criteria
 *    interface:
 *    std::vector<size_t> apply(  std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data)
 *      filters candidate_matches with a criterion making use of a set of views (typically from previous frame) and potentially additional data (MonoCriteriaData)
 *      returns vector of feature indices for features that passed the criterion.
 *
 *    concrete classes:
 *    MonoInitBestScore - feature's descriptor must have the "best score" (lowest distance) to landmark's descriptor among candidates.
 *              further more, that best distance must not be greater than score_threshold (passed in constructor),
 *              and optionally must be less than nnRatio*2ndBestScore to ensure the best match is substantially better than alternative possibilities
 *              basically the same as BestScoreCriterion but interface is different.
 *    MonoInitScoreExceedsPrevious - if the feature has already been matched to another feature, ensure the feature descriptors similarity to new potential
 *              match is better than to previous match
 *
 * BAG OF WORDS MATCHING
 * 5. BoWIndexCriterion{
 *    virtual std::vector<unsigned int> apply(KeyFrame* pKF, std::vector<unsigned int> cand_idxs)
 *    takes a candidate set of feature matches (cand_idx) in pKF and applies criterion return feature indices that pass
 *
 *    concrete
 *    PreviouslyMatchedIndexCriterion - constructor takes boolean that controls whether the criteria passes only features in pKF that have previously been matched to a landmark (keep_previously_matched = true)
 *    or the opposite -> keeps only features that haven't been matched to a landmark  (keep_previously_matched = true)
 *    StereoIndexCriterion - passes only features that have a Stereomatched feature in the right frame
 *
 * 6. BoWMatchCriterion{
 *   virtual std::vector<unsigned int> apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 )
 *   considers potential matches cand_idx2 (from views2) to feature idx1 in views1 and filters the potential candidates returning feature indices that pass criterion
 *
 *   concrete classes:
 *   BestMatchBoWCriterion - feature's descriptor must have the "best score" (lowest distance) to idx1's descriptor among candidates.
 *              further more, that best distance must not be greater than score_threshold (passed in constructor),
 *              and optionally must be less than nnRatio*2ndBestScore to ensure the best match is substantially better than alternative possibilities
 *              basically the same as BestScoreCriterion but interface is different.
 *   EpipolarConsistencyBoWCriterion - using fundamental matrix for Frames 1 and 2 (or KeyFrames) passed in constructor only passes candidate features in
 *      cand_idx2 if the Epipolar line distance in frame 1 is below a threshold - ensures geometric consistency of match
 *
 * RotationConsistencyBoW - standalone, considers landmarks that were matched in both current Frame (F) and previous frame (in CriteriaData).
 *      the rotation of keypoint angles between successive frames should be roughly the same among all valid matches.
 *      enforces this by binning rotation angles and only allows matches in 3 maximum bins in histogram to pass
 *
 */

#include <MapPoint.h>
#include <Frame.h>
#include <KeyFrame.h>
#include <FeatureDescriptor.h>

#include <map>
#include <vector>

namespace HYSLAM{

//HAD TO DUPLICATE MANY CRITERIA B/C FRAME AND KEYFRAME ARE NOT DERIVED FROM A COMMON BASE CLASS

struct SingleMatchData{
    size_t idx;
    float distance;
    float distance_2ndbest;
};

using MatchesFound = std::map<MapPoint*, SingleMatchData>;
using MatchesIdx   = std::map<size_t, size_t>;

class CriteriaData{
public:
    CriteriaData(){};
    void setPreviousFrame( Frame* prev_frame_){prev_frame = prev_frame_;}
    Frame* getPreviousFrame() {return prev_frame; };
    void setBestScore(float score_){best_score = score_;}
    float getBestScore(){return best_score;}
private:
    Frame* prev_frame = nullptr;
    float best_score;

};

class LandMarkCriterion{  //abstract base class defining interface
public:
   virtual std::vector<MapPoint*> apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data) = 0;
   virtual std::vector<MapPoint*> apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data) = 0;
};

class LandMarkViewCriterion{  //abstract base class defining interface
public:
    virtual std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data) = 0;
    virtual std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data) = 0;
};

class GlobalCriterion{ //abstract base class defining interface
public:
    virtual MatchesFound apply(MatchesFound current_matches, Frame &frame, CriteriaData &data) = 0;
};

//LandMark Criteria
class ProjectionCriterion : public LandMarkCriterion{
public:
    std::vector<MapPoint*> apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data);
    std::vector<MapPoint*> apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data);
};

class DistanceCriterion : public LandMarkCriterion{
public:
    std::vector<MapPoint*> apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data);
    std::vector<MapPoint*> apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data);
};

class ViewingAngleCriterion : public LandMarkCriterion{
public:
    ViewingAngleCriterion(){};
    ViewingAngleCriterion(float max_angle_radians_);
    std::vector<MapPoint*> apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data);
    std::vector<MapPoint*> apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data);
private:
    float max_angle_radians = 1.047; //default is 60 degrees = 1.047 in radians
};

//LandMarkView Criteria
class PreviouslyMatchedCriterion : public LandMarkViewCriterion {
public:
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
};

class StereoConsistencyCriterion : public LandMarkViewCriterion {
public:
    StereoConsistencyCriterion(){};
    StereoConsistencyCriterion(float threshold_);
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
private:
    float threshold = 10.0;
};

class BestScoreCriterion : public LandMarkViewCriterion {
public:
    BestScoreCriterion(){};
    BestScoreCriterion( float score_threshold_, float second_best_ratio_);
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
private:
    float score_threshold = 80;
    float second_best_ratio = 0.7;

};

class ProjectionViewCriterion : public LandMarkViewCriterion {
public:
    ProjectionViewCriterion(){};
    ProjectionViewCriterion(float  error_threshold_);
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
private:
    float  error_threshold = 5.99;
};

class FeatureSizeCriterion: public LandMarkViewCriterion {
public:
    FeatureSizeCriterion(){};
    FeatureSizeCriterion(float frac_smaller_, float frac_larger_);
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
private:
    float frac_smaller = 0.5;
    float frac_larger = 1.5;

};

//Global Criteria

class RotationConsistencyCriterion : public GlobalCriterion{
public:
    MatchesFound apply(MatchesFound current_matches, Frame &frame, CriteriaData &data);
private:
    int HISTO_LENGTH = 30;

};

class GlobalBestScoreCriterion : public GlobalCriterion{
public:
    GlobalBestScoreCriterion(){};
    GlobalBestScoreCriterion(bool keep_highest_);
    MatchesFound apply(MatchesFound current_matches, Frame &frame, CriteriaData &data);
private:
    bool keep_highest; // highest score best (i.e. similarity) = true; lowest score best (distance) = false;
};

// Mono Initializer Criteria

class MonoCriteriaData{
public:
    MonoCriteriaData(){};
    void setDescriptor( FeatureDescriptor desc_){descriptor = desc_;}
    FeatureDescriptor getDescriptor() {return descriptor; };
    void setIndex(size_t idx){idx_current = idx;}
    size_t getIndex(){return idx_current;}
    void setDistance(size_t idx, float dist);
    float getDistance(size_t idx);
private:
    FeatureDescriptor descriptor;
    size_t idx_current;
    std::map<size_t, float> distances;

};

class MonoInitViewCriterion{  //abstract base class defining interface
public:
    virtual std::vector<size_t> apply(  std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data) = 0;
};
/*
class MonoInitGlobalCriterion{ //abstract base class defining interface
public:
    virtual std::map<size_t, size_t>  apply(std::map<size_t, size_t> current_matches, Frame &frame, CriteriaData &data) = 0;
};
*/
class MonoInitBestScore : public MonoInitViewCriterion{
public:
    MonoInitBestScore(){};
    MonoInitBestScore( float score_threshold_, float second_best_ratio_);
    std::vector<size_t> apply( std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data);
private:
    float score_threshold = 80;
    float second_best_ratio = 0.7;
};
class MonoInitScoreExceedsPrevious : public MonoInitViewCriterion{
public:
    std::vector<size_t> apply(  std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data);

};

//BoW match

class BoWIndexCriterion{
public:
    virtual std::vector<unsigned int> apply(KeyFrame* pKF, std::vector<unsigned int> cand_idxs) = 0;
};

class BoWMatchCriterion{
public:
    virtual std::vector<unsigned int> apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 ) = 0;
};

class PreviouslyMatchedIndexCriterion : public BoWIndexCriterion{
public:
    PreviouslyMatchedIndexCriterion(){};
    PreviouslyMatchedIndexCriterion(bool keep_previously_matched_);
    std::vector<unsigned int> apply(KeyFrame* pKF, std::vector<unsigned int> cand_idxs );
private:
    bool keep_previously_matched = true;
};

class StereoIndexCriterion: public BoWIndexCriterion{
public:
    std::vector<unsigned int> apply(KeyFrame* pKF, std::vector<unsigned int> cand_idxs );
};

class BestMatchBoWCriterion : public BoWMatchCriterion{
public:
    BestMatchBoWCriterion(){};
    BestMatchBoWCriterion(float score_threshold_, float second_best_ratio_);
    std::vector<unsigned int> apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 );
private:
    float score_threshold = 80;
    float second_best_ratio = 0.7;
};

class EpipolarConsistencyBoWCriterion : public BoWMatchCriterion {
public:
    EpipolarConsistencyBoWCriterion(){};
    EpipolarConsistencyBoWCriterion(cv::Mat Fmatrix_, float ex, float ey);
    std::vector<unsigned int> apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 );
private:
    cv::Mat Fmatrix;
    float ex;
    float ey;

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, float sigma2);
};

class RotationConsistencyBoW{
public:
    MatchesIdx apply(MatchesIdx current_matches, const FeatureViews &views1, const FeatureViews &views2 );
};

//Free functions
std::vector<MapPoint*> DistanceCriterionCore(cv::Mat camera_center, std::vector<MapPoint*> &candidate_lms);
std::vector<MapPoint*>  ViewingAngleCriterionCore(cv::Mat camera_center, std::vector<MapPoint*> &candidate_lms, float max_angle);
std::vector<size_t> PreviouslyMatchedCriterionCore(const LandMarkMatches &lm_matches, std::vector<size_t> & candidate_views);
SingleMatchData BestScoreCriterionCore(const FeatureViews &views,MapPoint* lm, std::vector<size_t> &candidate_views);
std::vector<size_t> FeatureSizeCriterionCore(float projected_size,float frac_smaller_, float frac_larger_, std::vector<size_t> &candidate_views, const FeatureViews &views);
//MatchesIdx RotationConsistency(MatchesIdx current_matches,Frame &frame_current, Frame &frame_previous);
MatchesIdx RotationConsistency(MatchesIdx current_matches, const FeatureViews &views_curr,  const FeatureViews &views_prev );
void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

}//end namespace
#endif