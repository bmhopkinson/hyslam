#ifndef MATCHCRITERIA_H_
#define MATCHCRITERIA_H_

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
    float score;
    int level;
    float score_2ndbest;
    int level_2ndbest;
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
    BestScoreCriterion( int score_threshold_, float second_best_ratio_);
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
private:
    int score_threshold = 80;
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


class PyramidLevelCriterion: public LandMarkViewCriterion {
public:
    PyramidLevelCriterion(){};
    PyramidLevelCriterion(int n_below_, int n_above_);
    std::vector<size_t> apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
    std::vector<size_t> apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data);
private:
    int n_below = 1;
    int n_above = 1;

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
    MonoInitBestScore( int score_threshold_, float second_best_ratio_);
    std::vector<size_t> apply( std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data);
private:
    int score_threshold = 80;
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
    BestMatchBoWCriterion(int score_threshold_, float second_best_ratio_);
    std::vector<unsigned int> apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 );
private:
    int score_threshold = 80;
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
int descriptorDistance(const cv::Mat &a, const cv::Mat &b);
int determinePredictedLevel(Frame &frame, MapPoint* lm, CriteriaData &data);
int determinePredictedLevel(KeyFrame* pKF, MapPoint* lm, CriteriaData &data);
std::vector<MapPoint*> DistanceCriterionCore(cv::Mat camera_center, std::vector<MapPoint*> &candidate_lms);
std::vector<MapPoint*>  ViewingAngleCriterionCore(cv::Mat camera_center, std::vector<MapPoint*> &candidate_lms, float max_angle);
std::vector<size_t> PreviouslyMatchedCriterionCore(const LandMarkMatches &lm_matches, std::vector<size_t> & candidate_views);
SingleMatchData BestScoreCriterionCore(const FeatureViews &views,MapPoint* lm, std::vector<size_t> &candidate_views);
std::vector<size_t> PyramidLevelCriterionCore(int predicted_level, int n_below, int n_above, const FeatureViews &views, MapPoint* lm, std::vector<size_t> &candidate_views );
//MatchesIdx RotationConsistency(MatchesIdx current_matches,Frame &frame_current, Frame &frame_previous);
MatchesIdx RotationConsistency(MatchesIdx current_matches, const FeatureViews &views_curr,  const FeatureViews &views_prev );
void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

}//end namespace
#endif