#include <MatchCriteria.h>
#include <LandMarkMatches.h>
#include <FeatureViews.h>
#include <FeatureExtractorSettings.h>
#include <Camera.h>
#include <math.h>
#include <assert.h>
#include <limits>

namespace HYSLAM{

//LandMark Criteria
std::vector<MapPoint*> ProjectionCriterion::apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data){
    std::vector<MapPoint*> lms_passed;
    for(auto it = candidate_lms.begin(); it != candidate_lms.end(); ++it){
        MapPoint* lm = *it;
        if(!lm){
            continue;
        }

        cv::Mat uv;
        bool valid = frame.ProjectLandMark(lm, uv);
        if(valid){
            lms_passed.push_back(lm);
        }
    }
    return lms_passed;
}

std::vector<MapPoint*> ProjectionCriterion::apply(KeyFrame* pKF,  std::vector<MapPoint*> &candidate_lms, CriteriaData &data){
    std::vector<MapPoint*> lms_passed;
    for(auto it = candidate_lms.begin(); it != candidate_lms.end(); ++it){
        MapPoint* lm = *it;
        if(!lm){
            continue;
        }

        cv::Mat uv;
        if(pKF->ProjectLandMark(lm, uv)){
            lms_passed.push_back(lm);
        }
    }
    return lms_passed;
}

std::vector<MapPoint*> DistanceCriterion::apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data) {
    cv::Mat camera_center = frame.GetCameraCenter();
    return DistanceCriterionCore(camera_center, candidate_lms);

}

std::vector<MapPoint*> DistanceCriterion::apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data) {
    cv::Mat camera_center = pKF->GetCameraCenter();
    return DistanceCriterionCore(camera_center, candidate_lms);

}

std::vector<MapPoint*> DistanceCriterionCore(cv::Mat camera_center, std::vector<MapPoint*> &candidate_lms){
    std::vector<MapPoint*> lms_passed;
    for(auto it = candidate_lms.begin(); it != candidate_lms.end(); ++it){
        MapPoint* lm = *it;
        if(!lm){
            continue;
        }

        cv::Mat PO = lm->GetWorldPos() - camera_center;
        const float lm_dist = cv::norm(PO);

        const float maxDistance = lm->GetMaxDistanceInvariance();
        const float minDistance = lm->GetMinDistanceInvariance();
        if(lm_dist < minDistance || lm_dist > maxDistance){
            continue;
        } else {
            lms_passed.push_back(lm);
        }
    }
    return lms_passed;
}

ViewingAngleCriterion::ViewingAngleCriterion(float max_angle_radians_) : max_angle_radians(max_angle_radians_)
{
}

std::vector<MapPoint*>  ViewingAngleCriterion::apply(Frame &frame, std::vector<MapPoint*> &candidate_lms, CriteriaData &data){
    cv::Mat camera_center = frame.GetCameraCenter();
    return ViewingAngleCriterionCore(camera_center,candidate_lms, max_angle_radians );
}

std::vector<MapPoint*>  ViewingAngleCriterion::apply(KeyFrame* pKF, std::vector<MapPoint*> &candidate_lms, CriteriaData &data){
    cv::Mat camera_center = pKF->GetCameraCenter();
    return ViewingAngleCriterionCore(camera_center,candidate_lms, max_angle_radians );
}

std::vector<MapPoint*>  ViewingAngleCriterionCore(cv::Mat camera_center, std::vector<MapPoint*> &candidate_lms, float max_angle){
    std::vector<MapPoint*> lms_passed;

    for(auto it = candidate_lms.begin(); it != candidate_lms.end(); ++it) {
        MapPoint* lm = *it;
        cv::Mat lm_position = lm->GetWorldPos();
        cv::Mat PO = lm_position - camera_center;
        const float distance = cv::norm(PO);
        cv::Mat Pn = lm->GetNormal();
        PO = PO/distance;

        if (PO.dot(Pn) > cos(max_angle)) {
            lms_passed.push_back(lm);
        }
    }
    return lms_passed;
}

//LandMarkView Criteria
std::vector<size_t> PreviouslyMatchedCriterion::apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    const LandMarkMatches lm_matches = frame.getLandMarkMatches();
    return PreviouslyMatchedCriterionCore(lm_matches, candidate_views);

}

std::vector<size_t> PreviouslyMatchedCriterion::apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    const LandMarkMatches lm_matches = pKF->getLandMarkMatches();
    return PreviouslyMatchedCriterionCore(lm_matches, candidate_views);
}

std::vector<size_t> PreviouslyMatchedCriterionCore(const LandMarkMatches &lm_matches, std::vector<size_t> & candidate_views){
    std::vector<size_t> views_passed;
    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it){
        size_t idx = *it;

        bool save = true;
        MapPoint* lm_prev = lm_matches.hasAssociation(idx);
        if(lm_prev){
            if(lm_prev->Observations() > 0) {  //not sure why this test is neccesary
                save = false;
            }
        }

        if(save){
            views_passed.push_back(idx);
        }

    }

    return views_passed;
}

StereoConsistencyCriterion::StereoConsistencyCriterion(float threshold_) : threshold(threshold_)
{}

std::vector<size_t> StereoConsistencyCriterion::apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    std::vector<size_t> views_passed;

    Camera camera = frame.getCamera();
    if(camera.sensor == 0){ //if camera is mono, this criterion cannot be applied.
        return candidate_views;
    }

    const FeatureViews& views = frame.getViews();
    FeatureExtractorSettings orb_params = views.orbParams();
    Frame* frame_prev = data.getPreviousFrame();

    const float radius = threshold * frame.landMarkSizePixels(lm)/orb_params.size_ref;
    cv::Mat uv;
    frame.ProjectLandMark(lm, uv);
    float ur = uv.at<float>(2,0);

    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it) {
        size_t idx = *it;

        float ur_view = views.uR(idx);
        const float er = fabs(ur - ur_view );
        if(er < radius && ur_view > 0){  //ur_view < 0 indicates no stereo correspondence
            views_passed.push_back(idx);
        }
    }

    return views_passed;
}


std::vector<size_t> StereoConsistencyCriterion::apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    std::vector<size_t> views_passed;

    Camera camera = pKF->getCamera();
    if(camera.sensor == 0){ //if camera is mono, this criterion cannot be applied.
        return candidate_views;
    }

    const FeatureViews& views = pKF->getViews();
    FeatureExtractorSettings orb_params = views.orbParams();
    Frame* frame_prev = data.getPreviousFrame();

    const float radius = threshold * pKF->landMarkSizePixels(lm)/orb_params.size_ref;
    cv::Mat uv;
    pKF->ProjectLandMark(lm, uv);
    float ur = uv.at<float>(2,0);

    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it) {
        size_t idx = *it;

        float ur_view = views.uR(idx);
        const float er = fabs(ur - ur_view );
        if(er < radius && ur_view > 0){  //ur_view < 0 indicates no stereo correspondence
            views_passed.push_back(idx);
        }
    }

    return views_passed;
}

BestScoreCriterion::BestScoreCriterion( float score_threshold_, float second_best_ratio_) :
score_threshold(score_threshold_), second_best_ratio(second_best_ratio_)
{}

std::vector<size_t> BestScoreCriterion::apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    std::vector<size_t> views_passed;

    const FeatureViews& views = frame.getViews();
    SingleMatchData best_view =  BestScoreCriterionCore(views, lm, candidate_views);

    if(best_view.distance <= score_threshold) {
        if (best_view.distance > second_best_ratio * best_view.distance_2ndbest) {
        } else {
            data.setBestScore(best_view.distance);
            views_passed.push_back(best_view.idx);
        }
    }

    return views_passed;

}

std::vector<size_t> BestScoreCriterion::apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    std::vector<size_t> views_passed;
    const FeatureViews& views = pKF->getViews();
    SingleMatchData best_view =  BestScoreCriterionCore(views, lm, candidate_views);

    if(best_view.distance <= score_threshold) {
        if (best_view.distance > second_best_ratio * best_view.distance_2ndbest) {
        } else {
            data.setBestScore(best_view.distance);
            views_passed.push_back(best_view.idx);
        }
    }

    return views_passed;
}

SingleMatchData BestScoreCriterionCore(const FeatureViews &views,MapPoint* lm, std::vector<size_t> &candidate_views){
    SingleMatchData best_view;
    const FeatureDescriptor& lm_descriptor = lm->GetDescriptor();

    float bestDist = std::numeric_limits<float>::max();
    float bestDist2 = std::numeric_limits<float>::max();
    int bestIdx =-1 ;

    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it) {
        size_t idx = *it;
        const FeatureDescriptor &d = views.descriptor(idx);

        const float dist_feature = lm_descriptor.distance(d);

        if(dist_feature<bestDist)
        {
            bestDist2=bestDist;
            bestDist=dist_feature;
            bestIdx=idx;
        }
        else if(dist_feature<bestDist2)
        {
            bestDist2=dist_feature;
        }

    }

    best_view.idx = bestIdx;
    best_view.distance = bestDist;
    best_view.distance_2ndbest = bestDist2;

    return best_view;
}

ProjectionViewCriterion::ProjectionViewCriterion(float  error_threshold_):  error_threshold( error_threshold_)
{}

std::vector<size_t> ProjectionViewCriterion::apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    std::vector<size_t> views_passed;
    const FeatureViews& views = frame.getViews();
    FeatureExtractorSettings orb_params = views.orbParams();

    cv::Mat lm_position = lm->GetWorldPos();
    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it){
        size_t idx = *it;
        float reproj_err = frame.ReprojectionError(lm_position, idx);

        const cv::KeyPoint &kp = views.keypt(idx);
        float sigma_size_corrected = orb_params.determineSigma2(kp.size);

        float stereo_factor = 1.00;
        if(views.uR(idx) > 0 ) { //stereo observation
            stereo_factor = 1.30;
        }
        if( (reproj_err/sigma_size_corrected) < stereo_factor * error_threshold) {
            views_passed.push_back(idx);
        }
    }
    return views_passed;
}

std::vector<size_t> ProjectionViewCriterion::apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    std::vector<size_t> views_passed;
    const FeatureViews& views = pKF->getViews();
    FeatureExtractorSettings orb_params = views.orbParams();

    cv::Mat lm_position = lm->GetWorldPos();
    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it){
        size_t idx = *it;
        float reproj_err = pKF->ReprojectionError(lm_position, idx);

        const cv::KeyPoint &kp = views.keypt(idx);
        float sigma_size_corrected =  orb_params.determineSigma2(kp.size);

        float stereo_factor = 1.00;
        if(views.uR(idx) > 0 ) { //stereo observation
            stereo_factor = 1.30;
        }

        if( (reproj_err/sigma_size_corrected) < stereo_factor * error_threshold){
            views_passed.push_back(idx);
        }
    }
    return views_passed;

}

FeatureSizeCriterion::FeatureSizeCriterion(float frac_smaller_, float frac_larger_) : frac_smaller(frac_smaller_), frac_larger(frac_larger_)
{}

std::vector<size_t> FeatureSizeCriterion::apply(Frame &frame, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    float projected_size = frame.landMarkSizePixels(lm);
    const FeatureViews& views = frame.getViews();
    return FeatureSizeCriterionCore(projected_size, frac_smaller, frac_larger, candidate_views, views);
}

std::vector<size_t> FeatureSizeCriterion::apply(KeyFrame* pKF, MapPoint* lm,  std::vector<size_t> &candidate_views, CriteriaData &data){
    float projected_size = pKF->landMarkSizePixels(lm);
    const FeatureViews& views = pKF->getViews();
    return FeatureSizeCriterionCore(projected_size, frac_smaller, frac_larger, candidate_views, views);
}

std::vector<size_t> FeatureSizeCriterionCore(float projected_size,float frac_smaller_, float frac_larger_, std::vector<size_t> &candidate_views, const FeatureViews &views){
    std::vector<size_t> views_passed;
    for(auto it = candidate_views.begin(); it != candidate_views.end(); ++it) {
        size_t idx = *it;
        const cv::KeyPoint &kp = views.keypt(idx);
        if(kp.size > frac_smaller_*projected_size && kp.size < frac_larger_*projected_size ){
            views_passed.push_back(idx);
        }
    }
    return views_passed;
}

// Global Criteria
MatchesFound RotationConsistencyCriterion::apply(MatchesFound current_matches, Frame &frame, CriteriaData &data){
    MatchesFound matches_passed = current_matches;

    const FeatureViews& views = frame.getViews();
    Frame* frame_prev = data.getPreviousFrame();
    if(!frame_prev){  //need prev frame to check rotational consistency
        return current_matches;
    }


    //alternate approach
    //get data into correct format for RotationConsistency
    MatchesIdx current_matches_idx;
    std::map<size_t, MapPoint*> current_matches_inverse;
    for(auto it = current_matches.begin(); it != current_matches.end(); ++it) {
        MapPoint *lm = it->first;
        SingleMatchData match_data = it->second;
        size_t idx_curr = match_data.idx;
        int idx_prev = frame_prev->hasAssociation(lm);
        current_matches_idx[idx_curr] = idx_prev;

        current_matches_inverse[idx_curr] = lm;
    }

    //do rotation consistency check
    const FeatureViews &views_curr = frame.getViews();
    const FeatureViews &views_prev = frame_prev->getViews();
    MatchesIdx matches_passed_idx_alt = RotationConsistency(current_matches_idx, views_curr, views_prev);

    //translate results into proper format for output
    MatchesFound matches_passed_alt;
    for(auto it = matches_passed_idx_alt.begin(); it != matches_passed_idx_alt.end(); ++it){
        size_t idx = it->first;
        MapPoint* lm = current_matches_inverse[idx];
        matches_passed_alt[lm] = current_matches[lm];
    }

    return matches_passed_alt;
}

GlobalBestScoreCriterion::GlobalBestScoreCriterion(bool keep_highest_) : keep_highest(keep_highest_)
{}

MatchesFound GlobalBestScoreCriterion::apply(MatchesFound current_matches, Frame &frame, CriteriaData &data){
    MatchesFound matches_passed = current_matches;
    std::map<std::size_t , std::vector<std::pair<MapPoint*, float> > >  lm_matches;

    //populate lm_matches
    for(auto it = current_matches.begin(); it != current_matches.end(); ++it){
        MapPoint* lm = it->first;
        SingleMatchData smdata = it->second;
        size_t idx = smdata.idx;

        auto mit = lm_matches.find(idx);
        if(mit == lm_matches.end()){ //new element
            std::vector<std::pair<MapPoint*, float> > new_element;
            new_element.push_back(std::make_pair(lm, smdata.distance));
            lm_matches[idx] = new_element;
        } else {
            lm_matches[idx].push_back(std::make_pair(lm, smdata.distance));
        }
    }

    //resolve cases where multiple landmarks matched a single keypoint
    for(auto it = lm_matches.begin(); it != lm_matches.end(); ++it){
        std::vector<std::pair<MapPoint*, float> > idx_matches;
        idx_matches = it->second;
        if( idx_matches.size() > 1 ){
            MapPoint* best_lm;
            float best_score;
            if(keep_highest){
                best_score = std::numeric_limits<float>::min();
            } else {
                best_score = std::numeric_limits<float>::max();
            }

            for(auto vit = idx_matches.begin(); vit != idx_matches.end(); ++vit){ //find best score
                MapPoint* lm = vit->first;
                float lm_score = vit->second;
                if(keep_highest){
                    if(lm_score > best_score){
                        best_score = lm_score;
                        best_lm = lm;
                    }
                } else {
                    if(lm_score < best_score){
                        best_score = lm_score;
                        best_lm = lm;
                    }
                }
            }

            for(auto vit = idx_matches.begin(); vit != idx_matches.end(); ++vit) { //remove matches to landmarks other than the best one
                MapPoint* lm = vit->first;
                if(lm != best_lm){
                    matches_passed.erase(lm);
                    std::cout << "erasing duplicate match to idx: " << it->first << ", " << lm->mnId << " because lm " << best_lm->mnId << "is better" << std::endl;
                }
            }

        }
    }

    return matches_passed;
}

void  MonoCriteriaData::setDistance(size_t idx, float dist){
    distances[idx] = dist;
}

float  MonoCriteriaData::getDistance(size_t idx){
    auto it = distances.find(idx);
    if(it != distances.end()){
        return it->second;
    } else {
        return -1;
    }
}

MonoInitBestScore::MonoInitBestScore( float score_threshold_, float second_best_ratio_) :
score_threshold(score_threshold_), second_best_ratio(second_best_ratio_)
{}

std::vector<size_t> MonoInitBestScore::apply( std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data){
    std::vector<size_t> views_passed;

    float bestDist = std::numeric_limits<float>::max();
    float bestDist2 = std::numeric_limits<float>::max();
    int bestIdx = -1;
    FeatureDescriptor d1 = data.getDescriptor();

    for(std::vector<size_t>::iterator vit=candidate_matches.begin(); vit!=candidate_matches.end(); vit++)
    {
        size_t i2 = *vit;

        FeatureDescriptor d2 = views.descriptor(i2);

        float dist = d1.distance(d2);

        if(dist<bestDist)
        {
            bestDist2=bestDist;
            bestDist=dist;
            bestIdx=i2;
        }
        else if(dist<bestDist2)
        {
            bestDist2=dist;
        }
    }

    if(bestDist<= score_threshold) //distance must be less than threshold to be considered a match
    {
        if(bestDist<(float)bestDist2 * second_best_ratio )  //best match must substantially exceed 2nd best match
        {
            views_passed.push_back(bestIdx);
        }
    }

    return views_passed;
}

std::vector<size_t> MonoInitScoreExceedsPrevious::apply( std::vector<size_t> &candidate_matches, const FeatureViews &views,  MonoCriteriaData &data) {
    std::vector<size_t> views_passed;

    FeatureDescriptor d1 = data.getDescriptor();

    for (std::vector<size_t>::iterator vit = candidate_matches.begin(); vit != candidate_matches.end(); vit++) {
        size_t i2 = *vit;

        float dist_prev = data.getDistance(i2);
        if(dist_prev < 0) { //no previous match so it passes
            views_passed.push_back(i2);

        } else {
            FeatureDescriptor d2 = views.descriptor(i2);
            int dist = d1.distance(d2);

            if(dist < dist_prev){
                views_passed.push_back(i2);
            }
        }

    }

    return views_passed;
}

PreviouslyMatchedIndexCriterion::PreviouslyMatchedIndexCriterion(bool keep_previously_matched_) :
keep_previously_matched(keep_previously_matched_)
{}

std::vector<unsigned int> PreviouslyMatchedIndexCriterion::apply(KeyFrame* pKF, std::vector<unsigned int> cand_idxs ){
    std::vector<unsigned int> idxs_passed;
    for(auto it = cand_idxs.begin(); it != cand_idxs.end(); ++it){
        bool has_match = false;
        unsigned int idx = *it;
        MapPoint* lm = pKF->hasAssociation(idx);
        if(lm){
            if(!lm->isBad()){
                has_match = true;
            }
        }
        if(keep_previously_matched && has_match){
            idxs_passed.push_back(idx);
        }
        if(!keep_previously_matched && !has_match){
            idxs_passed.push_back(idx);
        }
    }
    return idxs_passed;
}


std::vector<unsigned int>  StereoIndexCriterion::apply(KeyFrame* pKF, std::vector<unsigned int> cand_idxs ){
    std::vector<unsigned int> idxs_passed;

    Camera camera = pKF->getCamera();
    if(camera.sensor == 0){ //if camera is mono, this criterion cannot be applied.
        return cand_idxs;
    }

    const FeatureViews& KFviews = pKF->getViews();
    for(auto it = cand_idxs.begin(); it != cand_idxs.end(); ++it) {
        unsigned int idx = *it;
        bool is_stereo = KFviews.uR(idx) >= 0;
        if(is_stereo){
            idxs_passed.push_back(idx);
        }
    }
    return idxs_passed;
}

BestMatchBoWCriterion::BestMatchBoWCriterion(float score_threshold_, float second_best_ratio_) :
score_threshold(score_threshold_), second_best_ratio(second_best_ratio_)
{}


std::vector<unsigned int> BestMatchBoWCriterion::apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 ){
    std::vector<unsigned int> idxs_passed;
    FeatureDescriptor d1 = views1.descriptor(idx1);

    float bestDist1 = std::numeric_limits<float>::max();
    int bestIdx2 =-1 ;
    float bestDist2 = std::numeric_limits<float>::max();

    for(auto it = cand_idx2.begin(); it != cand_idx2.end(); ++it)
    {
        unsigned int idx2 = *it;

        const FeatureDescriptor &d2 = views2.descriptor(idx2);

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

    if(bestDist1 < score_threshold) {
        if (static_cast<float>(bestDist1) < second_best_ratio * static_cast<float>(bestDist2)) {
            idxs_passed.push_back(bestIdx2);
        }
    }
    return idxs_passed;
}

EpipolarConsistencyBoWCriterion::EpipolarConsistencyBoWCriterion(cv::Mat Fmatrix_, float ex_, float ey_) :
Fmatrix(Fmatrix_), ex(ex_), ey(ey_)
{}

std::vector<unsigned int> EpipolarConsistencyBoWCriterion::apply(size_t idx1, std::vector<unsigned int> cand_idx2, const FeatureViews &views1, const FeatureViews &views2 ){
    std::vector<unsigned int> idxs_passed;

    const cv::KeyPoint &kp1 = views1.keypt(idx1);
    FeatureExtractorSettings orb_params = views2.orbParams();

    for(auto it = cand_idx2.begin(); it != cand_idx2.end(); ++it) {
        size_t idx2 = *it;
        cv::KeyPoint kp2 = views2.keypt(idx2);
        float sigma_size_corrected = orb_params.determineSigma2(kp2.size);
        if (CheckDistEpipolarLine(kp1, kp2, Fmatrix, sigma_size_corrected)) {
            idxs_passed.push_back(idx2);
        }
    }

    return idxs_passed;
}

bool EpipolarConsistencyBoWCriterion::CheckDistEpipolarLine(const cv::KeyPoint &kp1,const cv::KeyPoint &kp2,const cv::Mat &F12, float sigma2)
{
    // Epipolar line in second image l = x1'F12 = [a b c]
    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return false;

    const float dsqr = num*num/den;

    return dsqr < 3.84* sigma2;
}


MatchesIdx RotationConsistencyBoW::apply(std::map<size_t, size_t> current_matches, const FeatureViews &views1, const FeatureViews &views2 ){

    return RotationConsistency(current_matches, views1, views2 );
}

MatchesIdx RotationConsistency(MatchesIdx current_matches, const FeatureViews &views_curr,  const FeatureViews &views_prev ){
    int HISTO_LENGTH = 30;
    MatchesIdx matches_passed = current_matches;

    std::vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    for(auto it = current_matches.begin(); it != current_matches.end(); ++it) {
        size_t idx_curr = it->first;
        size_t idx_prev = it->second;

        float rot = views_prev.keypt(idx_prev).angle - views_curr.keypt(idx_curr).angle;
        if (rot < 0.0)
            rot += 360.0f;
        int bin = round(rot * factor);
        if (bin == HISTO_LENGTH)
            bin = 0;
        assert(bin >= 0 && bin < HISTO_LENGTH);
        rotHist[bin].push_back(idx_curr);
    }


    int ind1=-1;
    int ind2=-1;
    int ind3=-1;

    ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

    for(int i=0; i<HISTO_LENGTH; i++)
    {
        if(i!=ind1 && i!=ind2 && i!=ind3)
        {
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                matches_passed.erase(rotHist[i][j]);
            }
        }
    }

    return matches_passed;
}
void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3){
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

}


