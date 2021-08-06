#include <TrackingStateRelocalize.h>
#include <Tracking_datastructs.h>

namespace HYSLAM{

TrackingStateRelocalize::TrackingStateRelocalize(optInfo optimizer_info_, StateRelocalizeParameters params_, std::ofstream &log,
                                                 MainThreadsStatus* thread_status_, FeatureFactory* factory) :
TrackingState(log, thread_status_), params(params_), feature_factory(factory)
{
    track_place_recognition = std::make_unique<TrackPlaceRecognition>(optimizer_info_, params.tplacerecog_params, feature_factory);
    track_local_map = std::make_unique<TrackLocalMap>(optimizer_info_, params.tlocalmap_params, feature_factory);
}

bool TrackingStateRelocalize::initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,
                                                    std::map<std::string, std::shared_ptr<Trajectory>> &trajectories){
    int nmatches = track_place_recognition->track(current_frame, frames, pKF, pMap, trajectories.at("SLAM").get() );
    return nmatches > params.thresh_init;
}


bool TrackingStateRelocalize::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories){
    int nmatches =  track_local_map->track(current_frame, frames, pKF, pMap, trajectories.at("SLAM").get() );
    return nmatches > params.thresh_refine;
}

bool TrackingStateRelocalize::needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force){
    return false; //don't insert a keyframe immediately after relocalization
}


}//end namespace