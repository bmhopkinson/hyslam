#ifndef TRACKINGSTATERELOCALIZE_H_
#define TRACKINGSTATERELOCALIZE_H_

/*
 * TrackingState when tracking has been lost and a relocalization based on image appearance is being attempted
 *
 * initialPoseEstimation() uses TrackPlaceRecognition strategy to attempt to relocalize and obtain pose for current_frame.
 *   considers tracking successful if number of landmarks tracked exceeds a threshold (threshold can be set from parameters)
 *
 * refinePoseEstimate() - uses TrackLocalMap strategy to refine pose estimate.
 *   considers tracking successful if number of landmarks tracked exceeds a threshold (threshold can be set from parameters)
 *
 * needNewKeyFrame() - currently returns false - don't immediately create a new keyframe after relocalization (but haven't tried true,
 *    it might make sense to insert a keyframe immediately upon successful relocalization )
 *
 */

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <InterThread.h>
#include <TrackPlaceRecognition.h>
#include <TrackLocalMap.h>
#include <FeatureFactory.h>

#include <opencv2/core/core.hpp>

namespace HYSLAM{
class TrackingStateRelocalize : public TrackingState{
public:
    TrackingStateRelocalize(optInfo optimizer_info_, StateRelocalizeParameters params_, std::ofstream &log,
                            MainThreadsStatus* thread_status_, FeatureFactory* factory);
    bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories); //signature mimics TrackingStrategy
    bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories);
    void clear(){};
private:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force);
    std::unique_ptr<TrackPlaceRecognition> track_place_recognition;
    std::unique_ptr<TrackLocalMap> track_local_map;

    StateRelocalizeParameters params;
    FeatureFactory* feature_factory;

};
}

#endif