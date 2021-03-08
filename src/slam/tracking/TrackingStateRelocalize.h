#ifndef TRACKINGSTATERELOCALIZE_H_
#define TRACKINGSTATERELOCALIZE_H_

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <ORBSLAM_datastructs.h>
#include <TrackPlaceRecognition.h>
#include <TrackLocalMap.h>

#include <opencv2/core/core.hpp>

namespace HYSLAM{
class TrackingStateRelocalize : public TrackingState{
public:
    TrackingStateRelocalize(optInfo optimizer_info_, StateRelocalizeParameters params_, std::ofstream &log,  MainThreadsStatus* thread_status_);
    bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,   std::map< std::string, std::unique_ptr<Trajectory> > &trajectories); //signature mimics TrackingStrategy
    bool refinePoseEstimate( Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories);
private:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper, unsigned int last_keyframe_id, bool force);
    std::unique_ptr<TrackPlaceRecognition> track_place_recognition;
    std::unique_ptr<TrackLocalMap> track_local_map;

    StateRelocalizeParameters params;

};
}

#endif