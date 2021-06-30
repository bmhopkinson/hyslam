//
// Created by cv-bhlab on 5/17/21.
//

#include "TrackingStateNull.h"

namespace HYSLAM {

TrackingStateNull::TrackingStateNull(std::ofstream &log, MainThreadsStatus *thread_status_) :
    TrackingState(log, thread_status_)
    {
    }

bool TrackingStateNull::initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF, Map *pMap,
                                              std::map <std::string, std::unique_ptr<Trajectory>> &trajectories) {
    return false;
}

bool TrackingStateNull::refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF, Map *pMap,
                                           std::map <std::string, std::unique_ptr<Trajectory>> &trajectories) {
    return false;
}

std::vector<KeyFrame *> TrackingStateNull::newKeyFrame(Frame &current_frame, Map *pMap, unsigned int last_keyframe_id, bool force) {
    return std::vector<KeyFrame *>();
}

bool TrackingStateNull::needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force){
    return false;
}

} //end namespace