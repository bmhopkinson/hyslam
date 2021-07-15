//
// Created by cv-bhlab on 5/17/21.
//

#ifndef HYSLAM_TRACKINGSTATENULL_H
#define HYSLAM_TRACKINGSTATENULL_H

/*
 * a no-op tracking state - don't attempt to track
 */

#include <TrackingState.h>
namespace HYSLAM {

class TrackingStateNull : public TrackingState {
public:

    TrackingStateNull(std::ofstream &log, MainThreadsStatus *thread_status_);

    bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF, Map *pMap,
                               std::map <std::string, std::unique_ptr<Trajectory>> &trajectories); //signature mimics TrackingStrategy
    bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame *pKF, Map *pMap,
                            std::map <std::string, std::unique_ptr<Trajectory>> &trajectories);

    std::vector<KeyFrame *> newKeyFrame(Frame &current_frame, Map *pMap, unsigned int last_keyframe_id,
                                        bool force); //template method using needNewKeyFrame, createNewKeyFrame
    void clear(){};

private:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force);
};
} //close namespace

#endif //HYSLAM_TRACKINGSTATENULL_H
