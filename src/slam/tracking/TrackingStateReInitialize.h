//
// Created by cv-bhlab on 7/14/21.
//

#ifndef HYSLAM_TRACKINGSTATEREINITIALIZE_H
#define HYSLAM_TRACKINGSTATEREINITIALIZE_H

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <Initializer.h>
#include <FeatureFactory.h>
#include <InterThread.h>
#include <Camera.h>

namespace HYSLAM {

class TrackingStateReInitialize : public TrackingState {
public:
    TrackingStateReInitialize(optInfo optimizer_info_, Camera camera_, InitializerData &init_data_,
            StateReInitializeParameters params_,  std::ofstream &log, MainThreadsStatus* thread_status_,
            FeatureFactory* factory);

    bool initialPoseEstimation(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories); //signature mimics TrackingStrategy
    bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, std::map<std::string, std::shared_ptr<Trajectory>> &trajectories);
    void clear();

private:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap,  unsigned int last_keyframe_id, bool force);
    std::vector<KeyFrame*> createNewKeyFrame(Frame &current_frame, Map* pMap);

    std::unique_ptr<Initializer> initializer;
    optInfo optimizer_info;
    StateReInitializeParameters params;
    Camera camera;
    InitializerData* init_data;
    FeatureFactory* feature_factory;
    bool success;
    std::vector<KeyFrame*> KFnew;
};

}
#endif //HYSLAM_TRACKINGSTATEREINITIALIZE_H
