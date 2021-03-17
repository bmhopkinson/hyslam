#ifndef TRACKINGSTATENORMAL_H_
#define TRACKINGSTATENORMAL_H_

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <InterThread.h>
#include <TrackMotionModel.h>
#include <TrackReferenceKeyFrame.h>
#include <TrackLocalMap.h>
#include <FeatureFactory.h>

#include <opencv2/core/core.hpp>
#include <chrono>

namespace HYSLAM {
class TrackingStateNormal : public TrackingState {
public:
    TrackingStateNormal(optInfo optimizer_info_, StateNormalParameters params_, std::ofstream &log,
                        MainThreadsStatus* thread_status_, FeatureFactory* factory);
    bool initialPoseEstimation( Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories); //signature mimics TrackingStrategy
    bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories);

private:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force);
    std::unique_ptr<TrackMotionModel> track_motion_model;
    std::unique_ptr<TrackReferenceKeyFrame> track_reference_keyframe;
    std::unique_ptr<TrackLocalMap> track_local_map;

    StateNormalParameters params;
    int thresh_init;
    int thresh_refine;
    int mnMatchesInliers = 0; // number of tracked points after refinePoseEstimate

    FeatureFactory* feature_factory;

    //timing
    static int n_calls;
    static std::chrono::duration<int, std::milli> init_pose_duration;
    static std::chrono::duration<int, std::milli> refine_pose_duration;
};

}

#endif