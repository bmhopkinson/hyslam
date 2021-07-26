#ifndef TRACKINGSTATENORMAL_H_
#define TRACKINGSTATENORMAL_H_

/*
 * TrackingState under normal conditions when tracking is maintained. 
 *
 *  initialPoseEstimation() - if a frame-to-frame velocity can be estimated from trajectory, uses this estimated velocity for more 
 *    computationally efficient and physically constrained tracking strategy: TrackMotionModel. if not uses TrackReferenceKeyFrame(), also 
 *    if TrackMotionModel fails reverts to attempting to TrackReferenceKeyFrame(). 
 *    considers tracking succesful if number of landmarks tracked exceeds a threshold (threshold can be set from parameters)
 *
 *  refinePoseEstimate() - uses TrackLocalMap strategy to refine pose estimate. 
 *     considers tracking succesful if number of landmarks tracked exceeds a threshold (threshold can be set from parameters)
 * 
 *  needNewKeyFrame() - considers several conditions/criteria for whether to insert a keyframe. [this logic can dramatically affect 
 *     tracking, map size, etc and has evolved over time - SHOULD BE CONSIDERED CRUCIAL and may want to make more flexible, state dependent, 
 *     for example by adding new tracking states that use different keyframe insertion logic]
 *     current: can force keyframe insertion with force = true. 
 * 				otherwise considers several criteria:
 *				1) how many landmarks are tracked relative to target number (settable in parameters)
 *				2) how many tracked landmarks are "close" - i.e. provide motion/position information rather than just orientation (distant)
 *              3) how long has it been (in number of frames) since a keyframe was last inserted. 
 * 		triggers a definite insert (regardless of whether mapping is idle) if number of landmarks tracked is very low or max keyframe insertion interval has been exceeded.
 *      triggers an optional insert (only if mapping is idle) if tracking is weak or lacking close landmarks (and minimum keyframe insertion interval exceeded)
 *      regardless of these triggers, will only actually request keyframe creation if mapping queue is not backed up (currently hardcoded at 3 keyframes- should make this user settable) 
 *      OR insertion is forced. 
 */

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
    void clear();

protected:
    virtual bool needNewKeyFrame(Frame &current_frame, Map* pMap, unsigned int last_keyframe_id, bool force);
    std::unique_ptr<TrackMotionModel> track_motion_model;
    std::unique_ptr<TrackReferenceKeyFrame> track_reference_keyframe;
    std::unique_ptr<TrackLocalMap> track_local_map;

    int n_frames_tracked_consecutively = 0;

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