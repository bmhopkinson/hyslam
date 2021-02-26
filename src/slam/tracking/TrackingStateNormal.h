#ifndef TRACKINGSTATENORMAL_H_
#define TRACKINGSTATENORMAL_H_

#include <TrackingState.h>
#include <Tracking_datastructs.h>
#include <ORBSLAM_datastructs.h>
#include <TrackMotionModel.h>
#include <TrackReferenceKeyFrame.h>
#include <TrackLocalMap.h>

#include <opencv2/core/core.hpp>

namespace HYSLAM {
class TrackingStateNormal : public TrackingState {
public:
    TrackingStateNormal(optInfo optimizer_info_, StateNormalParameters params_, std::ofstream &log);
    bool initialPoseEstimation( Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories); //signature mimics TrackingStrategy
    bool refinePoseEstimate(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap,  std::map< std::string, std::unique_ptr<Trajectory> > &trajectories);

private:
    bool needNewKeyFrame(Frame &current_frame, Map* pMap, Mapping* pLocalMapper, unsigned int last_keyframe_id, bool force);
    std::unique_ptr<TrackMotionModel> track_motion_model;
    std::unique_ptr<TrackReferenceKeyFrame> track_reference_keyframe;
    std::unique_ptr<TrackLocalMap> track_local_map;

    StateNormalParameters params;
    int thresh_init;
    int thresh_refine;
    int mnMatchesInliers = 0; // number of tracked points after refinePoseEstimate

};

}

#endif