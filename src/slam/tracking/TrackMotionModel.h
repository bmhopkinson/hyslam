#ifndef TRACKMOTIONMODEL_H_
#define TRACKMOTIONMODEL_H_

/*
 * TrackingStrategy that attempts to track (determine pose and landmark associations) current_frame based on recent motion estimate
 *
 * track() - for SLAM camera: uses constant motion model to estimate current position, for other cameras, uses SLAM camera's trajectory
 *   to estimate current position (SLAM camera time should always preceed other cameras time - that is most recent frame is always a SLAM frame
 *   frames from other cameras are not tracked until at least a little bit after SLAM).
 *   once the current position has been estimated attempts to propogate matches from the previous frame into current frame with
 *     FeatureMatcher::SearchByProjection(). If not enough landmark matches are found in a first pass, a second SearchByProjection()
 *     is attempted with a wider search window.
 *    if a sufficient number of landmark matches are made to the current_frame, runs PoseOptimization to refine pose estimate based on landmark to image feature correspondences.
 *    subsequently removes outlier landmark associations and returns number of remaining associations (inlier matches).
 */

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>
#include <FeatureFactory.h>

namespace HYSLAM{

    class TrackMotionModel : public TrackingStrategy{
    public:
        TrackMotionModel(optInfo optimizer_info_, const TrackMotionModelParameters &params_, FeatureFactory* factory);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
    private:
        optInfo optimizer_info;
        TrackMotionModelParameters params;
        FeatureFactory* feature_factory;
    };

}//end namespace
#endif