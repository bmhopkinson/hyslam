#ifndef TRACKREFERENCEKEYFRAME_H_
#define TRACKREFERENCEKEYFRAME_H_

/*
 * TrackingStrategy that attempts to track (determine pose and landmark associations) current_frame by matching features
 *   with reference keyframe (pKF)
 *
 *   track() - establishes feature matches between current_frame and reference keyframe (pKF) with FeatureMatcher::SearchByBoW().
 *     if a sufficient number of landmark matches are made to the current_frame, runs PoseOptimization to refine pose estimate
 *     (sets initial pose to that of the previous frame) based on landmark to image feature correspondences.
 *     subsequently removes outlier landmark associations and returns number of remaining associations (inlier matches).
 *
 */

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>
#include <FeatureFactory.h>

namespace HYSLAM{

class TrackReferenceKeyFrame : public TrackingStrategy{
public:
    TrackReferenceKeyFrame(optInfo optimizer_info_, const TrackReferenceKeyFrameParameters &params_, FeatureFactory* factory);
    int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
private:
    optInfo optimizer_info;
    TrackReferenceKeyFrameParameters params;
    FeatureFactory* feature_factory;

};

}//end namespace
#endif