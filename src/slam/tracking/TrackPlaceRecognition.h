#ifndef TRACKPLACERECOGNITION_H_
#define TRACKPLACERECOGNITION_H_

/*
 * TrackingStrategy that attempts to track (determine pose and landmark associations) current_frame by finding keyframes
 *   with visual similarity to current_frame and then using a fairly complicated procedure to identify geometrically valid
 *   landmark matches and optimize pose.
 *
 *   track() - queries keyframe visual database (DBoW2) with current_frame to obtain potential keyframe matches.
 *   for each potential keyframe match, first attempts to establish feature coorespondences using FeatureMatcher::SearchByBoW().
 *   For each keyframe match with sufficient feature correspondences, runs a PnP solver (with RANSAC for robustness) to establish
 *   geometrically valid matches. if sufficient matches are found, optimize pose (PoseOptimization) and then find additional
 *   landmark matches by alternately running FeatureMatcher::SearchByProjection(), PoseOptimization.
 *   tracking is successful if sufficient matches are found.
 */

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>
#include <FeatureFactory.h>

namespace HYSLAM{

    class TrackPlaceRecognition : public TrackingStrategy{
    public:
        TrackPlaceRecognition(optInfo optimizer_info_,TrackPlaceRecognitionParameters params_, FeatureFactory* factory );
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
    private:
        optInfo optimizer_info;
        TrackPlaceRecognitionParameters params;
        FeatureFactory* feature_factory;
    };

}//end namespace
#endif