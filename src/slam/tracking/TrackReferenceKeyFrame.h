#ifndef TRACKREFERENCEKEYFRAME_H_
#define TRACKREFERENCEKEYFRAME_H_

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