#ifndef TRACKREFERENCEKEYFRAME_H_
#define TRACKREFERENCEKEYFRAME_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>

namespace ORB_SLAM2{

class TrackReferenceKeyFrame : public TrackingStrategy{
public:
    TrackReferenceKeyFrame(optInfo optimizer_info_, const TrackReferenceKeyFrameParameters &params_);
    int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
private:
    optInfo optimizer_info;
    TrackReferenceKeyFrameParameters params;

};

}//end namespace
#endif