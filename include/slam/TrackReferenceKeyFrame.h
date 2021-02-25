#ifndef TRACKREFERENCEKEYFRAME_H_
#define TRACKREFERENCEKEYFRAME_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>

namespace ORB_SLAM2{

class TrackReferenceKeyFrame : public TrackingStrategy{
public:
    TrackReferenceKeyFrame(optInfo optimizer_info_);
    int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory);
private:
    optInfo optimizer_info;

};

}//end namespace
#endif