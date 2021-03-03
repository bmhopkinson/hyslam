#ifndef TRACKREFERENCEKEYFRAME_H_
#define TRACKREFERENCEKEYFRAME_H_

#include <TrackingStrategy.h>
#include <src/main/ORBSLAM_datastructs.h>

namespace HYSLAM{

class TrackReferenceKeyFrame : public TrackingStrategy{
public:
    TrackReferenceKeyFrame(optInfo optimizer_info_);
    int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory);
private:
    optInfo optimizer_info;

};

}//end namespace
#endif