#ifndef TRACKMOTIONMODEL_H_
#define TRACKMOTIONMODEL_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>

namespace ORB_SLAM2{

    class TrackMotionModel : public TrackingStrategy{
    public:
        TrackMotionModel(optInfo optimizer_info_, const TrackMotionModelParameters &params_);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
    private:
        optInfo optimizer_info;
        TrackMotionModelParameters params;
    };

}//end namespace
#endif