#ifndef TRACKMOTIONMODEL_H_
#define TRACKMOTIONMODEL_H_

#include <TrackingStrategy.h>
#include <src/main/ORBSLAM_datastructs.h>

namespace HYSLAM{

    class TrackMotionModel : public TrackingStrategy{
    public:
        TrackMotionModel(optInfo optimizer_info_);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory);
    private:
        optInfo optimizer_info;

    };

}//end namespace
#endif