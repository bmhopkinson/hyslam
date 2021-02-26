#ifndef TRACKPLACERECOGNITION_H_
#define TRACKPLACERECOGNITION_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>

namespace HYSLAM{

    class TrackPlaceRecognition : public TrackingStrategy{
    public:
        TrackPlaceRecognition(optInfo optimizer_info_);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory);
    private:
        optInfo optimizer_info;

    };

}//end namespace
#endif