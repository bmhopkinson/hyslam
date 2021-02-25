#ifndef TRACKPLACERECOGNITION_H_
#define TRACKPLACERECOGNITION_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>

namespace ORB_SLAM2{

    class TrackPlaceRecognition : public TrackingStrategy{
    public:
        TrackPlaceRecognition(optInfo optimizer_info_);
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, const Trajectory &trajectory);
    private:
        optInfo optimizer_info;

    };

}//end namespace
#endif