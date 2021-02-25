#ifndef TRACKPLACERECOGNITION_H_
#define TRACKPLACERECOGNITION_H_

#include <TrackingStrategy.h>
#include <ORBSLAM_datastructs.h>
#include <Tracking_datastructs.h>

namespace ORB_SLAM2{

    class TrackPlaceRecognition : public TrackingStrategy{
    public:
        TrackPlaceRecognition(optInfo optimizer_info_,TrackPlaceRecognitionParameters params_ );
        int track(Frame &current_frame, const FrameBuffer &frames, KeyFrame* pKF, Map* pMap, Trajectory* trajectory);
    private:
        optInfo optimizer_info;
        TrackPlaceRecognitionParameters params;
    };

}//end namespace
#endif